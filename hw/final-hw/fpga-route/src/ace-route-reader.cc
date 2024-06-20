#include "ace-route.h"
#include <zlib.h>

#include <log4cplus/logger.h>
#include <log4cplus/loggingmacros.h>

using namespace std;

static log4cplus::Logger logger = log4cplus::Logger::getInstance("ace.route");

vector<PhysicalNetlist::PhysNetlist::PhysSitePin::Reader>
AceRoute::extract_sitepins(
    capnp::List<PhysicalNetlist::PhysNetlist::RouteBranch,
                capnp::Kind::STRUCT>::Reader branches,
    /*bool is_sink,*/ NetInfo &net_info,
    std::set<std::pair<uint32_t, uint32_t>> &stub_wires) {
  vector<PhysicalNetlist::PhysNetlist::PhysSitePin::Reader> sitepins;
  for (auto branch : branches) {
    queue<pair<PhysicalNetlist::PhysNetlist::RouteBranch::Reader, int>> q;
    q.push(make_pair(branch, 0));
    while (!q.empty()) {
      auto [branch, depth] = q.front();
      q.pop();
      auto route_segment = branch.getRouteSegment();
      if (route_segment.isSitePin()) {
        sitepins.push_back(route_segment.getSitePin());
        // if (is_sink)
        //   break;
      } else if (route_segment.which() ==
                 PhysicalNetlist::PhysNetlist::RouteBranch::RouteSegment::
                     Which::PIP) {
        auto pip_reader = route_segment.getPip();

        auto tile_id = dev->id(str_list[pip_reader.getTile()]);
        auto wire0_id = dev->id(str_list[pip_reader.getWire0()]);
        auto wire1_id = dev->id(str_list[pip_reader.getWire1()]);

        if (dev->tiles().contains(tile_id) && dev->wires().contains(wire0_id) &&
            dev->wires().contains(wire1_id)) {

          auto tile = dev->tiles().at(tile_id);
          auto wire0 = dev->wires().at(wire0_id);
          auto wire1 = dev->wires().at(wire1_id);

          bool is_directional = pip_reader.getForward();

          auto pip = device::PipInst(tile, wire0, wire1, is_directional);
          pip.set_fixed(pip_reader.getIsFixed());

          if (stub_wires.contains(
                  std::make_pair(tile->name(), wire1->name()))) {
            stub_wires.erase(std::make_pair(tile->name(), wire1->name()));
            pip.set_stub(true);
          }
          net_info.add_pip(pip);
        } else {
          //! bug: wire 221498 is not found
          LOG4CPLUS_TRACE_FMT(logger,
                              "WARN: Tile %d or wire %d or wire %d not found",
                              pip_reader.getTile(), pip_reader.getWire0(),
                              pip_reader.getWire1());
        }
      }
      for (auto next_branch : branch.getBranches())
        q.push(make_pair(next_branch, depth + 1));
    }
  }
  return sitepins;
};

void AceRoute::parse_netlist() {
  using namespace std::chrono;
  steady_clock::time_point last_time = steady_clock::now();
  auto elapsed_seconds = [](steady_clock::time_point last_time) -> double {
    return duration_cast<duration<double>>(steady_clock::now() - last_time)
        .count();
  };

  kj::ArrayPtr<const kj::byte> arr(
      reinterpret_cast<const kj::byte *>(netlist_file->data()),
      netlist_file->size());
  stream = make_unique<kj::ArrayInputStream>(arr);

  capnp::ReaderOptions options;
  options.traversalLimitInWords = 10000000000LL;
  options.nestingLimit = 1000;

  message = make_unique<capnp::InputStreamMessageReader>(*stream, options);
  netlist = message->getRoot<PhysicalNetlist::PhysNetlist>();

  for (string s : netlist.getStrList())
    str_list.push_back(s);

  int net_num = 0, source_num = 0, sink_num = 0;
  for (auto net : netlist.getPhysNets()) {
    NetInfo net_info;
    if (net.getType() == PhysicalNetlist::PhysNetlist::NetType::GND) {
      net_info.set_type(NetInfo::Type::GND);
    } else if (net.getType() == PhysicalNetlist::PhysNetlist::NetType::VCC) {
      net_info.set_type(NetInfo::Type::VCC);
    } else {
      net_info.set_type(NetInfo::Type::WIRE);
    }

    std::set<std::pair<uint32_t, uint32_t>> stub_wires;
    if (net.hasStubNodes()) {
      for (auto stub_node : net.getStubNodes()) {
        stub_wires.insert(make_pair(stub_node.getTile(), stub_node.getWire()));
      }
    }

    // auto sink_pins = extract_sitepins(net.getStubs(), /*is_sink=*/true,
    //                                   net_info, stub_wires);
    // auto source_pins = extract_sitepins(net.getSources(), /*is_sink=*/false,
    //                                     net_info, stub_wires);
    auto sink_pins = extract_sitepins(net.getStubs(), net_info, stub_wires);
    auto source_pins = extract_sitepins(net.getSources(), net_info, stub_wires);
    auto &source_sp_to_node = net_info.source_sp_to_node;
    auto &sink_sp_to_node = net_info.sink_sp_to_node;
    for (auto source_pin : source_pins) {
      string site_name = str_list.at(source_pin.getSite());
      string pin_name = str_list.at(source_pin.getPin());
      SitePin sp(site_name, pin_name);
      int node = rg.get_node_from_site_pin(sp);
      if (node != -1)
        source_sp_to_node.push_back(make_pair(sp, node));
    }
    for (auto sink_pin : sink_pins) {
      string site_name = str_list.at(sink_pin.getSite());
      string pin_name = str_list.at(sink_pin.getPin());
      auto sp = SitePin(site_name, pin_name);
      int node_id = rg.get_node_from_site_pin(sp);
      if (node_id == -1)
        continue;
      sink_sp_to_node.push_back(make_pair(sp, node_id));
      // remove all children
      rg.node_info(node_id).downhill_dev_nodes.clear();
    }

    if (str_list[net.getName()] != "GLOBAL_USEDNET" &&
        net.getType() == PhysicalNetlist::PhysNetlist::NetType::SIGNAL &&
        net.getStubs().size() > 0 && net.getSources().size() > 0) {
      if (source_sp_to_node.empty() || sink_sp_to_node.empty())
        continue;
      net_info.set_need_route(true);
      net_num++;
      source_num += source_sp_to_node.size();
      sink_num += sink_sp_to_node.size();
    } else {
      net_info.set_need_route(false);
      // see https://github.com/Xilinx/fpga24_routing_contest/discussions/54
      unordered_set<device::NodePtr> nodes;
      for (auto pip : net_info.pips) {
        auto tile = pip.tile();
        auto wire0 = pip.wire0();
        auto wire1 = pip.wire1();
        if (tile->wire_to_node().contains(wire0)) {
          nodes.insert(tile->wire_to_node().at(wire0));
        }
        if (tile->wire_to_node().contains(wire1)) {
          nodes.insert(tile->wire_to_node().at(wire1));
        }
      }
      for (auto node : nodes) {
        // Don't get_or_create()
        rg.remove_node(node->id());
      }
      // for (auto [_, node_id] : source_sp_to_node) {
      //   rg.get_or_create(node_id);
      //   rg.remove_node(node_id);
      // }
      // for (auto [_, node_id] : sink_sp_to_node) {
      //   rg.get_or_create(node_id);
      //   rg.remove_node(node_id);
      // }
      // continue;
    }

    // Stub nodes that don't belong on a PIP
    for (auto [tile_id, wire_id] : stub_wires) {
      // null-end PIP
      net_info.add_pip(
          device::PipInst(dev->tiles().at(tile_id), dev->wires().at(wire_id)));
    }
    id_to_netinfo[net.getName()] = net_info;
  }
  LOG4CPLUS_INFO_FMT(logger, "Build %d/%d nets, %d sources, %d sinks: %.2lfs",
                     net_num, netlist.getPhysNets().size(), source_num,
                     sink_num, elapsed_seconds(last_time));
}
