#include "ace-route.h"
#include <chrono>
#include <zlib.h>

#include <log4cplus/logger.h>
#include <log4cplus/loggingmacros.h>

using namespace std;
using namespace std::chrono;

static log4cplus::Logger logger = log4cplus::Logger::getInstance("ace.route");

static auto elapsed_seconds = [](steady_clock::time_point last_time) -> double {
  return duration_cast<duration<double>>(steady_clock::now() - last_time)
      .count();
};

void AceRoute::prepare_write() {
  auto prepare = [&](const std::vector<ConnectionPtr> &connections) {
    for (ConnectionPtr connection : connections) {
      if (!connection->sink()->is_routed()) {
        LOG4CPLUS_ERROR_FMT(logger, "Net %s : %s connection (%d,%d) not routed",
                            str_list[connection->net()->id()].c_str(),
                            connection->is_direct() ? "direct" : "indirect",
                            connection->source()->id(),
                            connection->sink()->id());
        continue;
      }
      auto rnodes = connection->get_rnodes();
      for (size_t i = 0; i + 1 < rnodes.size(); i++) {
        int prev_id = rnodes[i + 1]->id();
        int next_id = rnodes[i]->id();
        int net_id = connection->net()->id();
        node_to_net_to_next[prev_id][net_id].insert(next_id);
      }
    }
  };

  prepare(connections);
  prepare(direct_connections);
}

void AceRoute::write() {
  auto last_time = steady_clock::now();
  capnp::MallocMessageBuilder message_builder;
  message_builder.initRoot<PhysicalNetlist::PhysNetlist>();
  message_builder.setRoot(std::move(netlist));
  auto netlist_builder =
      message_builder.getRoot<PhysicalNetlist::PhysNetlist>();

  auto str_list = netlist_builder.getStrList();
  for (auto str : str_list) {
    get_new_str_id(str.cStr());
  }
  int pip_num = 0;
  for (auto net : netlist_builder.getPhysNets()) {
    int net_name_id = net.getName();
    // LOG4CPLUS_DEBUG(logger, "writing net " << net_name_id);
    if (!id_to_netinfo.contains(net_name_id))
      continue;
    NetInfo net_info = id_to_netinfo[net_name_id];
    if (!net_info.is_need_route())
      continue;
    unordered_map<SitePin, PhysicalNetlist::PhysNetlist::RouteBranch::Reader,
                  SitePinHasher>
        sink_to_rb;
    SitePin t;
    auto stubs = net.disownStubs();
    for (auto rb : stubs.getReader()) {
      auto rs = rb.getRouteSegment();
      assert(rs.isSitePin());
      auto sp = rs.getSitePin();
      SitePin sp_name(str_list[sp.getSite()].cStr(),
                      str_list[sp.getPin()].cStr());
      sink_to_rb[sp_name] = rb;
      t = sp_name;
      //   LOG4CPLUS_INFO_FMT(
      //       logger, "test0 %d",
      //       sink_to_rb[t].getRouteSegment().getSitePin().getSite());
      //   break;
    }
    // LOG4CPLUS_INFO_FMT(logger, "test1 %d",
    //                    sink_to_rb[t].getRouteSegment().getSitePin().getSite());

    std::queue<PhysicalNetlist::PhysNetlist::RouteBranch::Builder> q;
    for (auto source : net.getSources()) {
      q.push(source);
    }
    while (!q.empty()) {
      auto rb = q.front();
      q.pop();
      for (auto next_branch : rb.getBranches()) {
        q.push(next_branch);
      }
      auto rs = rb.getRouteSegment();
      if (!rs.isSitePin())
        continue;
      auto sp = rs.getSitePin();
      SitePin sp_name(str_list[sp.getSite()].cStr(),
                      str_list[sp.getPin()].cStr());
      // int source_node = net_info.source_sp_to_node[sp_name];
      int source_node = rg.get_node_from_site_pin(sp_name);
      auto &net_to_next = node_to_net_to_next[source_node];
      if (!net_to_next.contains(net_name_id))
        continue;

      std::queue<
          std::pair<PhysicalNetlist::PhysNetlist::RouteBranch::Builder, int>>
          q2;
      q2.push(make_pair(rb, source_node));
      while (!q2.empty()) {
        auto [rb, cur_node] = q2.front();
        q2.pop();
        assert(rb.getBranches().size() == 0);
        set<int> next_nodes = node_to_net_to_next[cur_node][net_name_id];
        SitePin sp = rg.get_site_pin_from_node(cur_node);
        capnp::List<PhysicalNetlist::PhysNetlist::RouteBranch>::Builder
            new_branches;
        if (sink_to_rb.contains(sp)) {
          new_branches = rb.initBranches(next_nodes.size() + 1);
          new_branches.setWithCaveats(next_nodes.size(), sink_to_rb[sp]);
          sink_to_rb.erase(sp);
        } else {
          new_branches = rb.initBranches(next_nodes.size());
        }
        int i = 0;
        for (auto next_node : next_nodes) {
          auto next_rb = new_branches[i];
          auto pip = next_rb.getRouteSegment().initPip();
          PipData pip_data = rg.get_pip_data_from_nodes(cur_node, next_node);
          pip.setTile(get_new_str_id(rg.name(pip_data.tile_name_id)));
          pip.setWire0(get_new_str_id(rg.name(pip_data.wire0_name_id)));
          pip.setWire1(get_new_str_id(rg.name(pip_data.wire1_name_id)));
          pip.setForward(pip_data.is_forward);
          pip_num++;
          q2.push(make_pair(next_rb, next_node));

          //   LOG4CPLUS_TRACE_FMT(
          //       logger, "PIP: in tile %s, wire %s->%s, forward=%d",
          //       rg.name(pip_data.tile_name_id).c_str(),
          //       rg.name(pip_data.wire0_name_id).c_str(),
          //       rg.name(pip_data.wire1_name_id).c_str(),
          //       pip_data.is_forward);

          i++;
        }
      }
    }
    if (!sink_to_rb.empty()) {
      //   for (auto [sp, rb] : sink_to_rb) {
      //     LOG4CPLUS_DEBUG(logger, "unrouted sink: " << sp);
      //   }
      auto new_stubs = net.initStubs(sink_to_rb.size());
      int i = 0;
      for (auto &[_, rb] : sink_to_rb) {
        new_stubs.setWithCaveats(i++, rb);
      }
    }
  }
  int old_str_list_size = netlist_builder.getStrList().size();
  auto new_str_list = netlist_builder.initStrList(str_to_id.size());
  for (auto [str, i] : str_to_id) {
    capnp::Text::Reader str_reader(str);
    new_str_list.set(i, str);
  }
  LOG4CPLUS_INFO_FMT(logger, "Insert %d PIPs and %d strings: %lfs", pip_num,
                     new_str_list.size() - old_str_list_size,
                     elapsed_seconds(last_time));
  last_time = steady_clock::now();

  kj::Array<capnp::word> array = capnp::messageToFlatArray(message_builder);
  kj::ArrayPtr<kj::byte> byteStream = array.asBytes();
  gzFile file = gzopen(out_filename.c_str(), "wb");
  if (file == NULL) {
    LOG4CPLUS_ERROR(logger, "gzopen " << out_filename << " failed");
  } else {
    const uint32_t max_chunk_size = 1000000000;
    uint64_t total_size = byteStream.size();
    uint64_t written = 0;
    while (written < total_size) {
      // 计算这一块的大小
      uint32_t chunk_size = static_cast<uint32_t>(
          std::min(static_cast<size_t>(max_chunk_size), total_size - written));
      // 写入这一块的数据
      if (gzwrite(file, &byteStream[written], chunk_size) != (int)chunk_size) {
        LOG4CPLUS_ERROR(logger, "gzwrite " << out_filename << " failed");
      }
      written += chunk_size;
    }
    gzclose(file);
  }
  LOG4CPLUS_INFO_FMT(logger, "Write to gzip file: %lfs",
                     elapsed_seconds(last_time));
}
