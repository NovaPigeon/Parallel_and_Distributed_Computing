#include "device.h"
#include "utils.h"
#include <cassert>
#include <chrono>
#include <omp.h>
#include <regex>
#include <string>

#include <log4cplus/logger.h>
#include <log4cplus/loggingmacros.h>

using namespace std;

static log4cplus::Logger logger = log4cplus::Logger::getInstance("ace.device");

namespace device {

std::unordered_map<uint32_t, SiteTypePtr> TileType::site_type_lookup;

void TileType::build(
    DeviceResources::Device::Reader fpgaif_device,
    DeviceResources::Device::TileType::Reader fpgaif_tile_type) {
  std::unordered_map<uint32_t, WirePtr> wire_lookup;
  for (auto wire : wires_) {
    wire_lookup[wire->name()] = wire;
  }

  for (uint32_t site_type_inst_id = 0;
       site_type_inst_id < fpgaif_tile_type.getSiteTypes().size();
       site_type_inst_id++) {
    auto fpgaif_site_type_in_tile_type =
        fpgaif_tile_type.getSiteTypes()[site_type_inst_id];
    uint32_t primary_type_id = fpgaif_site_type_in_tile_type.getPrimaryType();
    auto fpgaif_primary_type = fpgaif_device.getSiteTypeList()[primary_type_id];

    auto site_type = [&]() {
      SiteTypePtr ret = nullptr;
      if (TileType::site_type_lookup.contains(primary_type_id)) {
        ret = TileType::site_type_lookup.at(primary_type_id);
      } else {
        std::vector<SitePinPtr> site_pins;
        for (auto fpgaif_site_pin : fpgaif_primary_type.getPins()) {
          site_pins.push_back(SitePin::create(fpgaif_site_pin.getName()));
        }
        ret = SiteType::create(fpgaif_primary_type.getName(), site_pins);
        TileType::site_type_lookup[primary_type_id] = ret;
      }
      return ret;
    }();

    std::vector<WirePtr> pin_wires;
    for (auto pin_wire_id :
         fpgaif_site_type_in_tile_type.getPrimaryPinsToTileWires()) {
      assert(wire_lookup.contains(pin_wire_id));
      pin_wires.push_back(wire_lookup.at(pin_wire_id));
    }
    site_type_insts_.push_back(SiteTypeInst::create(site_type, pin_wires));
  }

  for (auto [wire0, wire1_map] : pips_.container) {
    for (auto [wire1, is_directional] : wire1_map) {
      if (is_directional) {
        wire_to_downhill_wires_[wire0].push_back(wire1);
        wire_to_uphill_wires_[wire1].push_back(wire0);
      } else {
        wire_to_downhill_wires_[wire0].push_back(wire1);
        wire_to_downhill_wires_[wire1].push_back(wire0);

        wire_to_uphill_wires_[wire0].push_back(wire1);
        wire_to_uphill_wires_[wire1].push_back(wire0);
      }
    }
  }

  struct WireLess {
    bool operator()(const WirePtr &wire0, const WirePtr &wire1) const {
      return wire0->name() < wire1->name();
    }
  };
  for (auto &wire : wire_to_downhill_wires_) {
    std::sort(wire.second.begin(), wire.second.end(), WireLess());
  }
  for (auto &wire : wire_to_uphill_wires_) {
    std::sort(wire.second.begin(), wire.second.end(), WireLess());
  }
}

Tile::Tile(DeviceResources::Device::Reader fpgaif_device,
           DeviceResources::Device::Tile::Reader fpgaif_tile, TileTypePtr type_,
           std::string tile_name)
    : name_(fpgaif_tile.getName()), type_(type_), x_(0), y_(0) {
  static const regex pattern("([A-Z0-9_]+)_X(\\d+)Y(\\d+)");
  smatch match;
  if (regex_search(tile_name, match, pattern)) {
    x_ = stoi(match[2]);
    y_ = stoi(match[3]);
    // LOG4CPLUS_DEBUG(logger, "Tile name: " << name_ << " x: " << x_ << " y: "
    // << y_);
  } else {
    // Handle case where there is no match
    LOG4CPLUS_ERROR(logger, "Input string does not match the pattern.");
  }

  for (const auto &fpgaif_site : fpgaif_tile.getSites()) {
    const auto &fpgaif_tile_type =
        fpgaif_device.getTileTypeList()[fpgaif_tile.getType()];
    auto primary_type_id =
        fpgaif_tile_type.getSiteTypes()[fpgaif_site.getType()].getPrimaryType();
    assert(TileType::site_type_lookup.contains(primary_type_id));
    sites_.push_back(Site::create(
        fpgaif_site.getName(), TileType::site_type_lookup.at(primary_type_id),
        type()->site_type_insts()[fpgaif_site.getType()]));
  }
}

void Device::build() {
  using namespace std::chrono;
  steady_clock::time_point last_time = steady_clock::now();
  auto elapsed_seconds = [](steady_clock::time_point last_time) -> double {
    return duration_cast<duration<double>>(steady_clock::now() - last_time)
        .count();
  };

  DeviceResources::Device::Reader fpgaif_device(
      message.getRoot<DeviceResources::Device>());

  for (const auto &s : fpgaif_device.getStrList()) {
    str_list.emplace_back(s.cStr());
    str_to_id[string(s.cStr())] = str_list.size() - 1;
  }

  const auto &fpgaif_wires = fpgaif_device.getWires();
  const auto &fpgaif_nodes = fpgaif_device.getNodes();

  LOG4CPLUS_INFO_FMT(logger, "Read DeviceResources: %.2lfs, %.2lfGB RAM",
                     elapsed_seconds(last_time),
                     getMemUsage() / 1024.0 / 1024.0 / 1024.0);

  last_time = steady_clock::now();

  for (const auto &fpgaif_tile_type : fpgaif_device.getTileTypeList()) {
    auto tile_type_name = name(fpgaif_tile_type.getName());

    std::vector<pair<device::WirePtr, IntentCode>> wire_intentcode;
    for (auto wire_id : fpgaif_tile_type.getWires()) {
      auto wire = [&]() {
        if (wires_.contains(wire_id)) {
          return wires_.at(wire_id);
        } else {
          return wires_[wire_id] = device::Wire::create(wire_id);
        }
      }();
      auto wire_type =
          fpgaif_device.getWireTypes()[fpgaif_wires[wire_id].getType()];
      auto intentcode = Wire::intentcode(
          name(wire_id), name(wire_type.getName()), tile_type_name);
      wire_intentcode.push_back(make_pair(wire, intentcode));
    }

    device::PipContainer::Type pip_container;
    bool is_cle_or_rclk_type =
        tile_type_name.find("CLE") == 0 || tile_type_name.find("RCLK") == 0;
    for (const auto &fpgaif_pip : fpgaif_tile_type.getPips()) {
      auto wire0 = wire_intentcode[fpgaif_pip.getWire0()].first;
      auto wire1 = wire_intentcode[fpgaif_pip.getWire1()].first;
      if (!fpgaif_pip.isConventional()) {
        if (is_cle_or_rclk_type) {
          LOG4CPLUS_TRACE_FMT(
              logger, "Tile type %s : Skip unconventional PIP %s - %s",
              tile_type_name.c_str(), name(wire0).c_str(), name(wire1).c_str());
          // From `nxroute-poc.py`:
          // # Ignore non-conventional PIPs on CLE tiles
          // # (LUT route-thrus that traverse an entire site)
          // # and on RCLK tiles (BUFCE route-thrus that access
          // # the global routing network)
          // also see https://github.com/Xilinx/fpga24_routing_contest/pull/51
          continue;
        } else {
          LOG4CPLUS_TRACE_FMT(
              logger, "Tile type %s : Keep unconventional PIP %s - %s",
              tile_type_name.c_str(), name(wire0).c_str(), name(wire1).c_str());
        }
      }
      pip_container[wire0][wire1] = fpgaif_pip.getDirectional();
    }
    bool is_INT = (tile_type_name == "INT");
    auto new_tile_type =
        device::TileType::create(fpgaif_device, fpgaif_tile_type,
                                 wire_intentcode, pip_container, is_INT);
    tile_types_.push_back(new_tile_type);
  }

  for (const auto &fpgaif_tile : fpgaif_device.getTileList()) {
    auto name_id = fpgaif_tile.getName();
    auto tile_type = tile_types_[fpgaif_tile.getType()];
    tiles_[name_id] = device::Tile::create(fpgaif_device, fpgaif_tile,
                                           tile_type, name(name_id));
    if (name(tile_type).find("LAG_LAG") == 0)
      laguna_tiles.insert(name_id);

    LOG4CPLUS_TRACE_FMT(logger, "Found tile %s with type %s",
                        name(name_id).c_str(), name(tile_type).c_str());
  }

  LOG4CPLUS_INFO_FMT(
      logger,
      "Build %zu tiles with %zu types and %zu wires: %.2lfs, %.2lfGB RAM",
      tiles().size(), tile_types().size(), wires().size(),
      elapsed_seconds(last_time), getMemUsage() / 1024.0 / 1024.0 / 1024.0);

  last_time = steady_clock::now();

  int node_count = 0;
  nodes_.resize(fpgaif_nodes.size());

  unordered_map<uint32_t, omp_lock_t> tile_lock;
  for (auto [_, tile] : tiles_) {
    omp_init_lock(&tile_lock[tile->name()]);
  }
#pragma omp parallel num_threads(OMP_MAX_THREADS) // HACK
  {
#pragma omp for reduction(+ : node_count)
    for (size_t node_id = 0; node_id < fpgaif_nodes.size(); node_id++) {
      const auto &fpgaif_node = fpgaif_nodes[node_id];

      auto tile_wires = [&]() {
        vector<device::TileWire> ret;
        for (auto fpgaif_wire_id : fpgaif_node.getWires()) {
          const auto &fpgaif_wire = fpgaif_wires[fpgaif_wire_id];
          auto tile = tiles_.at(fpgaif_wire.getTile());
          auto wire = wires_.at(fpgaif_wire.getWire());
          ret.push_back(make_pair(tile, wire));
        }
        return ret;
      }();

      device::NodePtr node = [&]() {
        device::TileWire begin_wire = tile_wires.front();
        device::TileWire end_wire = make_pair(nullptr, nullptr);
        bool is_first_INT = true;
        for (auto tile_wire : tile_wires) {
          auto [tile, wire] = tile_wire;
          if (tile->type()->is_INT()) {
            if (is_first_INT) {
              is_first_INT = false;
              begin_wire = tile_wire;
            } else {
              end_wire = tile_wire;
              break;
            }
          }
        }
        if (end_wire.first == nullptr) {
          end_wire = begin_wire;
        }
        return device::Node::create(node_id, tile_wires, begin_wire, end_wire);
      }();

      for (auto [tile, wire] : tile_wires) {
        // critical section
        omp_set_lock(&tile_lock.at(tile->name()));
        tile->attach(wire, node);
        omp_unset_lock(&tile_lock.at(tile->name()));
      }

      nodes_[node_id] = node;
      node_count++;
    }
  } // parallel
  for (auto [_, lock] : tile_lock) {
    omp_destroy_lock(&lock);
  }
  LOG4CPLUS_INFO_FMT(logger, "Build %d nodes: %.2lfs, %.2lfGB RAM", node_count,
                     elapsed_seconds(last_time),
                     getMemUsage() / 1024.0 / 1024.0 / 1024.0);

  for (auto tile_type : tile_types_) {
    const std::string &tile_type_name = name(tile_type);
    if (!(tile_type->is_INT() || tile_type_name.find("LAG") == 0)) {
      tile_type->set_route_excluded(true);
      // LOG4CPLUS_INFO_FMT(logger, "Excluded tile type %s",
      // tile_type_name.c_str());
    }
  }

  for (auto [tile_id, tile] : tiles_) {
    // Find an arbitrary INT tile
    if (tile->type()->is_INT()) {

      for (auto [wire, node] : tile->wire_to_node()) {
        if (node == nullptr)
          continue;
        auto base_tile = node->base_tile();
        auto base_wire = node->base_wire();
        std::string base_wire_name = name(base_wire);
        if (base_wire_name.find("INODE_") == 0) {
          // assert(node->intentcode() == NODE_LOCAL);
          if (base_tile->name() == tile_id) {
            continue; // ???
          }
        } else if (base_wire_name.find("BOUNCE_") == 0) {
          // assert(node->intentcode() == NODE_PINBOUNCE);
        } else if (base_wire_name.find("BYPASS_") == 0) {
          // assert(node->intentcode() == NODE_PINBOUNCE);
        } else if (base_wire_name.find("INT_NODE_GLOBAL_") == 0 ||
                   base_wire_name.find("INT_NODE_IMUX_") == 0) {
          // assert(node->intentcode() == NODE_LOCAL);
        } else {
          continue;
        }
        accessible_wires_same_col.insert(base_wire->name());
      }

      break;
    }
  }

  // Examine all wires in Laguna tile. Record those uphill of a Super Long Line
  // that originates in an INT tile (and thus must be a NODE_PINFEED).
  for (auto &tile_id : laguna_tiles) {
    auto tile = tiles_.at(tile_id);
    for (auto wire : tile->type()->wires()) {
      if (!(name(wire).find("UBUMP") == 0))
        continue;
      NodePtr sll_node = tile->wire_to_node().at(wire);
      for (NodePtr uphill1 : sll_node->get_uphill_nodes()) {
        for (NodePtr uphill2 : uphill1->get_uphill_nodes()) {
          auto uphill2_tile = uphill2->base_tile();
          if (!uphill2_tile->type()->is_INT())
            continue;
          // assert(uphill2->intentcode() == NODE_PINFEED);
          if (laguna_tiles_to_specwires.find(uphill2->base_tile()->name()) ==
              laguna_tiles_to_specwires.end()) {
            laguna_tiles_to_specwires[uphill2->base_tile()->name()] =
                std::set<uint32_t>();
          }
          laguna_tiles_to_specwires[uphill2->base_tile()->name()].insert(
              uphill2->base_wire()->name());
        }
      }
    }
  }
}

const TilePtr Node::end_tile() const {
  TilePtr end_tile = nullptr;
  auto base_tile_type = base_tile()->type();
  for (auto [tile, wire] : wires_) {
    auto tile_type = tile->type();
    if (tile_type->is_INT() || tile_type->name() == base_tile_type->name()) {
      bool end_tile_not_null = (end_tile != nullptr);
      end_tile = tile;
      // Break if this is the second non-null tile
      if (end_tile_not_null)
        break;
    }
  }
  if (end_tile == nullptr) {
    end_tile = base_tile();
  }
  return end_tile;
}

void Node::sort_and_uniquify(std::vector<NodePtr> &nodes) {
  struct WireLess {
    bool operator()(const NodePtr &lhs, const NodePtr &rhs) const {
      return lhs->base_wire()->name() < rhs->base_wire()->name();
    }
  };
  struct IdEqual {
    bool operator()(const NodePtr &lhs, const NodePtr &rhs) const {
      return lhs->id() == rhs->id();
    }
  };
  sort(nodes.begin(), nodes.end(), WireLess());
  auto it = unique(nodes.begin(), nodes.end(), IdEqual());
  nodes.erase(it, nodes.end());
}

vector<NodePtr> Node::get_downhill_nodes() const {
  vector<NodePtr> downhill_nodes;
  for (auto [tile, wire] : wires()) {
    auto tile_type = tile->type();
    auto downhill_wires = tile_type->get_downhill_wires(wire);
    for (auto downhill_wire : downhill_wires) {
      auto downhill_node = tile->wire_to_node().at(downhill_wire);
      downhill_nodes.push_back(downhill_node);
    }
  }
  // auto size_before_dedup = downhill_nodes.size();
  Node::sort_and_uniquify(downhill_nodes);
  // auto size_after_dedup = downhill_nodes.size();
  // if (size_after_dedup < size_before_dedup) {
  //   LOG4CPLUS_DEBUG_FMT(logger, "Dedup node %d's downhill nodes: %zu -> %zu",
  //                       id(), size_before_dedup, size_after_dedup);
  //   for (auto [tile, wire] : wires()) {
  //     auto tile_type = tile->type();
  //     auto downhill_wires = tile_type->get_downhill_wires(wire);
  //     LOG4CPLUS_DEBUG_FMT(logger, "  tile %d type %d wire %d", tile->name(),
  //                         tile_type->name(), wire->name());
  //     for (auto downhill_wire : downhill_wires) {
  //       auto downhill_node = tile->wire_to_node().at(downhill_wire);
  //       LOG4CPLUS_DEBUG_FMT(logger, "    wire %d node %d",
  //       downhill_wire->name(),
  //                           downhill_node->id());
  //     }
  //   }
  // }
  return downhill_nodes;
}

vector<NodePtr> Node::get_uphill_nodes() const {
  vector<NodePtr> uphill_nodes;
  for (auto [tile, wire] : wires()) {
    auto tile_type = tile->type();
    auto uphill_wires = tile_type->get_uphill_wires(wire);
    for (auto uphill_wire : uphill_wires) {
      if (tile->wire_to_node().contains(uphill_wire)) {
        auto uphill_node = tile->wire_to_node().at(uphill_wire);
        uphill_nodes.push_back(uphill_node);
      }
      // auto uphill_node = tile->wire_to_node().at(uphill_wire);
      // uphill_nodes.push_back(uphill_node);
    }
  }
  Node::sort_and_uniquify(uphill_nodes);
  return uphill_nodes;
}

IntentCode Wire::intentcode(const string &wire_name,
                            const string &wire_type_name,
                            const string &tile_type_name) {
  static const unordered_map<string, IntentCode> wire_name_to_ic = {
      // {"[EW]{2}1_[EW]_BEG[0-7]", IntentCode::NODE_HSINGLE},
      {"EE1_E_BEG0", IntentCode::NODE_HSINGLE},
      {"EE1_E_BEG1", IntentCode::NODE_HSINGLE},
      {"EE1_E_BEG2", IntentCode::NODE_HSINGLE},
      {"EE1_E_BEG3", IntentCode::NODE_HSINGLE},
      {"EE1_E_BEG4", IntentCode::NODE_HSINGLE},
      {"EE1_E_BEG5", IntentCode::NODE_HSINGLE},
      {"EE1_E_BEG6", IntentCode::NODE_HSINGLE},
      {"EE1_E_BEG7", IntentCode::NODE_HSINGLE},
      {"EE1_W_BEG0", IntentCode::NODE_HSINGLE},
      {"EE1_W_BEG1", IntentCode::NODE_HSINGLE},
      {"EE1_W_BEG2", IntentCode::NODE_HSINGLE},
      {"EE1_W_BEG3", IntentCode::NODE_HSINGLE},
      {"EE1_W_BEG4", IntentCode::NODE_HSINGLE},
      {"EE1_W_BEG5", IntentCode::NODE_HSINGLE},
      {"EE1_W_BEG6", IntentCode::NODE_HSINGLE},
      {"EE1_W_BEG7", IntentCode::NODE_HSINGLE},
      {"WW1_E_BEG0", IntentCode::NODE_HSINGLE},
      {"WW1_E_BEG1", IntentCode::NODE_HSINGLE},
      {"WW1_E_BEG2", IntentCode::NODE_HSINGLE},
      {"WW1_E_BEG3", IntentCode::NODE_HSINGLE},
      {"WW1_E_BEG4", IntentCode::NODE_HSINGLE},
      {"WW1_E_BEG5", IntentCode::NODE_HSINGLE},
      {"WW1_E_BEG6", IntentCode::NODE_HSINGLE},
      {"WW1_E_BEG7", IntentCode::NODE_HSINGLE},
      {"WW1_W_BEG0", IntentCode::NODE_HSINGLE},
      {"WW1_W_BEG1", IntentCode::NODE_HSINGLE},
      {"WW1_W_BEG2", IntentCode::NODE_HSINGLE},
      {"WW1_W_BEG3", IntentCode::NODE_HSINGLE},
      {"WW1_W_BEG4", IntentCode::NODE_HSINGLE},
      {"WW1_W_BEG5", IntentCode::NODE_HSINGLE},
      {"WW1_W_BEG6", IntentCode::NODE_HSINGLE},
      {"WW1_W_BEG7", IntentCode::NODE_HSINGLE},
      // {"WW1_E_7_FT0", IntentCode::NODE_HSINGLE},
      {"WW1_E_7_FT0", IntentCode::NODE_HSINGLE},
      // {"[NS]{2}1_[EW]_BEG[0-7]", IntentCode::NODE_VSINGLE},
      {"NN1_E_BEG0", IntentCode::NODE_VSINGLE},
      {"NN1_E_BEG1", IntentCode::NODE_VSINGLE},
      {"NN1_E_BEG2", IntentCode::NODE_VSINGLE},
      {"NN1_E_BEG3", IntentCode::NODE_VSINGLE},
      {"NN1_E_BEG4", IntentCode::NODE_VSINGLE},
      {"NN1_E_BEG5", IntentCode::NODE_VSINGLE},
      {"NN1_E_BEG6", IntentCode::NODE_VSINGLE},
      {"NN1_E_BEG7", IntentCode::NODE_VSINGLE},
      {"NN1_W_BEG0", IntentCode::NODE_VSINGLE},
      {"NN1_W_BEG1", IntentCode::NODE_VSINGLE},
      {"NN1_W_BEG2", IntentCode::NODE_VSINGLE},
      {"NN1_W_BEG3", IntentCode::NODE_VSINGLE},
      {"NN1_W_BEG4", IntentCode::NODE_VSINGLE},
      {"NN1_W_BEG5", IntentCode::NODE_VSINGLE},
      {"NN1_W_BEG6", IntentCode::NODE_VSINGLE},
      {"NN1_W_BEG7", IntentCode::NODE_VSINGLE},
      {"SS1_E_BEG0", IntentCode::NODE_VSINGLE},
      {"SS1_E_BEG1", IntentCode::NODE_VSINGLE},
      {"SS1_E_BEG2", IntentCode::NODE_VSINGLE},
      {"SS1_E_BEG3", IntentCode::NODE_VSINGLE},
      {"SS1_E_BEG4", IntentCode::NODE_VSINGLE},
      {"SS1_E_BEG5", IntentCode::NODE_VSINGLE},
      {"SS1_E_BEG6", IntentCode::NODE_VSINGLE},
      {"SS1_E_BEG7", IntentCode::NODE_VSINGLE},
      {"SS1_W_BEG0", IntentCode::NODE_VSINGLE},
      {"SS1_W_BEG1", IntentCode::NODE_VSINGLE},
      {"SS1_W_BEG2", IntentCode::NODE_VSINGLE},
      {"SS1_W_BEG3", IntentCode::NODE_VSINGLE},
      {"SS1_W_BEG4", IntentCode::NODE_VSINGLE},
      {"SS1_W_BEG5", IntentCode::NODE_VSINGLE},
      {"SS1_W_BEG6", IntentCode::NODE_VSINGLE},
      {"SS1_W_BEG7", IntentCode::NODE_VSINGLE},
      // {"[EW]{2}2_[EW]_BEG[0-7]", IntentCode::NODE_HDOUBLE},
      {"EE2_E_BEG0", IntentCode::NODE_HDOUBLE},
      {"EE2_E_BEG1", IntentCode::NODE_HDOUBLE},
      {"EE2_E_BEG2", IntentCode::NODE_HDOUBLE},
      {"EE2_E_BEG3", IntentCode::NODE_HDOUBLE},
      {"EE2_E_BEG4", IntentCode::NODE_HDOUBLE},
      {"EE2_E_BEG5", IntentCode::NODE_HDOUBLE},
      {"EE2_E_BEG6", IntentCode::NODE_HDOUBLE},
      {"EE2_E_BEG7", IntentCode::NODE_HDOUBLE},
      {"EE2_W_BEG0", IntentCode::NODE_HDOUBLE},
      {"EE2_W_BEG1", IntentCode::NODE_HDOUBLE},
      {"EE2_W_BEG2", IntentCode::NODE_HDOUBLE},
      {"EE2_W_BEG3", IntentCode::NODE_HDOUBLE},
      {"EE2_W_BEG4", IntentCode::NODE_HDOUBLE},
      {"EE2_W_BEG5", IntentCode::NODE_HDOUBLE},
      {"EE2_W_BEG6", IntentCode::NODE_HDOUBLE},
      {"EE2_W_BEG7", IntentCode::NODE_HDOUBLE},
      {"WW2_E_BEG0", IntentCode::NODE_HDOUBLE},
      {"WW2_E_BEG1", IntentCode::NODE_HDOUBLE},
      {"WW2_E_BEG2", IntentCode::NODE_HDOUBLE},
      {"WW2_E_BEG3", IntentCode::NODE_HDOUBLE},
      {"WW2_E_BEG4", IntentCode::NODE_HDOUBLE},
      {"WW2_E_BEG5", IntentCode::NODE_HDOUBLE},
      {"WW2_E_BEG6", IntentCode::NODE_HDOUBLE},
      {"WW2_E_BEG7", IntentCode::NODE_HDOUBLE},
      {"WW2_W_BEG0", IntentCode::NODE_HDOUBLE},
      {"WW2_W_BEG1", IntentCode::NODE_HDOUBLE},
      {"WW2_W_BEG2", IntentCode::NODE_HDOUBLE},
      {"WW2_W_BEG3", IntentCode::NODE_HDOUBLE},
      {"WW2_W_BEG4", IntentCode::NODE_HDOUBLE},
      {"WW2_W_BEG5", IntentCode::NODE_HDOUBLE},
      {"WW2_W_BEG6", IntentCode::NODE_HDOUBLE},
      {"WW2_W_BEG7", IntentCode::NODE_HDOUBLE},
      // {"[NS]{2}2_[EW]_BEG[0-7]", IntentCode::NODE_VDOUBLE},
      {"NN2_E_BEG0", IntentCode::NODE_VDOUBLE},
      {"NN2_E_BEG1", IntentCode::NODE_VDOUBLE},
      {"NN2_E_BEG2", IntentCode::NODE_VDOUBLE},
      {"NN2_E_BEG3", IntentCode::NODE_VDOUBLE},
      {"NN2_E_BEG4", IntentCode::NODE_VDOUBLE},
      {"NN2_E_BEG5", IntentCode::NODE_VDOUBLE},
      {"NN2_E_BEG6", IntentCode::NODE_VDOUBLE},
      {"NN2_E_BEG7", IntentCode::NODE_VDOUBLE},
      {"NN2_W_BEG0", IntentCode::NODE_VDOUBLE},
      {"NN2_W_BEG1", IntentCode::NODE_VDOUBLE},
      {"NN2_W_BEG2", IntentCode::NODE_VDOUBLE},
      {"NN2_W_BEG3", IntentCode::NODE_VDOUBLE},
      {"NN2_W_BEG4", IntentCode::NODE_VDOUBLE},
      {"NN2_W_BEG5", IntentCode::NODE_VDOUBLE},
      {"NN2_W_BEG6", IntentCode::NODE_VDOUBLE},
      {"NN2_W_BEG7", IntentCode::NODE_VDOUBLE},
      {"SS2_E_BEG0", IntentCode::NODE_VDOUBLE},
      {"SS2_E_BEG1", IntentCode::NODE_VDOUBLE},
      {"SS2_E_BEG2", IntentCode::NODE_VDOUBLE},
      {"SS2_E_BEG3", IntentCode::NODE_VDOUBLE},
      {"SS2_E_BEG4", IntentCode::NODE_VDOUBLE},
      {"SS2_E_BEG5", IntentCode::NODE_VDOUBLE},
      {"SS2_E_BEG6", IntentCode::NODE_VDOUBLE},
      {"SS2_E_BEG7", IntentCode::NODE_VDOUBLE},
      {"SS2_W_BEG0", IntentCode::NODE_VDOUBLE},
      {"SS2_W_BEG1", IntentCode::NODE_VDOUBLE},
      {"SS2_W_BEG2", IntentCode::NODE_VDOUBLE},
      {"SS2_W_BEG3", IntentCode::NODE_VDOUBLE},
      {"SS2_W_BEG4", IntentCode::NODE_VDOUBLE},
      {"SS2_W_BEG5", IntentCode::NODE_VDOUBLE},
      {"SS2_W_BEG6", IntentCode::NODE_VDOUBLE},
      {"SS2_W_BEG7", IntentCode::NODE_VDOUBLE},
      // {"[EW]{2}4_[EW]_BEG[0-7]", IntentCode::NODE_HQUAD},
      {"EE4_E_BEG0", IntentCode::NODE_HQUAD},
      {"EE4_E_BEG1", IntentCode::NODE_HQUAD},
      {"EE4_E_BEG2", IntentCode::NODE_HQUAD},
      {"EE4_E_BEG3", IntentCode::NODE_HQUAD},
      {"EE4_E_BEG4", IntentCode::NODE_HQUAD},
      {"EE4_E_BEG5", IntentCode::NODE_HQUAD},
      {"EE4_E_BEG6", IntentCode::NODE_HQUAD},
      {"EE4_E_BEG7", IntentCode::NODE_HQUAD},
      {"EE4_W_BEG0", IntentCode::NODE_HQUAD},
      {"EE4_W_BEG1", IntentCode::NODE_HQUAD},
      {"EE4_W_BEG2", IntentCode::NODE_HQUAD},
      {"EE4_W_BEG3", IntentCode::NODE_HQUAD},
      {"EE4_W_BEG4", IntentCode::NODE_HQUAD},
      {"EE4_W_BEG5", IntentCode::NODE_HQUAD},
      {"EE4_W_BEG6", IntentCode::NODE_HQUAD},
      {"EE4_W_BEG7", IntentCode::NODE_HQUAD},
      {"WW4_E_BEG0", IntentCode::NODE_HQUAD},
      {"WW4_E_BEG1", IntentCode::NODE_HQUAD},
      {"WW4_E_BEG2", IntentCode::NODE_HQUAD},
      {"WW4_E_BEG3", IntentCode::NODE_HQUAD},
      {"WW4_E_BEG4", IntentCode::NODE_HQUAD},
      {"WW4_E_BEG5", IntentCode::NODE_HQUAD},
      {"WW4_E_BEG6", IntentCode::NODE_HQUAD},
      {"WW4_E_BEG7", IntentCode::NODE_HQUAD},
      {"WW4_W_BEG0", IntentCode::NODE_HQUAD},
      {"WW4_W_BEG1", IntentCode::NODE_HQUAD},
      {"WW4_W_BEG2", IntentCode::NODE_HQUAD},
      {"WW4_W_BEG3", IntentCode::NODE_HQUAD},
      {"WW4_W_BEG4", IntentCode::NODE_HQUAD},
      {"WW4_W_BEG5", IntentCode::NODE_HQUAD},
      {"WW4_W_BEG6", IntentCode::NODE_HQUAD},
      {"WW4_W_BEG7", IntentCode::NODE_HQUAD},
      // {"[NS]{2}4_[EW]_BEG[0-7]", IntentCode::NODE_VQUAD},
      {"NN4_E_BEG0", IntentCode::NODE_VQUAD},
      {"NN4_E_BEG1", IntentCode::NODE_VQUAD},
      {"NN4_E_BEG2", IntentCode::NODE_VQUAD},
      {"NN4_E_BEG3", IntentCode::NODE_VQUAD},
      {"NN4_E_BEG4", IntentCode::NODE_VQUAD},
      {"NN4_E_BEG5", IntentCode::NODE_VQUAD},
      {"NN4_E_BEG6", IntentCode::NODE_VQUAD},
      {"NN4_E_BEG7", IntentCode::NODE_VQUAD},
      {"NN4_W_BEG0", IntentCode::NODE_VQUAD},
      {"NN4_W_BEG1", IntentCode::NODE_VQUAD},
      {"NN4_W_BEG2", IntentCode::NODE_VQUAD},
      {"NN4_W_BEG3", IntentCode::NODE_VQUAD},
      {"NN4_W_BEG4", IntentCode::NODE_VQUAD},
      {"NN4_W_BEG5", IntentCode::NODE_VQUAD},
      {"NN4_W_BEG6", IntentCode::NODE_VQUAD},
      {"NN4_W_BEG7", IntentCode::NODE_VQUAD},
      {"SS4_E_BEG0", IntentCode::NODE_VQUAD},
      {"SS4_E_BEG1", IntentCode::NODE_VQUAD},
      {"SS4_E_BEG2", IntentCode::NODE_VQUAD},
      {"SS4_E_BEG3", IntentCode::NODE_VQUAD},
      {"SS4_E_BEG4", IntentCode::NODE_VQUAD},
      {"SS4_E_BEG5", IntentCode::NODE_VQUAD},
      {"SS4_E_BEG6", IntentCode::NODE_VQUAD},
      {"SS4_E_BEG7", IntentCode::NODE_VQUAD},
      {"SS4_W_BEG0", IntentCode::NODE_VQUAD},
      {"SS4_W_BEG1", IntentCode::NODE_VQUAD},
      {"SS4_W_BEG2", IntentCode::NODE_VQUAD},
      {"SS4_W_BEG3", IntentCode::NODE_VQUAD},
      {"SS4_W_BEG4", IntentCode::NODE_VQUAD},
      {"SS4_W_BEG5", IntentCode::NODE_VQUAD},
      {"SS4_W_BEG6", IntentCode::NODE_VQUAD},
      {"SS4_W_BEG7", IntentCode::NODE_VQUAD},
      // {"[EW]{2}12_BEG[0-7]", IntentCode::NODE_HLONG},
      {"EE12_BEG0", IntentCode::NODE_HLONG},
      {"EE12_BEG1", IntentCode::NODE_HLONG},
      {"EE12_BEG2", IntentCode::NODE_HLONG},
      {"EE12_BEG3", IntentCode::NODE_HLONG},
      {"EE12_BEG4", IntentCode::NODE_HLONG},
      {"EE12_BEG5", IntentCode::NODE_HLONG},
      {"EE12_BEG6", IntentCode::NODE_HLONG},
      {"EE12_BEG7", IntentCode::NODE_HLONG},
      {"WW12_BEG0", IntentCode::NODE_HLONG},
      {"WW12_BEG1", IntentCode::NODE_HLONG},
      {"WW12_BEG2", IntentCode::NODE_HLONG},
      {"WW12_BEG3", IntentCode::NODE_HLONG},
      {"WW12_BEG4", IntentCode::NODE_HLONG},
      {"WW12_BEG5", IntentCode::NODE_HLONG},
      {"WW12_BEG6", IntentCode::NODE_HLONG},
      {"WW12_BEG7", IntentCode::NODE_HLONG},
      // {"[NS]{2}12_BEG[0-7]", IntentCode::NODE_VLONG},
      {"NN12_BEG0", IntentCode::NODE_VLONG},
      {"NN12_BEG1", IntentCode::NODE_VLONG},
      {"NN12_BEG2", IntentCode::NODE_VLONG},
      {"NN12_BEG3", IntentCode::NODE_VLONG},
      {"NN12_BEG4", IntentCode::NODE_VLONG},
      {"NN12_BEG5", IntentCode::NODE_VLONG},
      {"NN12_BEG6", IntentCode::NODE_VLONG},
      {"NN12_BEG7", IntentCode::NODE_VLONG},
      {"SS12_BEG0", IntentCode::NODE_VLONG},
      {"SS12_BEG1", IntentCode::NODE_VLONG},
      {"SS12_BEG2", IntentCode::NODE_VLONG},
      {"SS12_BEG3", IntentCode::NODE_VLONG},
      {"SS12_BEG4", IntentCode::NODE_VLONG},
      {"SS12_BEG5", IntentCode::NODE_VLONG},
      {"SS12_BEG6", IntentCode::NODE_VLONG},
      {"SS12_BEG7", IntentCode::NODE_VLONG},
  };

  static const unordered_map<string, IntentCode> wire_type_name_to_ic = {
      {"INTENT_DEFAULT", IntentCode::INTENT_DEFAULT},
      {"NODE_CLE_OUTPUT", IntentCode::NODE_CLE_OUTPUT},
      {"NODE_DEDICATED", IntentCode::NODE_DEDICATED},
      {"NODE_DOUBLE", IntentCode::NODE_DOUBLE},
      {"NODE_GLOBAL_BUFG", IntentCode::NODE_GLOBAL_BUFG},
      {"NODE_GLOBAL_HDISTR", IntentCode::NODE_GLOBAL_HDISTR},
      {"NODE_GLOBAL_HROUTE", IntentCode::NODE_GLOBAL_HROUTE},
      {"NODE_GLOBAL_LEAF", IntentCode::NODE_GLOBAL_LEAF},
      {"NODE_GLOBAL_VDISTR", IntentCode::NODE_GLOBAL_VDISTR},
      {"NODE_GLOBAL_VROUTE", IntentCode::NODE_GLOBAL_VROUTE},
      {"NODE_HLONG", IntentCode::NODE_HLONG},
      {"NODE_HQUAD", IntentCode::NODE_HQUAD},
      {"NODE_INT_INTERFACE", IntentCode::NODE_INT_INTERFACE},
      {"NODE_LAGUNA_DATA", IntentCode::NODE_LAGUNA_DATA},
      {"NODE_LAGUNA_OUTPUT", IntentCode::NODE_LAGUNA_OUTPUT},
      {"NODE_LOCAL", IntentCode::NODE_LOCAL},
      {"NODE_OPTDELAY", IntentCode::NODE_OPTDELAY},
      {"NODE_OUTPUT", IntentCode::NODE_OUTPUT},
      {"NODE_PINBOUNCE", IntentCode::NODE_PINBOUNCE},
      {"NODE_PINFEED", IntentCode::NODE_PINFEED},
      {"NODE_SINGLE", IntentCode::NODE_SINGLE},
      {"NODE_VLONG", IntentCode::NODE_VLONG},
      {"NODE_VQUAD", IntentCode::NODE_VQUAD},
  };

  // static const std::pair<std::string, Wire::Type> wire_name_patterns[9] = {
  //     {"[EW]{2}1_[EW]_BEG[0-7]", NODE_HSINGLE},
  //     {"WW1_E_7_FT0", NODE_HSINGLE},
  //     {"[NS]{2}1_[EW]_BEG[0-7]", NODE_VSINGLE},
  //     {"[EW]{2}2_[EW]_BEG[0-7]", NODE_HDOUBLE},
  //     {"[NS]{2}2_[EW]_BEG[0-7]", NODE_VDOUBLE},
  //     {"[EW]{2}4_[EW]_BEG[0-7]", NODE_HQUAD},
  //     {"[NS]{2}4_[EW]_BEG[0-7]", NODE_VQUAD},
  //     {"[EW]{2}12_BEG[0-7]", NODE_HLONG},
  //     {"[NS]{2}12_BEG[0-7]", NODE_VLONG},
  // };
  // for (const auto &pattern : wire_name_patterns) {
  //   if (std::regex_match(wire_name, std::regex(pattern.first))) {
  //     return pattern.second;
  //   }
  // }

  if ((tile_type_name == "INT") &&
      (wire_name.find("IMUX_") == 0 || wire_name.find("CTRL_") == 0)) {
    // assert(wire_type_name == "NODE_PINFEED");
    if (wire_type_name != "NODE_PINFEED") {
      LOG4CPLUS_TRACE_FMT(logger, "Wire %s type %s tile type %s",
                          wire_name.c_str(), wire_type_name.c_str(),
                          tile_type_name.c_str());
    }
    return IntentCode::NODE_PINFEED;
  }
  if ((tile_type_name == "RCLK_INT_L" || tile_type_name == "RCLK_INT_R") &&
      (wire_name.find("INT_NODE_IMUX_") == 0 ||
       wire_name.find("INT_RCLK_TO_CLK_") == 0)) {
    // Note: the following wire name patterns are also with NODE_PINFEED
    //   "^CLK_LEAF_SITES_[0-9]\+_CE_INT"
    //   "^CLK_LEAF_SITES_[0-9]\+_CLK_CASC_IN"
    //   "^CLK_LEAF_SITES_[0-9]\+_ENSEL_PROG"

    // assert(wire_type_name == "NODE_PINFEED");
    if (wire_type_name != "NODE_PINFEED") {
      LOG4CPLUS_TRACE_FMT(logger, "Wire %s type %s tile type %s",
                          wire_name.c_str(), wire_type_name.c_str(),
                          tile_type_name.c_str());
    }
    return IntentCode::NODE_PINFEED;
  }

  if (wire_name_to_ic.contains(wire_name)) {
    return wire_name_to_ic.at(wire_name);
  }
  assert(wire_type_name_to_ic.contains(wire_type_name));
  return wire_type_name_to_ic.at(wire_type_name);
}

} // namespace device
