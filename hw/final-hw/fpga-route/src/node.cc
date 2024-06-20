#include "node.h"
#include "connection.h"
#include "routing-graph.h"
#include <regex>
#include <set>

#include <log4cplus/logger.h>
#include <log4cplus/loggingmacros.h>

static log4cplus::Logger logger = log4cplus::Logger::getInstance("ace.graph");

namespace rwroute {
IntentCode intentcode(const std::string &wire_name) {
  IntentCode type = IntentCode::INTENT_DEFAULT;
  if (wire_name.find("EE4_") == 0 || wire_name.find("WW4_") == 0) {
    type = IntentCode::NODE_HQUAD;
  } else if (wire_name.find("NN4_") == 0 || wire_name.find("SS4_") == 0) {
    type = IntentCode::NODE_VQUAD;
  } else if (wire_name.find("EE12_") == 0 || wire_name.find("WW12_") == 0) {
    type = IntentCode::NODE_HLONG;
  } else if (wire_name.find("NN12_") == 0 || wire_name.find("SS12_") == 0) {
    type = IntentCode::NODE_VLONG;
  }
  return type;
}

NodeType node_type(const device::DevicePtr &dev,
                   const device::NodePtr &dev_node) {
  NodeType type = NodeType::UNKNOWN;

  auto ends_with = [](const std::string &full_string,
                      const std::string &ending) {
    if (full_string.length() >= ending.length()) {
      return (0 == full_string.compare(full_string.length() - ending.length(),
                                       ending.length(), ending));
    } else {
      return false;
    }
  };

  if (dev_node->intentcode() == IntentCode::NODE_PINBOUNCE) {
    type = NodeType::PINBOUNCE;
  } else if (dev_node->intentcode() == IntentCode::NODE_PINFEED) {
    if (dev->laguna_tiles_to_specwires.contains(
            dev_node->base_tile()->name()) &&
        dev->laguna_tiles_to_specwires[dev_node->base_tile()->name()].contains(
            dev_node->base_wire()->name())) {
      type = NodeType::LAGUNA_I;
    } else {
      type = NodeType::PINFEED_I;
    }
  } else if (dev_node->intentcode() == IntentCode::NODE_LAGUNA_OUTPUT) {
    assert(dev->name(dev_node->base_tile()->type()).find("LAG_LAG") == 0);
    if (ends_with(dev->name(dev_node->base_wire()), "_TXOUT")) {
      type = NodeType::LAGUNA_I;
    }
  } else if (dev_node->intentcode() == IntentCode::NODE_LAGUNA_DATA) {
    assert(dev->name(dev_node->base_tile()->type()).find("LAG_LAG") == 0);
    if (dev_node->base_tile()->name() != dev_node->end_tile()->name()) {
      type = NodeType::SUPER_LONG_LINE;
    }
  } else if (dev_node->intentcode() == IntentCode::INTENT_DEFAULT) {
    if (dev->name(dev_node->base_tile()->type()).find("LAGUNA_TILE") == 0) {
      auto &wire_name = dev->name(dev_node->base_wire());
      if (!(wire_name.find("UBUMP") == 0)) {
        assert(dev_node->base_tile()->name() != dev_node->end_tile()->name());
        type = NodeType::SUPER_LONG_LINE;
      } else if (ends_with(wire_name, "_TXOUT")) {
        type = NodeType::LAGUNA_I;
      }
    }
  }
  if (type == NodeType::UNKNOWN) {
    type = NodeType::WIRE; // Default
  }
  return type;
}

double compute_base_cost(IntentCode ic, int length) {
  using namespace device;
  double base_cost;
  switch (ic) {
  case IntentCode::NODE_HQUAD:
    base_cost = 0.35 * length;
    break;
  case IntentCode::NODE_VQUAD:
  case IntentCode::NODE_HLONG:
    base_cost = 0.15 * length;
    break;
  case IntentCode::NODE_VLONG:
    base_cost = 0.7;
    break;
  default:
    base_cost = 0.4; // Default case, if none of the patterns match
  }
  return base_cost;
}
} // namespace rwroute

namespace contest {
int compute_wl_score(const std::string &wire_name) {
  static const std::pair<std::string, int> patterns[9] = {
      {"[EW]{2}1_[EW]_BEG[0-7]", 1}, {"WW1_E_7_FT0", 1},
      {"[NS]{2}1_[EW]_BEG[0-7]", 1}, {"[EW]{2}2_[EW]_BEG[0-7]", 5},
      {"[NS]{2}2_[EW]_BEG[0-7]", 3}, {"[EW]{2}4_[EW]_BEG[0-7]", 10},
      {"[NS]{2}4_[EW]_BEG[0-7]", 5}, {"[EW]{2}12_BEG[0-7]", 14},
      {"[NS]{2}12_BEG[0-7]", 12},
  };

  int wl_score = 0; // If no match is found, return 0.
  for (const auto &pattern : patterns) {
    if (std::regex_match(wire_name, std::regex(pattern.first))) {
      wl_score = pattern.second;
      break;
    }
  }
  return wl_score;
}
int compute_wl_score(const IntentCode &intentcode) {
  static const std::unordered_map<IntentCode, int> wl_score = {
      {IntentCode::NODE_HSINGLE, 1}, {IntentCode::NODE_VSINGLE, 1},
      {IntentCode::NODE_HDOUBLE, 5}, {IntentCode::NODE_VDOUBLE, 3},
      {IntentCode::NODE_HQUAD, 10},  {IntentCode::NODE_VQUAD, 5},
      {IntentCode::NODE_HLONG, 14},  {IntentCode::NODE_VLONG, 12},
  };
  if (!wl_score.contains(intentcode))
    return 0;
  return wl_score.at(intentcode);
}
} // namespace contest

bool Node::operator==(const NodePtr &that) const {
  return this->id() == that->id();
}

void Node::set_base_cost() {
  base_cost_ = 0.4f;
  switch (type()) {
  case NodeType::LAGUNA_I:
    // Make all approaches to SLLs zero-cost to encourage exploration
    // Assigning a base cost of zero would normally break congestion resolution
    // (since RWroute.getNodeCost() would return zero) but doing it here should
    // be okay because this node only leads to a SLL which will have a non-zero
    // base cost
    base_cost_ = 0.0f;
    break;
  case NodeType::SUPER_LONG_LINE:
    // assert(length() == RouteNodeGraph.SUPER_LONG_LINE_LENGTH_IN_TILES);
    base_cost_ = 0.3f * length();
    break;
  case NodeType::WIRE: {
    // NOTE: IntentCode is device-dependent
    auto ic = dev_node()->intentcode();
    switch (ic) {
    case IntentCode::NODE_OUTPUT: // LUT route-thru
    case IntentCode::NODE_CLE_OUTPUT:
    case IntentCode::NODE_LAGUNA_OUTPUT:
    case IntentCode::NODE_LAGUNA_DATA: // US+: U-turn SLL at the boundary of the
                                       // device
      assert(length() == 0);
      break;
    case IntentCode::NODE_LOCAL:
    case IntentCode::INTENT_DEFAULT:
      assert(length() <= 1);
      break;
    case IntentCode::NODE_SINGLE:
    case IntentCode::NODE_HSINGLE:
    case IntentCode::NODE_VSINGLE:
      assert(length() <= 2);
      if (length() == 2)
        base_cost_ *= length();
      break;
    case IntentCode::NODE_DOUBLE:
    case IntentCode::NODE_HDOUBLE:
    case IntentCode::NODE_VDOUBLE:
      if (end_x() != dev_node()->base_tile()->x()) {
        assert(length() <= 2);
        // Typically, length() = 1 (since tile X is not equal)
        // In US, have seen length() = 2, e.g. VU440's INT_X171Y827/EE2_E_BEG7.
        if (length() == 2)
          base_cost_ *= length();
      } else {
        // Typically, length() = 2 except for horizontal U-turns (length() = 0)
        // or vertical U-turns (length() = 1).
        // In US, have seen length() = 3, e.g. VU440's INT_X171Y827/NN2_E_BEG7.
        assert(length() <= 3);
      }
      break;
    case IntentCode::NODE_HQUAD:
      if (!(length() != 0 || downhill_dev_nodes.empty())) {
        LOG4CPLUS_ERROR_FMT(logger, "Unexpected route node %d", id());
        for (auto [tile, wire] : dev_node()->wires()) {
          LOG4CPLUS_ERROR_FMT(logger, "  tile %s wire %s",
                              routing_graph->name(tile).c_str(),
                              routing_graph->name(wire).c_str());
        }
        LOG4CPLUS_ERROR_FMT(logger,
                            "  length() = %d, downhill_dev_nodes.size() = %zu",
                            length(), downhill_dev_nodes.size());
      }
      // assert(length() != 0 || downhill_dev_nodes.empty());
      base_cost_ = 0.35f * length();
      break;
    case IntentCode::NODE_VQUAD:
      // In case of U-turn nodes
      if (length() != 0)
        base_cost_ = 0.15f * length(); // VQUADs have length() 4 and 5
      break;
    case IntentCode::NODE_HLONG:
      assert(length() != 0 || downhill_dev_nodes.empty());
      base_cost_ = 0.15f * length(); // HLONGs have length() 6 and 7
      break;
    case IntentCode::NODE_VLONG:
      base_cost_ = 0.7f;
      break;
    default:
        // base_cost_ = 0.4f;
        ;
    }
    break;
  }
  case NodeType::PINFEED_I:
  case NodeType::PINBOUNCE:
    break;
  case NodeType::PINFEED_O:
    base_cost_ = 1.f;
    break;
  default:
    LOG4CPLUS_FATAL(logger, "Node::set_base_cost() : Unknown node type");
    assert(false);
  }
}

void Node::update_present_congestion_cost(double pres_fac) {
  int occupancy = get_occupancy();
  if (occupancy < capacity) {
    set_present_congestion_cost(1);
  } else {
    set_present_congestion_cost(1 + (occupancy - capacity + 1) * pres_fac);
  }
}

int Node::count_connections_of_user(int net_id) {
  // std::shared_lock lock(users_connection_counts_mutex);
  for (auto &[id, count] : users_connection_counts) {
    if (id == net_id)
      return count;
  }
  return 0;
  //   if (users_connection_counts.empty() ||
  //   !users_connection_counts.contains(net_id))
  //     return 0;
  //   return users_connection_counts[net_id];
}

bool Node::will_overuse(int net_id) {
  int occ = get_occupancy();
  return occ > capacity ||
         (occ == capacity && count_connections_of_user(net_id) == 0);
}

bool Node::is_in_connection_bounding_box(ConnectionPtr connection) {
  return end_x_ > connection->get_xmin_bb() &&
         end_x_ < connection->get_xmax_bb() &&
         end_y_ > connection->get_ymin_bb() &&
         end_y_ < connection->get_ymax_bb();
}

RoutingGraph *Node::routing_graph = nullptr;
void Node::set_children() {
  if (children_set)
    return;
  children_set = true;

  // auto downhill_dev_nodes = dev_node()->get_downhill_nodes();
  const auto &downhill_dev_nodes = this->downhill_dev_nodes;

  for (auto downhill_dev_node : downhill_dev_nodes) {
    if (!routing_graph->is_partof_existing_route(this->dev_node(),
                                                 downhill_dev_node)) {
      if (routing_graph->is_preserved(downhill_dev_node) ||
          routing_graph->is_excluded(downhill_dev_node)) {
        children_skipped++;
        continue;
      }
    }
    NodePtr child_rnode = routing_graph->get_or_create(downhill_dev_node->id());
    if (child_rnode != nullptr &&
        !child_rnode->is_removed()) { // deal with removed nodes
      children.push_back(child_rnode);
    }
  }
}

void Node::set_parents() {
  if (parents_set)
    return;
  parents_set = true;

  // auto uphill_dev_nodes = dev_node()->get_uphill_nodes();
  const auto &uphill_dev_nodes = this->uphill_dev_nodes;

  for (auto uphill_dev_node : uphill_dev_nodes) {
    if (!routing_graph->is_partof_existing_route(this->dev_node(),
                                                 uphill_dev_node, BACKWARD)) {
      if (routing_graph->is_preserved(uphill_dev_node) ||
          routing_graph->is_excluded(uphill_dev_node)) {
        children_skipped++;
        continue;
      }
    }
    NodePtr uphill_rnode = routing_graph->get_or_create(uphill_dev_node->id());
    if (uphill_rnode != nullptr &&
        !uphill_rnode->is_removed()) { // deal with removed nodes
      uphill_nodes.push_back(uphill_rnode);
    }
  }
}
