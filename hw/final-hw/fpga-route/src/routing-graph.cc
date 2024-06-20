#include "routing-graph.h"
#include "ace-route.h"
#include "utils.h"
#include <chrono>
#include <iostream>
#include <map>
#include <omp.h>
#include <regex>
#include <unordered_set>

#include "DeviceResources.capnp.h"
#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include <capnp/serialize.h>

#include <log4cplus/logger.h>
#include <log4cplus/loggingmacros.h>

using namespace std;

static log4cplus::Logger logger = log4cplus::Logger::getInstance("ace.graph");

void Graph::create_node(int id_, int begin_x_, int begin_y_, int end_x_,
                        int end_y_, int wl_score_, std::string first_tile_name,
                        std::string first_wire_name) {
  nodes[id_] = Node::create(id_, begin_x_, begin_y_, end_x_, end_y_, wl_score_,
                            first_tile_name, first_wire_name);
}

void Graph::remove_node(int u) {
  if (nodes[u] != nullptr) {
    nodes[u]->set_removed();
    Node::destroy(nodes[u]);
  }
  // edges (u,*) removed
  // lazily remove (*,u)
}

void Graph::add_edge(int u, int v) { nodes[u]->add_children(nodes[v]); }

NodePtr RoutingGraph::get_or_create(int node_id, NodeType type) {
  // Return a rnode given `node_id` if it has been created, otherwise create
  // it Return nullptr if the node has been created and removed
  // Discriminate uncreated and removed nodes since the latter should not be
  // created again
  NodePtr rnode;
  if (rnodes[node_id] == nullptr) {
    auto &dev_node = dev()->nodes()[node_id];
    auto &info = node_info_[node_id];
    if (type != NodeType::UNKNOWN)
      info.type = type;
    rnode = Node::create(node_id, dev_node, info);

    std::unique_lock<std::mutex> lock(rnodes_write_mutex_array[node_id]);
    if (rnodes[node_id] == nullptr)
      rnodes[node_id] = rnode;
    else if (rnodes[node_id]->is_removed())
      rnode = nullptr;
    else
      rnode = rnodes[node_id];

  } else if (rnodes[node_id]->is_removed()) {
    rnode = nullptr;
  } else {
    rnode = rnodes[node_id];
  }
  return rnode;
}

void RoutingGraph::remove_node(
    int node_id) { // only called in AceRoute::parse_netlist()

  if (rnodes[node_id] != nullptr) {
    rnodes[node_id]->set_removed();
  } else {
    rnodes[node_id] = Node::create(node_id);
    rnodes[node_id]->set_removed();
  }
  // edges/children (u,*) removed
  // lazily remove (*,u)
}

RoutingGraph::RoutingGraph(device::DevicePtr dev_) : dev_(dev_) {
  Node::set_routing_graph(this);
  build_graph_dynamic();
  rnodes.resize(dev_->nodes().size(), nullptr);
  rnodes_write_mutex_array = new std::mutex[dev_->nodes().size()];
}
std::string RoutingGraph::get_base_tile_name(NodePtr node) {
  return name(node->dev_node()->base_tile());
}
std::string RoutingGraph::get_base_wire_name(NodePtr node) {
  return name(node->dev_node()->base_wire());
}

int RoutingGraph::get_node_from_site_pin(SitePin key) {

  auto [tile, site] = name_to_site.at(key.site_name);
  auto pin = name_to_pin.at(key.pin_name);
  auto pin_index = site->type()->pin_to_index().at(pin);
  auto wire = site->type_inst()->index_to_wire()[pin_index];
  auto node = tile->wire_to_node().at(wire);
  node_to_site_pin[node->id()] = make_pair(site, pin);
  return node->id();
}

SitePin RoutingGraph::get_site_pin_from_node(int node) {

  if (!node_to_site_pin.contains(node))
    return SitePin();
  auto [site, pin] = node_to_site_pin.at(node);
  return SitePin(name(site), name(pin));
}

PipData RoutingGraph::get_pip_data_from_nodes(int u, int v) {

  assert(rnodes[u] != nullptr && !rnodes[u]->is_removed());
  device::NodePtr u_dev_node = rnodes[u]->dev_node();
  for (auto [tile, wire0] : u_dev_node->wires()) {
    device::TileTypePtr tile_type = tile->type();
    std::vector<device::WirePtr> downhill_wires =
        tile_type->get_downhill_wires(wire0);
    for (device::WirePtr wire1 : downhill_wires) {
      if (v == (int)tile->wire_to_node().at(wire1)->id()) {
        if (tile_type->pips().contains(wire0, wire1)) {
          return (PipData){tile->name(), wire0->name(), wire1->name(),
                           /*is_forward=*/true};
        } else if (tile_type->pips().contains(wire1, wire0) &&
                   !tile_type->pips().is_directional(wire1, wire0)) {
          return (PipData){tile->name(), wire0->name(), wire1->name(),
                           /*is_forward=*/false};
        } else {
          LOG4CPLUS_FATAL_FMT(
              logger, "Cannot find pip data from node %d to node %d", u, v);
          exit(EXIT_FAILURE);
        }
      }
    }
  }
  LOG4CPLUS_FATAL_FMT(logger, "Cannot find pip data from node %d to node %d", u,
                      v);
  exit(EXIT_FAILURE);
}

int RoutingGraph::get_wl_score(string wire_name) {
  if (wl_score_map.contains(wire_name))
    return wl_score_map.at(wire_name);
  return wl_score_map[wire_name] = contest::compute_wl_score(wire_name);
}

bool RoutingGraph::shortest_path(int s, int t, vector<int> &path) {
  LOG4CPLUS_DEBUG(logger, "Finding shortest path from "
                              << get_site_pin_from_node(s) << " to "
                              << get_site_pin_from_node(t));
  const int INF = 0x3f3f3f3f;
  struct State {
    int dis = INF;
    int prev = -1;
  };

  struct QueueNode {
    int u, w;
    bool operator<(const QueueNode &t) const { return w > t.w; }
  };

  unordered_map<int, State> f;
  set<int> vis;
  priority_queue<QueueNode> q;

  f[s] = (State){0, -1};
  q.push((QueueNode){s, 0});
  static int mx = 0;
  int node_num = 0, vis_num = 0;
  while (!q.empty()) {
    auto [u, w] = q.top();
    q.pop();
    // cout << u << ' ' << w << endl;
    vis_num++;
    if (vis.find(u) != vis.end()) {
      continue;
    }
    vis.insert(u);
    if (u == t) {
      break;
    }
    mx = max(mx, w);
    node_num++;
    NodePtr node_u = get_or_create(u);
    for (auto node_v : node_u->get_children()) {
      int v = node_v->id();
      int w = 1;
      if (f[u].dis + w < f[v].dis) {
        f[v].dis = f[u].dis + w;
        f[v].prev = u;
        q.push((QueueNode){v, f[v].dis});
      }
    }
  }
  if (vis.find(t) == vis.end())
    return false;
  int u = t;
  while (u != s) {
    path.push_back(u);
    u = f[u].prev;
  }
  path.push_back(s);
  reverse(path.begin(), path.end());
  LOG4CPLUS_DEBUG_FMT(logger, "mx = %d node_num = %d vis_num = %d", mx,
                      node_num, vis_num);
  return true;
}

void RoutingGraph::build_graph_dynamic() {
  using namespace std::chrono;
  steady_clock::time_point last_time = steady_clock::now();
  auto elapsed_seconds = [](steady_clock::time_point last_time) -> double {
    return duration_cast<duration<double>>(steady_clock::now() - last_time)
        .count();
  };

  last_time = steady_clock::now();

  node_info_.resize(dev()->nodes().size());
  // omp_lock_t wl_score_lock;
  // omp_init_lock(&wl_score_lock);
// #pragma omp parallel for
#pragma omp parallel for num_threads(OMP_MAX_THREADS)
  for (auto node : dev()->nodes()) {
    // int wl_score = [&]() {
    //   int ret = 0;
    //   auto [begin_tile, begin_wire] = node->begin_wire();
    //   if (begin_tile->type()->is_INT()) {
    //     const auto &wire_name = name(begin_wire);
    //     auto it = wl_score_map.find(wire_name);
    //     if (it != wl_score_map.end()) {
    //       ret = (*it).second;
    //     } else {
    //       ret = contest::compute_wl_score(wire_name);
    //       omp_set_lock(&wl_score_lock);
    //       // critical section
    //       wl_score_map[wire_name] = ret;
    //       omp_unset_lock(&wl_score_lock);
    //     }
    //   }
    //   return ret;
    // }();

    auto [begin_tile, begin_wire] = node->begin_wire();
    auto [end_tile, _] = node->end_wire();

    auto &info = node_info_[node->id()];
    info.begin_x = begin_tile->x();
    info.begin_y = begin_tile->y();
    info.end_x = end_tile->x();
    info.end_y = end_tile->y();
    info.wl_score =
        contest::compute_wl_score(begin_tile->type()->intentcode(begin_wire));
    info.type = rwroute::node_type(dev(), node);
    info.downhill_dev_nodes = node->get_downhill_nodes();
    info.uphill_dev_nodes = node->get_uphill_nodes();
  }

  LOG4CPLUS_INFO_FMT(logger, "Build %ld node info: %.2lfs, %.2lfGB RAM",
                     node_info_.size(), elapsed_seconds(last_time),
                     getMemUsage() / 1024.0 / 1024.0 / 1024.0);

  last_time = steady_clock::now();

  int site_count = 0;
  for (auto [_, tile] : dev()->tiles()) {
    if (tile->wire_to_node().empty()) {
      continue;
    }
    for (auto site : tile->sites()) {
      site_count++;
      name_to_site[name(site)] = make_pair(tile, site);
    }
  }

  int pin_count = 0;
  for (auto tile_type : dev()->tile_types()) {
    unordered_set<uint32_t> site_type_ids;
    for (auto site_type_inst : tile_type->site_type_insts()) {
      auto site_type = site_type_inst->type();
      if (site_type_ids.contains(site_type->name()))
        continue;
      site_type_ids.insert(site_type->name());
      for (auto [pin, _] : site_type->pin_to_index()) {
        pin_count++;
        name_to_pin[name(pin)] = pin;
      }
    }
  }

  LOG4CPLUS_INFO_FMT(
      logger, "Build %d sites and %d (%ld) pins: %.2lfs, %.2lfGB RAM",
      site_count, pin_count, name_to_pin.size(), elapsed_seconds(last_time),
      getMemUsage() / 1024.0 / 1024.0 / 1024.0);
}

bool RoutingGraph::is_partof_existing_route(device::NodePtr start_node,
                                            device::NodePtr end_node,
                                            PBA_MODE mode) {
  NodePtr end_rnode = get_node(end_node->id());
  if (end_rnode == nullptr)
    return false;
  if (mode == NONE) {
    if (end_rnode->is_visited(ace_route->connections_routed))
      return false;
  } else if (mode == FORWARD) {
    if (end_rnode->is_visited_forward.load() == ace_route->connections_routed)
      return false;
  } else if (mode == BACKWARD) {
    if (end_rnode->is_visited_backward.load() == ace_route->connections_routed)
      return false;
  } else {
    assert(false);
  }
  // Presence of a prev pointer means that only that arc is allowed to enter
  // this end node
  NodePtr prev;
  if (mode != BACKWARD) {
    prev = end_rnode->get_prev();
  } else {
    prev = end_rnode->get_next();
  }
  if (prev != nullptr) {
    if (prev->id() == (int)start_node->id() && is_preserved(end_node)) {
      assert(get_preserved_net(start_node)->id() ==
             get_preserved_net(end_node)->id());
      return true;
    }
  }
  return false;
}

bool RoutingGraph::is_preserved(device::NodePtr node) {
  auto base_tile_id = node->base_tile()->name();
  auto base_wire_id = node->base_wire()->name();
  return preserved_tile_to_wireid_to_net.contains(base_tile_id) &&
         preserved_tile_to_wireid_to_net.at(base_tile_id)
             .contains(base_wire_id);
}

bool RoutingGraph::is_excluded(device::NodePtr node) {
  if (is_excluded_tile(node))
    return true;

  if (node->intentcode() == IntentCode::NODE_PINFEED) {
    // PINFEEDs can lead to a site pin, or into a Laguna tile
    NodePtr child_rnode = get_node(node->id());
    if (child_rnode != nullptr) {
      assert(child_rnode->type() == NodeType::PINFEED_I ||
             child_rnode->type() == NodeType::LAGUNA_I);
    } else {
      // child does not already exist in our routing graph, meaning it's not a
      // sink pin in our design, but it could be a LAGUNA_I
      if (dev()->laguna_tiles_to_specwires.empty()) {
        return true;
      }
      if (dev()->laguna_tiles_to_specwires.contains(
              node->base_tile()->name())) {
        if (dev()
                ->laguna_tiles_to_specwires.at(node->base_tile()->name())
                .contains(node->base_wire()->name())) {
          return false;
        }
      }
      return true;
    }
  }

  return false;
}

inline bool RoutingGraph::is_excluded_tile(device::NodePtr node) {
  auto tile = node->base_tile();
  return tile->type()->is_route_excluded();
}

NetPtr RoutingGraph::get_preserved_net(device::NodePtr node) {
  auto base_tile_id = node->base_tile()->name();
  auto base_wire_id = node->base_wire()->name();
  if (!preserved_tile_to_wireid_to_net.contains(base_tile_id))
    return nullptr;
  else if (!preserved_tile_to_wireid_to_net.at(base_tile_id)
                .contains(base_wire_id))
    return nullptr;
  else
    return preserved_tile_to_wireid_to_net.at(base_tile_id).at(base_wire_id);
}

NetPtr RoutingGraph::preserve(NetPtr net, device::NodePtr node) {
  auto base_tile_id = node->base_tile()->name();
  auto base_wire_id = node->base_wire()->name();
  if (!preserved_tile_to_wireid_to_net.contains(base_tile_id))
    preserved_tile_to_wireid_to_net[base_tile_id] = {};
  auto &wireid_to_net = preserved_tile_to_wireid_to_net.at(base_tile_id);
  auto old_net = wireid_to_net.contains(base_wire_id)
                     ? wireid_to_net.at(base_wire_id)
                     : nullptr;
  if (old_net == nullptr)
    wireid_to_net[base_wire_id] = net;
  return old_net;
}

bool RoutingGraph::unpreserve(device::NodePtr node) {
  auto base_tile_id = node->base_tile()->name();
  auto base_wire_id = node->base_wire()->name();
  if (!preserved_tile_to_wireid_to_net.contains(base_tile_id))
    return false;
  auto &wireid_to_net = preserved_tile_to_wireid_to_net.at(base_tile_id);
  if (!wireid_to_net.contains(base_wire_id))
    return false;
  wireid_to_net.erase(base_wire_id);
  return true;
}

std::ostream &operator<<(std::ostream &os, const SitePin &sp) {
  os << '(' << sp.site_name << ',' << sp.pin_name << ')';
  return os;
}
