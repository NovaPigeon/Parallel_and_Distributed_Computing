#ifndef ROUTING_GRAPH_H
#define ROUTING_GRAPH_H

#include "config.h"
#include "device.h"
#include "mapped-file.h"
#include "net.h"
#include "node.h"
#include "utils.h"
#include <cassert>
#include <memory>
#include <queue>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "DeviceResources.capnp.h"
#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include <capnp/serialize.h>

class AceRoute;

class Graph {
public:
  void create_node(int id_, int begin_x_, int begin_y_, int end_x_, int end_y_,
                   int wl_score_, std::string first_tile_name,
                   std::string first_wire_name);

  NodePtr get_node(int u) { return nodes.at(u); }

  void remove_node(int u);

  void add_edge(int u, int v);

  const std::unordered_map<int, NodePtr> &get_nodes() { return nodes; }

private:
  std::unordered_map<int, NodePtr> nodes;
};

struct SitePin {
  std::string site_name;
  std::string pin_name;

  SitePin(std::string site_name = "", std::string pin_name = "")
      : site_name(site_name), pin_name(pin_name) {}

  bool operator==(const SitePin &other) const {
    return site_name == other.site_name && pin_name == other.pin_name;
  }

  friend std::ostream &operator<<(std::ostream &os, const SitePin &sp);
};

struct SitePinHasher {
  std::size_t operator()(const SitePin &k) const {
    auto h1 = std::hash<std::string>{}(k.site_name);
    auto h2 = std::hash<std::string>{}(k.pin_name);

    return ((h1 ^ (h2 << 1)) >> 1);
  }
};

class RoutingGraph {
public:
  RoutingGraph(device::DevicePtr dev_);
  ~RoutingGraph() { delete[] rnodes_write_mutex_array; }
  void set_ace_route(AceRoute *ace) { ace_route = ace; }

  // Return a rnode given `node_id` if it has been created, otherwise create
  // it Return nullptr if the node has been created and removed
  // When set_children(), no need to specify type
  NodePtr get_or_create(int node_id, NodeType type = NodeType::UNKNOWN);

  // Return a node if it has been created, else nullptr
  //* No need for a read lock since it is called with a created node
  NodePtr get_node(int node_id) {
    if (rnodes[node_id] == nullptr)
      return nullptr;
    if (rnodes[node_id]->is_removed())
      return nullptr;
    return rnodes[node_id];
  }

  // only called in AceRoute::parse_netlist()
  void remove_node(int node_id);

  int get_node_from_site_pin(SitePin key);
  SitePin get_site_pin_from_node(int node);
  PipData get_pip_data_from_nodes(int u, int v);

  bool shortest_path(int s, int t, std::vector<int> &path);

  bool is_preserved(device::NodePtr node);
  bool is_excluded(device::NodePtr node);
  bool is_excluded_tile(device::NodePtr node);
  bool is_partof_existing_route(device::NodePtr start_node,
                                device::NodePtr end_node, PBA_MODE mode = NONE);

  NetPtr preserve(NetPtr net, device::NodePtr node);

  bool unpreserve(device::NodePtr node);

  NetPtr get_preserved_net(device::NodePtr node);

  const device::DevicePtr dev() const { return dev_; }
  const std::vector<NodePtr> &get_rnodes() { return rnodes; }
  template <typename T> const std::string &name(const T &entity) const {
    return dev()->name(entity);
  }
  std::string get_base_tile_name(NodePtr node);
  std::string get_base_wire_name(NodePtr node);

  NodeInfo &node_info(int node_id) { return node_info_[node_id]; }

private:
  void build_graph_dynamic();

  // std::unordered_map<int, NodePtr> rnodes;
  std::vector<NodePtr> rnodes;
  std::mutex *rnodes_write_mutex_array;
  device::DevicePtr dev_;
  AceRoute *ace_route = nullptr;

  std::unordered_map<std::string, std::pair<device::TilePtr, device::SitePtr>>
      name_to_site;
  std::unordered_map<std::string, device::SitePinPtr> name_to_pin;
  std::unordered_map<int, std::pair<device::SitePtr, device::SitePinPtr>>
      node_to_site_pin;
  std::unordered_map<uint32_t, std::unordered_map<uint32_t, NetPtr>>
      preserved_tile_to_wireid_to_net;

  std::vector<NodeInfo> node_info_;

  std::unordered_map<std::string, int> wl_score_map;
  int get_wl_score(std::string wire_name);
};

#endif // ROUTING_GRAPH_H
