#ifndef NODE_H
#define NODE_H

#include "config.h"
#include "device.h"
#include <atomic>
#include <memory>
#include <mutex>
#include <set>
#include <shared_mutex>
#include <unordered_map>
#include <vector>

class RoutingGraph;

DECLARE_PTR(Node)
DECLARE_PTR(Connection)
enum class NodeType;

namespace rwroute {
IntentCode intentcode(const std::string &wire_name);
NodeType node_type(const device::DevicePtr &dev,
                   const device::NodePtr &dev_node);
double compute_base_cost(IntentCode code, int length);
} // namespace rwroute

namespace contest {
int compute_wl_score(const std::string &wire_name);
int compute_wl_score(const IntentCode &intentcode);
} // namespace contest

/**
 * @brief Bulk parameters for Node::create().
 */
struct NodeInfo {
  int begin_x;
  int begin_y;
  int end_x;
  int end_y;
  int wl_score;
  NodeType type;
  std::vector<device::NodePtr> downhill_dev_nodes;
  std::vector<device::NodePtr> uphill_dev_nodes;
};

class Node {
public:
  PTR_FACTORY(Node)
  PTR_RECYCLE(Node)

  bool operator==(const NodePtr &that) const;

  const static int capacity = 1;

  // Deprecated
  Node(int id_, int begin_x_, int begin_y_, int end_x_, int end_y_,
       int wl_score_, std::string first_tile_name, std::string first_wire_name)
      : id_(id_), begin_x_(begin_x_), begin_y_(begin_y_), end_x_(end_x_),
        end_y_(end_y_), wl_score_(wl_score_), first_tile_name(first_tile_name),
        first_wire_name(first_wire_name) {
    type_ = rwroute::node_type(nullptr, nullptr); // HACK
    init_ro(rwroute::intentcode(first_wire_name));
    init_rw();
  }

  Node(int id_, device::NodePtr dev_node_, const NodeInfo &info)
      : id_(id_), dev_node_(dev_node_), begin_x_(info.begin_x),
        begin_y_(info.begin_y), end_x_(info.end_x), end_y_(info.end_y),
        wl_score_(info.wl_score), type_(info.type),
        downhill_dev_nodes(info.downhill_dev_nodes),
        uphill_dev_nodes(info.uphill_dev_nodes) {
    init_ro();
    init_rw();
  }

  Node(int id) : id_(id) {} // Create a dummy node

  static void set_routing_graph(RoutingGraph *rg) { Node::routing_graph = rg; }
  const std::vector<NodePtr> &get_children() { // Override Node::get_children()
    set_children();
    return children;
  }
  const std::vector<NodePtr> &get_parents() {
    set_parents();
    return uphill_nodes;
  }
  void remove_all_children() { // Override Node::remove_all_children()
    children.clear();
    children_set = true;
  }

  void add_children(NodePtr to) { children.push_back(to); }

  int get_occupancy() {
    // std::shared_lock lock(users_connection_counts_mutex);
    return users_connection_counts.size();
  }

  bool is_overused() {
    // std::shared_lock lock(users_connection_counts_mutex);
    return get_occupancy() > capacity;
  }

  void decrement_user(int net_id) {
    // std::unique_lock lock(users_connection_counts_mutex);
    // assert(users_connection_counts.contains(net_id) &&
    //        users_connection_counts.at(net_id) > 0);
    // users_connection_counts[net_id]--;
    // if (users_connection_counts[net_id] == 0) {
    //   users_connection_counts.erase(net_id);
    // }

    for (auto it = users_connection_counts.begin();
         it != users_connection_counts.end(); it++) {
      if (it->first == net_id) {
        if (--it->second == 0) {
          users_connection_counts.erase(it);
        }
        return;
      }
    }
  }

  void increment_user(int net_id) {
    // std::unique_lock lock(users_connection_counts_mutex);
    // users_connection_counts[net_id]++;

    for (auto &[id, count] : users_connection_counts) {
      if (id == net_id) {
        count++;
        return;
      }
    }
    users_connection_counts.emplace_back(std::make_pair(net_id, 1));
  }

  int count_connections_of_user(int net_id);

  bool will_overuse(int net_id);

  void set_visited(int connection_id) { visited_ = connection_id; }
  bool is_visited(int connection_id) { return visited_ == connection_id; }

  void update_present_congestion_cost(double pres_fac);

  bool is_in_connection_bounding_box(ConnectionPtr connection);

  void add_driver(NodePtr driver) { drivers.insert(driver); }

  bool have_multiple_drivers() { return drivers.size() > 1; }

  // initialized in constructor
  ACCESSOR_RO(int, id)
  ACCESSOR_RO(device::NodePtr, dev_node)
  ACCESSOR_RO(int, begin_x)
  ACCESSOR_RO(int, begin_y)
  ACCESSOR_RO(int, end_x)
  ACCESSOR_RO(int, end_y)
  ACCESSOR_RO(double, wl_score)
  ACCESSOR_RO(NodeType, type)
  ACCESSOR_RO(int, base_tile_x)
  ACCESSOR_RO(int, base_tile_y)

  // initialized in init_ro()
  ACCESSOR_RO(int, length)
  ACCESSOR_RO(double, base_cost)

  // initialized in init_rw()
  ACCESSOR_RW(double, present_congestion_cost)
  ACCESSOR_RW(double, historical_congestion_cost)
  ACCESSOR_RW(double, upstream_path_cost)
  ACCESSOR_RW(double, lower_bound_total_path_cost)
  ACCESSOR_RW(int, accessible_rg)
  ACCESSOR_RW_BOOL(target)
  ACCESSOR_RW_BOOL(routed)
  ACCESSOR_RW_BOOL(removed)
  ACCESSOR_RW(NodePtr, prev)
  ACCESSOR_RW(int, visited_forward)
  ACCESSOR_RW(int, visited_backward)

  std::atomic<int> is_visited_forward;
  std::atomic<int> is_visited_backward;
  std::atomic<int> is_updated_forward;
  std::atomic<int> is_updated_backward;
  // for backward search only
  ACCESSOR_RW(double, upstream_path_cost_back)
  ACCESSOR_RW(double, lower_bound_total_path_cost_back)
  ACCESSOR_RW(NodePtr, next)

  int children_skipped = 0;

private:
  void init_ro(IntentCode code) {
    length_ = std::abs(begin_x() - end_x()) + std::abs(begin_y() - end_y());
    base_cost_ = rwroute::compute_base_cost(code, length());
    base_tile_x_ = dev_node()->base_tile()->x();
    base_tile_y_ = dev_node()->base_tile()->y();
  }
  void init_ro() {
    length_ = std::abs(begin_x() - end_x()) + std::abs(begin_y() - end_y());
    set_base_cost();
    base_tile_x_ = dev_node()->base_tile()->x();
    base_tile_y_ = dev_node()->base_tile()->y();
  }

  void init_rw() {
    present_congestion_cost_ = 1.;
    historical_congestion_cost_ = 1.;
    upstream_path_cost_ = 0.;
    lower_bound_total_path_cost_ = std::numeric_limits<double>::max();
    upstream_path_cost_back_ = 0.;
    lower_bound_total_path_cost_back_ = std::numeric_limits<double>::max();
    accessible_rg_ = -1;

    is_target_ = false;
    is_routed_ = false;
    is_removed_ = false;
    prev_ = nullptr;
    next_ = nullptr;

    visited_ = -1;
    is_visited_forward.store(-1);
    is_visited_backward.store(-1);
    visited_forward_ = -1;
    visited_backward_ = -1;
  }

  void set_base_cost();

  int visited_; // connection number of the last visit
  std::vector<NodePtr> children;
  std::vector<NodePtr> uphill_nodes;
  //   std::unordered_map<int, int>
  //       users_connection_counts; // net id to connection count
  std::vector<std::pair<int, int>> users_connection_counts;
  // std::shared_mutex users_connection_counts_mutex;

  std::string first_tile_name;
  std::string first_wire_name;

  std::set<NodePtr> drivers;
  std::vector<device::NodePtr> downhill_dev_nodes;
  std::vector<device::NodePtr> uphill_dev_nodes;

  static RoutingGraph *routing_graph;
  bool children_set = false;
  bool parents_set = false;
  void set_children();
  void set_parents();

  friend class RoutingGraph;
};

struct LightweightNode;
// Use shared_ptr to avoid memory leak
typedef std::shared_ptr<LightweightNode> LightweightNodePtr;

struct LightweightNode {
  static LightweightNodePtr create(device::NodePtr dev_node_) {
    return std::make_shared<LightweightNode>(dev_node_);
  }
  static void destroy(LightweightNodePtr &entity) { entity.reset(); }

  LightweightNode(device::NodePtr dev_node_) : dev_node_(dev_node_){};

  ACCESSOR_RO(device::NodePtr, dev_node)
  ACCESSOR_RW(LightweightNodePtr, prev)

  int id() { return dev_node_->id(); }
};

enum class NodeType {
  /**
   * Denotes {@link RouteNode} objects that correspond to the output pins of
   * {@link Net} Objects, typically the source {@link RouteNode} Objects of
   * {@link Connection} Objects.
   */
  PINFEED_O,

  /**
   * Denotes {@link RouteNode} objects that correspond to input pins of {@link
   * Net} Objects, typically the sink {@link RouteNode} Objects of {@link
   * Connection} Objects.
   */
  PINFEED_I,

  /**
   * Denotes {@link RouteNode} objects that are created based on {@link Node}
   * Objects that have an {@link IntentCode} of NODE_PINBOUNCE.
   */
  PINBOUNCE,

  /**
   * Denotes {@link RouteNode} objects that correspond to a super long line
   * {@link Node}, i.e. nodes that span two SLRs.
   */
  SUPER_LONG_LINE,

  /**
   * Denotes {@link RouteNode} objects that correspond to {@link Node} objects
   * that enter a Laguna tile from an INT tile, or those Laguna tile nodes
   * leading to a SUPER_LONG_LINE.
   */
  LAGUNA_I,

  /**
   * Denotes other wiring {@link RouteNode} Objects
   * that are created for routing {@link Connection} Objects.
   */
  WIRE,

  // null value (not specified)
  UNKNOWN
};

#endif // NODE_H
