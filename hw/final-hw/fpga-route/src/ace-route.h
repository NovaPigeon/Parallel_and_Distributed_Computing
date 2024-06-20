#ifndef ACE_ROUTE_H
#define ACE_ROUTE_H

#include "config.h"
#include "connection.h"
#include "mapped-file.h"
#include "net.h"
#include "routing-graph.h"
#include <queue>
#include <unordered_map>
#include <vector>

#include "PhysicalNetlist.capnp.h"

#include <taskflow/taskflow.hpp>

class AceRoute
{
public:
  AceRoute(std::string netlist_filename, std::string out_filename,
           Config config)
      : out_filename(out_filename), dev(device::Device::create()), rg(dev),
        config(config)
  {
    rg.set_ace_route(this);
    std::string ungzip_netlist_filename = netlist_filename + ".ungzip";
    ungzip(netlist_filename, ungzip_netlist_filename);
    netlist_file = std::make_unique<MappedFile>(ungzip_netlist_filename);
    parse_netlist();

    if (config.connections_log.size() > 0)
    {
      connections_log.open(config.connections_log);
      connections_log
          << "Iteration, Connection ID, Bbox, HPWL, Path Nodes, "
             "Path Cost, Nodes Pushed/Popped, Route Time (ms), Result\n";
    }
  }

  void route()
  {
    path_finder();
  }

  friend class RoutingGraph;

private:
  void path_finder();
  void path_finder_parallel(int n = 3);

  class CompareNodePtr
  {
  public:
    bool operator()(const NodePtr &a, const NodePtr &b) const
    {
      // greater-than comparator makes a min priority queue
      return a->get_lower_bound_total_path_cost() >
             b->get_lower_bound_total_path_cost();
    }
  };
  class CompareBackNodePtr
  {
  public:
    bool operator()(const NodePtr &a, const NodePtr &b) const
    {
      // greater-than comparator makes a min priority queue
      return a->get_lower_bound_total_path_cost_back() >
             b->get_lower_bound_total_path_cost_back();
    }
  };

  class NetInfo
  {
  public:
    enum class Type
    {
      GND,
      VCC,
      WIRE,
      UNKNOWN
    };
    std::vector<std::pair<SitePin, int>> source_sp_to_node;
    std::vector<std::pair<SitePin, int>> sink_sp_to_node;
    std::vector<device::PipInst> pips;

    void add_pip(device::PipInst pip) { pips.push_back(pip); }
    bool has_pip() { return !pips.empty(); }
    bool has_source() { return !source_sp_to_node.empty(); }
    bool is_static() { return type_ == Type::GND || type_ == Type::VCC; }

    ACCESSOR_RW(Type, type)
    ACCESSOR_RW_BOOL_DEFAULT(need_route, false)
  };

  std::unordered_map<int, NetInfo> id_to_netinfo;

  std::string out_filename;
  device::DevicePtr dev;
  RoutingGraph rg;
  Config config;
  std::ofstream connections_log;

  std::vector<NetPtr> nets; // nets need routing
  std::vector<NetPtr> all_nets;
  std::unique_ptr<kj::ArrayInputStream> stream;
  std::unique_ptr<capnp::InputStreamMessageReader> message;
  PhysicalNetlist::PhysNetlist::Reader netlist;
  std::unique_ptr<MappedFile> netlist_file;
  std::vector<std::string> str_list;
  std::unordered_map<int, std::unordered_map<int, std::set<int>>>
      node_to_net_to_next;
  std::unordered_map<std::string, int> str_to_id;
  std::vector<ConnectionPtr> connections;
  std::vector<ConnectionPtr> direct_connections;

  int route_iteration;
  int connections_routed = 0;
  int connections_routed_this_iteration = 0;
  double present_congestion_factor;
  std::set<NodePtr> overused_rnodes;
  unsigned long long nodes_pushed;
  unsigned long long nodes_popped;
  long overused_nodes_last_iteration = std::numeric_limits<long>::max();
  /*
   * For accurate statistics for nodes pushed/popped in parallel version, use:
   * std::atomic<unsigned long long> nodes_pushed;
   * std::atomic<unsigned long long> nodes_popped;
   * May cause performance degradation
   */
  unsigned long long children_skipped;

  void sort_connections();

  void initialize_path_finder();

  void rip_up(ConnectionPtr connection);

  void update_users_and_present_congestion_cost(ConnectionPtr connection);

  void evaluate_cost(ConnectionPtr connection, NodePtr u, NodePtr v);

  void evaluate_forward_cost(ConnectionPtr, NodePtr, NodePtr);
  void evaluate_backward_cost(ConnectionPtr, NodePtr, NodePtr);

  bool route_connection(ConnectionPtr connection,
                        int &connections_routed_local);
  bool route_connection_pba(ConnectionPtr connection,
                            int &connections_routed_local);

  friend int route_connection_batch(AceRoute *,
                                    const std::vector<int> &connections_id,
                                    int &connections_routed_local);

  void route_connections_pba(const std::vector<ConnectionPtr> &, int &);
  void route_connections_pba();

  void route_connection_ba(ConnectionPtr connection,
                           int &connections_routed_local,
                           bool allow_early_stop = false);

  double compute_forward_future_cost(ConnectionPtr connection, NodePtr rnode,
                                     int count_users = -1);
  double compute_backward_future_cost(ConnectionPtr connection, NodePtr rnode,
                                      int count_users = -1);
  double compute_forward_dis(ConnectionPtr connection, NodePtr v);
  double compute_backward_dis(ConnectionPtr connection, NodePtr v);

  double compute_total_node_cost(ConnectionPtr connection, NodePtr rnode,
                                 int count_users = -1);

  void route_direct_connection(ConnectionPtr connection);

  bool can_skip_by_node_type(NodePtr node, ConnectionPtr connection);

  double get_node_cost(NodePtr rnode, ConnectionPtr connection,
                       int count_same_source_users, double sharing_factor);

  std::vector<PhysicalNetlist::PhysNetlist::PhysSitePin::Reader>
  extract_sitepins(capnp::List<PhysicalNetlist::PhysNetlist::RouteBranch,
                               capnp::Kind::STRUCT>::Reader branches,
                   /*bool is_sink,*/ NetInfo &net_info,
                   std::set<std::pair<uint32_t, uint32_t>> &stub_wires);

  void parse_netlist();

  void write();

  int get_new_str_id(std::string str);

  bool should_route(ConnectionPtr connection);

  void initialize_net_and_connection();

  void finish_route_connection(ConnectionPtr connection, NodePtr rnode);
  void finish_route_connection_pba(ConnectionPtr connection, NodePtr rnode);

  bool save_routing(ConnectionPtr connection, NodePtr rnode);
  bool save_routing_pba(ConnectionPtr connection, NodePtr rnode);

  void update_cost();

  std::vector<ConnectionPtr> get_unroutable_connections();

  std::vector<ConnectionPtr> get_congested_connections();

  void prepare_write();

  void generate_routing_task(
      tf::Subflow &, const std::vector<std::vector<int>> &connection_partitions,
      int index, int depth, int target_depth,
      std::atomic<int> &routed_connections, int &iteration);

  std::vector<std::vector<int>> connections_partition(int);

  std::vector<std::vector<int>>
  connections_partition_local(const std::vector<int> &, bool);

  bool is_accessible(NodePtr rnode, ConnectionPtr connection);

  bool is_accessible_rg(NodePtr rnode, ConnectionPtr connection);

  bool is_accessible_pinfeed_i(NodePtr rnode, ConnectionPtr connection);

  void fix_routes();

  std::vector<NetPtr> find_illegal_routes();

  void build_driver_counts(NetPtr net);

  void route_direct_connections();

  std::vector<LightweightNodePtr> project_input_pin_to_INT_nodes(int);

  LightweightNodePtr project_output_pin_to_INT_node(int);

  std::vector<LightweightNodePtr> find_path_between_nodes(NodePtr source,
                                                          NodePtr sink);

  void preserve_net(NetPtr net);

  bool need_route(NetPtr net)
  {
    return id_to_netinfo[net->id()].is_need_route();
  }

  bool get_node_accessible(NodePtr rnode);

  void complete_indirect_connections();

  void print_failure_stats();

  bool proc_congested_connections();

  bool if_use_ba(ConnectionPtr connection);
};

#endif // ACE_ROUTE_H
