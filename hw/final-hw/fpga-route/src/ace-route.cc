#include "ace-route.h"
#include "route-fixer.h"
#include <algorithm>
#include <cassert>
#include <chrono>
#include <omp.h>
#include <limits.h>
#include <log4cplus/logger.h>
#include <log4cplus/loggingmacros.h>

using namespace std;
using namespace std::chrono;

static log4cplus::Logger logger = log4cplus::Logger::getInstance("ace.route");

static auto elapsed_seconds = [](steady_clock::time_point last_time) -> double
{
  return duration_cast<duration<double>>(steady_clock::now() - last_time)
      .count();
};
static auto elapsed_milliseconds =
    [](steady_clock::time_point last_time) -> double
{
  return duration_cast<duration<double, std::milli>>(steady_clock::now() -
                                                     last_time)
      .count();
};

bool AceRoute::should_route(ConnectionPtr connection)
{
  // if (routeIteration > 1) {
  //     if (connection.getCriticality() > minRerouteCriticality) {
  //         return true;
  //     }
  // }
  return !connection->sink_INT()->is_routed() || connection->is_congested();
}

void AceRoute::rip_up(ConnectionPtr connection)
{
  for (NodePtr rnode : connection->get_rnodes())
  {
    rnode->decrement_user(connection->net()->id());
    rnode->update_present_congestion_cost(present_congestion_factor);
  }
}

void AceRoute::update_users_and_present_congestion_cost(
    ConnectionPtr connection)
{
  for (NodePtr rnode : connection->get_rnodes())
  {
    rnode->increment_user(connection->net()->id());
    rnode->update_present_congestion_cost(present_congestion_factor);
  }
}

inline double AceRoute::get_node_cost(NodePtr rnode, ConnectionPtr connection,
                                      int count_same_source_users,
                                      double sharing_factor)
{
  bool has_same_source_users = count_same_source_users != 0;
  double present_congestion_cost;

  if (has_same_source_users)
  { // the rnode is used by other connection(s) from
    // the same net
    int over_occupancy = rnode->get_occupancy() - rnode->capacity;
    present_congestion_cost = 1 + over_occupancy * present_congestion_factor;
  }
  else
  {
    present_congestion_cost = rnode->get_present_congestion_cost();
  }

  double bias_cost = 0;
  if (!rnode->is_target() && rnode->type() != NodeType::SUPER_LONG_LINE)
  {
    NetPtr net = connection->net();
    bias_cost = (double)rnode->base_cost() / net->connections().size() *
                (abs(rnode->end_x() - net->x_center()) +
                 abs(rnode->end_y() - net->y_center())) /
                net->double_hpwl();
  }

  return rnode->base_cost() * rnode->get_historical_congestion_cost() *
             present_congestion_cost / sharing_factor +
         bias_cost;
}

void AceRoute::evaluate_cost(ConnectionPtr connection, NodePtr rnode,
                             NodePtr child_rnode)
{
  int count_source_uses =
      child_rnode->count_connections_of_user(connection->net()->id());
  int sharing_factor = 1 + count_source_uses;
  child_rnode->set_prev(rnode);
  double new_partial_path_cost =
      rnode->get_upstream_path_cost() +
      get_node_cost(child_rnode, connection, count_source_uses,
                    sharing_factor) +
      (1 - config.wirelength_weight) * child_rnode->length() / sharing_factor;

  int child_x = child_rnode->end_x();
  int child_y = child_rnode->end_y();
  NodePtr sink = connection->sink_INT();
  int sink_x = sink->begin_x();
  int sink_y = sink->begin_y();
  int delta_x = abs(child_x - sink_x);
  int delta_y = abs(child_y - sink_y);
  int distance_to_sink = delta_x + delta_y;
  double new_total_path_cost =
      new_partial_path_cost +
      config.wirelength_weight * distance_to_sink / sharing_factor;

  child_rnode->set_upstream_path_cost(new_partial_path_cost);
  child_rnode->set_lower_bound_total_path_cost(new_total_path_cost);
}

bool AceRoute::save_routing(ConnectionPtr connection, NodePtr rnode)
{
  // LOG4CPLUS_DEBUG(logger, "Saving connection: " << connection->net()->id()<<
  // " at index: "<<std::distance(this->connections.begin(),
  // std::find(this->connections.begin(), this->connections.end(),
  // connection)));
  if (rnode != connection->sink_INT())
  {
    assert(rnode == connection->sink_INT());
  }
  connection->reset_route();

  while (rnode != nullptr)
  {
    connection->add_rnode(rnode);
    rnode = rnode->get_prev();
  }

  const auto &rnodes = connection->get_rnodes();
  if (rnodes.size() == 1)
  {
    // No prev pointer from sink rnode -> not routed
    return false;
  }
  assert(rnodes.back() == connection->source_INT());
  return true;
}

void AceRoute::finish_route_connection(ConnectionPtr connection,
                                       NodePtr rnode)
{
  bool routed = save_routing(connection, rnode);
  if (routed)
  {
    connection->sink_INT()->set_routed(routed);
    update_users_and_present_congestion_cost(connection);
  }
  else
  {
    connection->reset_route();
  }
}

bool AceRoute::is_accessible(NodePtr rnode, ConnectionPtr connection)
{
  return !config.use_bounding_box ||
         rnode->is_in_connection_bounding_box(connection);
}

bool AceRoute::get_node_accessible(NodePtr rnode)
{
  int node_accessible = rnode->get_accessible_rg();
  if (node_accessible == -1)
  {
    auto base_tile = rnode->dev_node()->base_tile();
    auto base_wire = rnode->dev_node()->base_wire();

    node_accessible =
        (int)(!base_tile->type()->is_INT() ||
              !dev->accessible_wires_same_col.contains(base_wire->name()));

    rnode->set_accessible_rg(node_accessible);
  }
  return node_accessible;
}

bool AceRoute::is_accessible_rg(NodePtr rnode, ConnectionPtr connection)
{
  if (get_node_accessible(rnode))
    return true;

  auto sink = connection->sink_INT();
  return (rnode->base_tile_x() == sink->base_tile_x() &&
          abs(rnode->base_tile_y() - sink->base_tile_y()) <= 1);
}

bool AceRoute::is_accessible_pinfeed_i(NodePtr rnode,
                                       ConnectionPtr connection)
{
  assert(rnode->type() == NodeType::PINFEED_I);
  if (rnode->is_target())
  {
    return true;
  }
  if (rnode->count_connections_of_user(connection->net()->id()) == 0 ||
      rnode->dev_node()->intentcode() != IntentCode::NODE_PINBOUNCE)
  {
    // Inaccessible if child is not a sink pin of another connection on the same
    // net, or it is not a PINBOUNCE node
    return false;
  }
  return true;
}

// serial path finder
bool AceRoute::route_connection(ConnectionPtr connection,
                                int &connections_routed_local)
{
  // std::cout << "Routig Connection " << connection->net()->id() << std::endl;
  rip_up(connection);
  connections_routed_local++;
  std::priority_queue<NodePtr, std::vector<NodePtr>, CompareNodePtr> q;
  auto begin_time = steady_clock::now();

  assert(connection->source_INT()->get_prev() == nullptr);
  connection->source_INT()->set_visited(connections_routed_local);
  connection->source_INT()->set_upstream_path_cost(0);
  connection->source_INT()->set_lower_bound_total_path_cost(0);
  q.push(connection->source_INT());
  connection->sink_INT()->set_target(true);
  bool success = false;
  decltype(nodes_pushed) nodes_pushed_this_connection = 1;
  decltype(nodes_popped) nodes_popped_this_connection = 0;

  NodePtr u;
  while (!q.empty())
  {
    u = q.top();
    q.pop();
    nodes_popped_this_connection++;
    if (u->is_target())
    {
      success = true;
      break;
    }
    for (NodePtr v : u->get_children())
    {
      if (v->is_visited(connections_routed_local))
        continue;
      if (v->is_target())
      {
        bool early_termination = false;
        if (v == connection->sink_INT() &&
            connection->alt_source_INT() == nullptr)
        {
          assert(v->get_occupancy() == 0
                 /*|| v->dev_node()->get_intent_code() == IntentCode::NODE_PINBOUNCE*/);
          early_termination = true;
        }
        else
        {
          early_termination = !v->will_overuse(connection->net()->id());
        }
        if (early_termination)
        {
          assert(!v->is_visited(connections_routed_local));
          // nodes_pushed_this_connection =
          //     nodes_popped_this_connection + q.size();
          clear_container(q);
        }
      }
      else
      {
        if (!is_accessible(v, connection))
          continue;

        if (can_skip_by_node_type(v, connection))
          continue;
      }
      evaluate_cost(connection, u, v);
      v->set_visited(connections_routed_local);
      q.push(v);
      nodes_pushed_this_connection++;

      if (v->is_target())
      {
        // if v is target, then there is only v in q
        // assert(q.size() == 1 && q.top() == v);
        break;
      }
    }
    children_skipped += u->children_skipped;
  }
  nodes_pushed += nodes_pushed_this_connection;
  nodes_popped += nodes_popped_this_connection;
  // LOG4CPLUS_TRACE_FMT(logger,
  //                     "  connect %d: %lld nodes pushed ; %lld nodes popped",
  //                     connections_routed, nodes_pushed_this_connection,
  //                     nodes_popped_this_connection);
  connection->sink_INT()->set_target(false);
  if (success)
  {
    finish_route_connection(connection, u);
    assert(connection->sink_INT()->is_routed());
  }
  else
  {
    assert(q.empty());
    connection->reset_route();
    assert(!connection->sink_INT()->is_routed());
  }
  connection->set_nodes_popped_last_iter(nodes_popped_this_connection);

  auto connection_route_time = elapsed_milliseconds(begin_time);
  connection->set_route_time_last_iter(connection_route_time);

  if (config.connections_log.size() > 0)
  {
    connections_log << route_iteration << ", " << connection->get_id_indirect()
                    << ", (" << connection->get_xmin_bb() << " "
                    << connection->get_ymin_bb() << " "
                    << connection->get_xmax_bb() << " "
                    << connection->get_ymax_bb() << "), " << connection->hpwl()
                    << ", " << connection->get_rnodes().size() << ", "
                    << connection->sink_INT()->get_upstream_path_cost() << ", "
                    << nodes_pushed_this_connection << "/"
                    << nodes_popped_this_connection << ", "
                    << connection_route_time << ", "
                    << (success ? "success" : "fail") << std::endl;
  }
  return success;
}

bool AceRoute::can_skip_by_node_type(NodePtr node, ConnectionPtr connection)
{
  switch (node->type())
  {
  case NodeType::WIRE:
    if (!is_accessible_rg(node, connection))
    {
      return true;
    }
    break;
  case NodeType::PINFEED_I:
    if (!is_accessible_pinfeed_i(node, connection))
    {
      return true;
    }
    break;
  case NodeType::PINBOUNCE:
    if (!is_accessible_rg(node, connection))
    {
      return true;
    }
    break;
  case NodeType::LAGUNA_I:
    /* if (!connection.isCrossSLR() ||
                      connection.getSinkRnode().getSLRIndex() ==
             childRNode.getSLRIndex())
    */
    return true;
  case NodeType::SUPER_LONG_LINE:
    // Do nothing
    break;
  default:
    assert(node->type() != NodeType::UNKNOWN);
    // LOG4CPLUS_ERROR_FMT(logger, "Unexpected node type %d",
    // v->get_type());
  }
  return false;
}

/**
 * Gets a list of Node instances that connect an input
 * SitePinInst instance to an INT Tile instance.
 * @param input The input pin.
 * @return A list of nodes from the input SitePinInst to an INT tile.
 */
std::vector<LightweightNodePtr>
AceRoute::project_input_pin_to_INT_nodes(int node_id)
{
  std::vector<LightweightNodePtr> sink_to_sb_path;
  LightweightNodePtr sink = LightweightNode::create(dev->nodes().at(node_id));
  sink->set_prev(nullptr);
  std::queue<LightweightNodePtr> q;
  q.push(sink);
  int watchdog = 1000;

  while (!q.empty())
  {
    LightweightNodePtr n = q.front();
    q.pop();
    if (n->dev_node()->base_tile()->type()->is_INT())
    {
      while (n != nullptr)
      {
        sink_to_sb_path.push_back(n);
        n = n->get_prev();
      }
      return sink_to_sb_path;
    }
    for (auto uphill : n->dev_node()->get_uphill_nodes())
    {
      if (uphill->get_uphill_nodes().size() == 0)
        continue;
      LightweightNodePtr prev = LightweightNode::create(uphill);
      prev->set_prev(n);
      q.push(prev);
    }
    watchdog--;
    if (watchdog < 0)
      break;
  }
  return sink_to_sb_path;
}

/**
 * Gets a Node instance that connects to an INT Tile instance
 * from an output SitePinInst instance.
 * @param output The output pin.
 * @return A node that connects to an INT tile from an output pin.
 */
LightweightNodePtr AceRoute::project_output_pin_to_INT_node(int node_id)
{
  int watchdog = 5;

  // Starting from the SPI's connected node, for each node in queue
  // return the first downhill node that is in an Interconnect tile.
  // Otherwise, restart the queue with all such downhill nodes and repeat.
  // No backtracking.
  std::queue<LightweightNodePtr> q;
  q.push(LightweightNode::create(dev->nodes().at(node_id)));
  while (!q.empty() && watchdog >= 0)
  {
    LightweightNodePtr node = q.front();
    q.pop();
    watchdog--;
    // FIXME: Use isInterConnect() instead of is_INT() to support 7-series
    assert(!node->dev_node()->base_tile()->type()->is_INT());

    auto downhill_nodes = node->dev_node()->get_downhill_nodes();
    if (downhill_nodes.empty())
      continue;

    while (!q.empty())
    { // q.clear()
      LightweightNodePtr ptr = q.front();
      q.pop();
      LightweightNode::destroy(ptr);
    }

    for (auto downhill : downhill_nodes)
    {
      // FIXME: Use isInterConnect() instead of is_INT() to support 7-series
      if (downhill->base_tile()->type()->is_INT())
        return node;
      q.push(LightweightNode::create(downhill));
    }
  }
  return nullptr;
}

void AceRoute::preserve_net(NetPtr net)
{
  NetInfo &net_info = id_to_netinfo[net->id()];
  for (auto [_, node_id] : net_info.source_sp_to_node)
  {
    rg.preserve(net, rg.dev()->nodes()[node_id]);
  }
  for (auto [_, node_id] : net_info.sink_sp_to_node)
  {
    rg.preserve(net, rg.dev()->nodes()[node_id]);
  }
  for (auto pip : net_info.pips)
  {
    rg.preserve(net, rg.dev()->nodes()[pip.node0()->id()]);
    rg.preserve(net, rg.dev()->nodes()[pip.node1()->id()]);
  }
}

// modified by ruiyang 2024/05/22
// to similify question
// 1> delete any connections which needs an alt_source
// 2> delete any direct connections (every indirect connection is from source_int to sink_int)
void AceRoute::initialize_net_and_connection()
{
  int nets_preserved = 0;
  int indirect_conn_id = 0;
  for (auto [net_id, net_info] : id_to_netinfo)
  {
    NetPtr net = Net::create(net_id);
    if (need_route(net))
    {
      NodePtr source = [&]() -> NodePtr
      {
        auto it = net_info.source_sp_to_node.begin();
        int source_id = it->second;
        return rg.get_or_create(source_id, NodeType::PINFEED_O);
      }();
      assert(source != nullptr);

      NodePtr alt_source = [&]() -> NodePtr
      {
        assert(net_info.source_sp_to_node.size() <= 2);
        if (net_info.source_sp_to_node.size() == 2)
        {
          auto it = net_info.source_sp_to_node.begin();
          auto alt_source_id = (++it)->second;
          return rg.get_or_create(alt_source_id, NodeType::PINFEED_O);
        }
        return nullptr;
      }();

      // to similify question
      // 1> delete any connections which needs an alt_source
      if (alt_source)
        continue;

      NodePtr source_INT_node = [&]() -> NodePtr
      {
        auto node = project_output_pin_to_INT_node(source->id());
        if (node)
          return rg.get_or_create(node->id(), NodeType::PINFEED_O);
        return nullptr;
      }();

      // NodePtr alt_source_INT_node = [&]() -> NodePtr {
      //   if (alt_source) {
      //     assert(alt_source->id() != source->id());
      //     auto node = project_output_pin_to_INT_node(alt_source->id());
      //     if (node)
      //       return rg.get_or_create(node->id(), NodeType::PINFEED_O);
      //   }
      //   return nullptr;
      // }();
      if (source_INT_node != nullptr)
      {
        source_INT_node->dev_node()->set_source_INT(true);
      }

      for (auto [_, sink_id] : net_info.sink_sp_to_node)
      {
        NodePtr sink = rg.get_or_create(sink_id, NodeType::PINFEED_I);
        std::vector<LightweightNodePtr> sink_to_sb_nodes =
            project_input_pin_to_INT_nodes(sink_id);
        assert(sink != nullptr);

        NodePtr sink_INT_node =
            (!sink_to_sb_nodes.empty())
                ? rg.get_or_create(sink_to_sb_nodes.front()->id(),
                                   NodeType::PINFEED_I)
                : nullptr;

        ConnectionPtr connection;
        // if (sink_INT_node == nullptr ||
        //     (source_INT_node == nullptr && alt_source_INT_node == nullptr)) {
        //   connection = Connection::create(net, source, sink);
        //   direct_connections.push_back(connection);
        //   connection->set_direct(true);
        // } else {
        //   if (source_INT_node != nullptr) {
        //     connection = Connection::create(net, source, sink, alt_source,
        //                                     source_INT_node, sink_INT_node,
        //                                     alt_source_INT_node);
        //   } else {
        //     assert(alt_source_INT_node != nullptr);
        //     // Primary source does not reach the fabric (e.g. COUT)
        //     // just use alternate source
        //     connection =
        //         Connection::create(net, alt_source, sink, nullptr,
        //                            alt_source_INT_node, sink_INT_node, nullptr);
        //   }

        // to similify question
        // 2> delete any direct connections (every indirect connection is from source_int to sink_int)
        if (sink_INT_node && source_INT_node)
        {
          connection = Connection::create(net, source, sink, nullptr,
                                          source_INT_node, sink_INT_node,
                                          nullptr);
          connections.push_back(connection);
          connection->set_id_indirect(indirect_conn_id++);
          connection->set_direct(false);
          connection->compute_hpwl();

          net->add_connection(connection);
        }
      }

      // LOG4CPLUS_DEBUG_FMT(logger, "Net %d : Connection (%d,%d) created",
      //                     net->id(), source->id(), sink->id());

      net->compute();
      nets.push_back(net);
    } // if (need_route(net))

    all_nets.push_back(net);

    if (net_info.has_pip() || !net_info.has_source() || net_info.is_static())
    {
      preserve_net(net);
      nets_preserved++;
    }
  }
  LOG4CPLUS_INFO(logger, "Initialize " << nets.size() << " nets to route and "
                                       << nets_preserved << " nets preserved");

  if (config.use_bounding_box)
  {
    for (auto connection : connections)
    {
      connection->compute_bounding_box(config.bounding_box_extension_x,
                                       config.bounding_box_extension_y);
    }
  }

  LOG4CPLUS_INFO_FMT(logger, "Create %zu connections, %zu direct, %zu indirect",
                     connections.size() + direct_connections.size(),
                     direct_connections.size(), connections.size());
}

void AceRoute::initialize_path_finder()
{
  // connections_routed = 0;
  nodes_pushed = 0;
  nodes_popped = 0;
  present_congestion_factor = config.initial_present_congestion_factor;
  initialize_net_and_connection();
}

void AceRoute::sort_connections()
{
  sort(connections.begin(), connections.end(),
       [](const ConnectionPtr &a, const ConnectionPtr &b)
       {
         int net_size_a = a->net()->connections().size();
         int net_size_b = b->net()->connections().size();
         if (net_size_a != net_size_b)
         {
           return net_size_a > net_size_b;
         }
         return a->hpwl() < b->hpwl();
       });
}

// Do not modify the mechanism of overuse checking
// This will judge whether you have completed a node-disjoint routing
void AceRoute::update_cost()
{
  present_congestion_factor *= config.present_congestion_multiplier;
  overused_rnodes.clear();
  for (auto rnode : rg.get_rnodes())
  {
    if (rnode == nullptr || rnode->is_removed())
      continue;
    int overuse = rnode->get_occupancy() - rnode->capacity;
    if (overuse == 0)
    {
      rnode->set_present_congestion_cost(1 + present_congestion_factor);
    }
    else if (overuse > 0)
    {
      overused_rnodes.insert(rnode);
      rnode->set_present_congestion_cost(1 + (overuse + 1) *
                                                 present_congestion_factor);
      rnode->set_historical_congestion_cost(
          rnode->get_historical_congestion_cost() +
          overuse * config.historical_congestion_factor);
    }
    else
    {
      assert(rnode->get_present_congestion_cost() == 1);
    }
  }
}

vector<ConnectionPtr> AceRoute::get_unroutable_connections()
{
  vector<ConnectionPtr> unrouted_connections;
  for (ConnectionPtr connection : connections)
  {
    if (!connection->sink_INT()->is_routed())
    {
      unrouted_connections.push_back(connection);
    }
  }
  return unrouted_connections;
}

vector<ConnectionPtr> AceRoute::get_congested_connections()
{
  vector<ConnectionPtr> congested_connections;
  for (ConnectionPtr connection : connections)
  {
    if (connection->is_congested())
    {
      congested_connections.push_back(connection);
    }
  }
  return congested_connections;
}

void AceRoute::build_driver_counts(NetPtr net)
{
  for (auto connection : net->connections())
  {
    auto &rnodes = connection->get_rnodes();
    NodePtr driver = nullptr;
    for (int i = (int)rnodes.size() - 1; i >= 0; i--)
    {
      NodePtr rnode = rnodes[i];
      if (driver != nullptr)
      {
        rnode->add_driver(driver);
      }
      driver = rnode;
    }
  }
}

vector<NetPtr> AceRoute::find_illegal_routes()
{
  vector<NetPtr> illegal_routes;
  for (NetPtr net : nets)
  {
    build_driver_counts(net);
    for (auto connection : net->connections())
    {
      if (connection->use_rnodes_with_multiple_drivers())
      {
        illegal_routes.push_back(net);
        break;
      }
    }
  }
  return illegal_routes;
}

void AceRoute::fix_routes()
{
  vector<NetPtr> illegal_routes = find_illegal_routes();
  for (auto net : illegal_routes)
  {
    for (auto connection : net->connections())
    {
      if (!connection->is_direct())
      {
        try
        {
          rip_up(connection);
        }
        catch (const std::exception &e)
        {
          LOG4CPLUS_ERROR(logger, "Net " << net->id() << " : Connection ("
                                         << connection->source()->id() << ","
                                         << connection->source_INT()->id()
                                         << ",...,"
                                         << connection->sink_INT()->id() << ","
                                         << connection->sink()->id()
                                         << ") cannot be ripped up");
          LOG4CPLUS_ERROR(logger, e.what());
        }
      }
    }
    RouteFixer route_fixer(net);
    route_fixer.fix_routes();
  }
}

void AceRoute::route_direct_connections()
{
  auto last_time = steady_clock::now();
  LOG4CPLUS_INFO(logger, "");
  LOG4CPLUS_INFO_FMT(logger, "Route %zu direct connections",
                     direct_connections.size());
  for (ConnectionPtr connection : direct_connections)
  {
    route_direct_connection(connection);
  }
  LOG4CPLUS_INFO(logger,
                 "Time (s): " << elapsed_seconds(last_time) << " | RAM (GB): "
                              << getMemUsage() / 1024.0 / 1024.0 / 1024.0);
  LOG4CPLUS_INFO(logger, "");
}

void AceRoute::route_direct_connection(ConnectionPtr connection)
{
  NodePtr source = connection->source();
  NodePtr sink = connection->sink();
  auto path = find_path_between_nodes(source, sink);
  if (!path.empty())
  {
    if (path.size() > 1)
    {
      for (auto node : path)
      {
        connection->add_rnode(rg.get_or_create(node->id()));
        LightweightNode::destroy(node);
      }
    }
    else
    {
      assert(path.size() == 1);
      // do nothing
      LightweightNode::destroy(path.front());
    }
    connection->sink()->set_routed(true);
  }
  else
  {
    LOG4CPLUS_ERROR(logger, "Route direct connection failed: "
                                << source->id() << " -> " << sink->id());
  }
}

vector<LightweightNodePtr> AceRoute::find_path_between_nodes(NodePtr source,
                                                             NodePtr sink)
{
  vector<LightweightNodePtr> path;
  if (source->id() == sink->id())
  {
    path.push_back(LightweightNode::create(sink->dev_node()));
    return path; // for pins without additional projected int_node
  }
  for (auto downhill : source->dev_node()->get_downhill_nodes())
  {
    if (downhill->id() == (uint32_t)sink->id())
    {
      path.push_back(LightweightNode::create(sink->dev_node()));
      path.push_back(LightweightNode::create(source->dev_node()));
      return path;
    }
  }
  LightweightNodePtr sourcer = LightweightNode::create(source->dev_node());
  sourcer->set_prev(nullptr);
  std::queue<LightweightNodePtr> q;
  q.push(sourcer);

  int watchdog = 10000;
  bool success = false;
  while (!q.empty() && watchdog >= 0)
  {
    LightweightNodePtr curr = q.front();
    q.pop();
    if (curr->id() == sink->id())
    {
      while (curr != nullptr)
      {
        path.push_back(LightweightNode::create(curr->dev_node()));
        curr = curr->get_prev();
      }
      success = true;
      break;
    }
    for (auto n : curr->dev_node()->get_downhill_nodes())
    {
      LightweightNodePtr child = LightweightNode::create(n);
      child->set_prev(curr);
      q.push(child);
    }
    watchdog--;
  }

  if (!success)
  {
    LOG4CPLUS_ERROR(logger, "Failed to find a path between two nodes: "
                                << source->id() << " -> " << sink->id());
    const static vector<LightweightNodePtr> null_path;
    return null_path;
  }
  return path;
}

int AceRoute::get_new_str_id(string str)
{
  if (!str_to_id.contains(str))
  {
    int new_id = str_to_id.size();
    str_to_id[str] = new_id;
  }
  if (str_to_id[str] == 0)
  {
  }
  return str_to_id[str];
}

void AceRoute::complete_indirect_connections()
{
  for (ConnectionPtr connection : connections)
  {
    if (connection->is_direct())
      continue;
    auto reversed_source_to_source_INT =
        find_path_between_nodes(connection->source(), connection->source_INT());
    auto reversed_sink_INT_to_sink =
        find_path_between_nodes(connection->sink_INT(), connection->sink());
    vector<NodePtr> reversed_source_INT_to_sink_INT =
        connection->get_rnodes(); // copy
    connection->clear_rnode();
    for (auto it = reversed_sink_INT_to_sink.begin();
         it < reversed_sink_INT_to_sink.end() - 1; ++it)
    {
      // sink to sink INT (excluded)
      auto node_type = NodeType::PINFEED_I; // HACK
      auto node = rg.get_or_create((*it)->id(), node_type);
      node->increment_user(connection->net()->id());
      connection->add_rnode(node);
    }
    for (auto node : reversed_source_INT_to_sink_INT)
    {
      // sink INT to source INT
      connection->add_rnode(node);
    }
    for (auto it = reversed_source_to_source_INT.begin() + 1;
         it < reversed_source_to_source_INT.end(); ++it)
    {
      // source INT (excluded) to source
      auto node_type = NodeType::PINFEED_O; // HACK
      auto node = rg.get_or_create((*it)->id(), node_type);
      node->increment_user(connection->net()->id());
      connection->add_rnode(node);
    }
    connection->sink()->set_routed(true);
  }
}

void AceRoute::print_failure_stats()
{
  LOG4CPLUS_ERROR(logger, "ERROR: Routing terminated after "
                              << config.max_iterations << " iterations.");
  LOG4CPLUS_ERROR(logger, "       Unroutable connections: "
                              << get_unroutable_connections().size());
  LOG4CPLUS_ERROR(logger,
                  "       Conflicting nodes: " << overused_rnodes.size());
  for (NodePtr conflict_node : overused_rnodes)
  {
    LOG4CPLUS_DEBUG(logger,
                    "Conflict node : "
                        << rg.get_base_tile_name(conflict_node) << " "
                        << rg.get_base_wire_name(conflict_node) << " "
                        << "Conflict net = " << conflict_node->get_occupancy());
    for (ConnectionPtr connection : connections)
    {
      if (connection->is_congested())
      {
        for (NodePtr rnode : connection->get_rnodes())
        {
          if (rnode == conflict_node)
          {
            LOG4CPLUS_DEBUG(
                logger,
                "Conflict conn : source : "
                    << rg.get_base_tile_name(connection->source()) << " "
                    << rg.get_base_wire_name(connection->source())
                    << " sink : " << rg.get_base_tile_name(connection->sink())
                    << " " << rg.get_base_wire_name(connection->sink()));
          }
        }
      }
    }
  }
}

bool AceRoute::proc_congested_connections()
{
  vector<ConnectionPtr> unroutable_connections = get_unroutable_connections();
  vector<ConnectionPtr> congested_connections = get_congested_connections();

  LOG4CPLUS_INFO(logger,
                 "Congested connections: " << congested_connections.size());

  // HACKed: a decision tree crafted by "Kevin7Zz" <1300549484@qq.com>
  if (route_iteration == 1 && (int)connections.size() > 800000 &&
      congested_connections.size() > 0)
  {
    int bbox_x_min = std::numeric_limits<int>::max();
    int bbox_x_max = std::numeric_limits<int>::min();
    int bbox_y_min = std::numeric_limits<int>::max();
    int bbox_y_max = std::numeric_limits<int>::min();
    for (auto connection : congested_connections)
    {
      bbox_x_min = std::min(bbox_x_min, connection->get_xmin_bb());
      bbox_x_max = std::max(bbox_x_max, connection->get_xmax_bb());
      bbox_y_min = std::min(bbox_y_min, connection->get_ymin_bb());
      bbox_y_max = std::max(bbox_y_max, connection->get_ymax_bb());
    }
    int area = (bbox_x_max - bbox_x_min) * (bbox_y_max - bbox_y_min);
    int density = (int)connections.size() / area;
    if (area < 30000 && density > 35)
    {
      config.enlarge_bounding_box = true;
    }
  }

  for (auto connection : unroutable_connections)
  {
    if (route_iteration == 1)
    {
      if (!connection->swap_source())
      {
        LOG4CPLUS_ERROR(logger,
                        "Net " << connection->net()->id() << " is unroutable");
      }
    }
    if (config.enlarge_bounding_box)
    {
      connection->enlarge_bounding_box(config.extension_x_increment,
                                       config.extension_y_increment);
    }
  }

  if (config.enlarge_bounding_box)
  {
    for (auto connection : congested_connections)
    {
      connection->enlarge_bounding_box(config.extension_x_increment,
                                       config.extension_y_increment);
    }
  }

  if (overused_rnodes.empty())
  {
    if (unroutable_connections.empty())
    {
      LOG4CPLUS_INFO(logger, "");
      LOG4CPLUS_INFO(logger, "Route indirect connections success");
      LOG4CPLUS_INFO(logger, "------------------------------");
      LOG4CPLUS_INFO_FMT(logger, "%-20s%10d",
                         "Num iterations:", route_iteration);
      LOG4CPLUS_INFO_FMT(logger, "%-20s%10d",
                         "Connections routed:", connections_routed);
      LOG4CPLUS_INFO_FMT(logger, "%-20s%10lld", "Nodes pushed:", nodes_pushed);
      LOG4CPLUS_INFO_FMT(logger, "%-20s%10lld", "Nodes popped:", nodes_popped);
      LOG4CPLUS_INFO(logger, "------------------------------");
      return true;
    }
    else
    {
      if (route_iteration == config.max_iterations)
      {
        LOG4CPLUS_ERROR(logger, "Unroutable connections: "
                                    << unroutable_connections.size());
      }
    }
  }
  return false;
}

void AceRoute::path_finder()
{
  initialize_path_finder();
  sort_connections();
  auto last_time = steady_clock::now();
  auto route_begin_time = last_time;
  //   if (!connections.empty()) {
  //     connections.erase(connections.begin() + 100, connections.end());
  //     // connections.erase(connections.begin());
  //   }
  int total_conn = connections.size();

  for (route_iteration = 1; route_iteration <= config.max_iterations;
       route_iteration++)
  {
    auto nodes_pushed_last_iteration = nodes_pushed;
    auto nodes_popped_last_iteration = nodes_popped;
    children_skipped = 0;
    if (route_iteration == 1)
    {
      LOG4CPLUS_INFO(logger, "");
    }
    LOG4CPLUS_INFO(logger, "============ Iteration " << route_iteration
                                                     << " ============");
    connections_routed_this_iteration = 0;

    //////////////////////////////////////////////
    // You can focus on parallelizing this part //
    //////////////////////////////////////////////
    for (ConnectionPtr connection : connections)
    {
      if (should_route(connection))
      {
        ++connections_routed_this_iteration;

        if (config.bidirectional_par)
          route_connection_pba(connection, connections_routed);
        else
          route_connection(connection, connections_routed);
      }
    }

    update_cost();

    long overUsed = overused_rnodes.size();
    LOG4CPLUS_INFO(logger,
                   "Routed connections: " << connections_routed_this_iteration
                                          << "/" << total_conn);
    LOG4CPLUS_INFO(logger, "Nodes with overlaps: " << overUsed);
    // LOG4CPLUS_INFO(logger,
    //                "Generated Nodes: " << rnodes_created_this_iteration);
    auto nodes_pushed_this_iteration =
        nodes_pushed - nodes_pushed_last_iteration;
    auto nodes_popped_this_iteration =
        nodes_popped - nodes_popped_last_iteration;
    LOG4CPLUS_INFO_FMT(logger, "%s%lld (%.2f/conn)",
                       "Nodes pushed: ", nodes_pushed_this_iteration,
                       double(nodes_pushed_this_iteration) /
                           connections_routed_this_iteration);
    LOG4CPLUS_INFO_FMT(logger, "%s%lld (%.2f/conn)",
                       "Nodes popped: ", nodes_popped_this_iteration,
                       double(nodes_popped_this_iteration) /
                           connections_routed_this_iteration);
    LOG4CPLUS_INFO_FMT(
        logger, "%s%lld (%.2f/conn)", "Children skipped: ", children_skipped,
        double(children_skipped) / connections_routed_this_iteration);

    bool route_indrect_conn_success = proc_congested_connections();
    if (route_indrect_conn_success)
      break;

    LOG4CPLUS_INFO(logger,
                   "Time (s): " << elapsed_seconds(last_time) << " | RAM (GB): "
                                << getMemUsage() / 1024.0 / 1024.0 / 1024.0);

    last_time = steady_clock::now();
    // break;
  }

  complete_indirect_connections();

  // It is not used because we have simplified all the direct connections in our homework
  route_direct_connections();

  fix_routes();

  if (route_iteration == config.max_iterations + 1)
  {
    print_failure_stats();
  }
  LOG4CPLUS_INFO(logger,
                 "Route time (s): " << elapsed_seconds(route_begin_time));

  prepare_write();
  write();
}

/* My Parallel New Bidirectional A* path finder */
bool AceRoute::route_connection_pba(ConnectionPtr connection,
                                    int &connections_routed_local)
{
  rip_up(connection);
  connections_routed_local++;
  bool finished = false;
  decltype(nodes_pushed) nodes_pushed_this_connection = 0;
  decltype(nodes_popped) nodes_popped_this_connection = 0;
  NodePtr solution = nullptr;
  double F1 = 0;
  double F2 = 0;
  double L = std::numeric_limits<double>::max();
  omp_lock_t l_par;
  omp_init_lock(&l_par);
  auto begin_time = steady_clock::now();

#pragma omp parallel num_threads(2) shared(finished, solution, F1, F2, L, l_par, connection, connections_routed_local, nodes_pushed_this_connection, nodes_popped_this_connection) //, children_skipped)//,nodes_pushed_this_connection_backward, nodes_popped_this_connection_backward)
  {
#pragma omp sections
    {
#pragma omp section
      {
        // forward search
        std::priority_queue<NodePtr, std::vector<NodePtr>, CompareNodePtr> min_heap_s;
        assert(connection->source_INT()->get_prev() == nullptr);
        connection->source_INT()->set_visited_forward(connections_routed_local);
        connection->source_INT()->set_upstream_path_cost(0);
        connection->source_INT()->set_lower_bound_total_path_cost(0);
        min_heap_s.push(connection->source_INT());
#pragma omp atomic
        nodes_pushed_this_connection++;

        NodePtr u;
        while (!finished && !min_heap_s.empty())
        {
          u = min_heap_s.top();
          min_heap_s.pop();
#pragma omp atomic
          nodes_popped_this_connection++;
          if (!u->is_visited(connections_routed_local))
          {
            if ((u->get_lower_bound_total_path_cost() < L) &&
                (u->get_upstream_path_cost() + F2 - compute_backward_future_cost(connection, u) < L))
            {

              for (NodePtr v : u->get_children())
              {
                if (v->is_visited(connections_routed_local) or
                    (!is_accessible(v, connection)) or
                    can_skip_by_node_type(v, connection))
                  continue;

                evaluate_forward_cost(connection, u, v);
                v->set_visited_forward(connections_routed_local);
                min_heap_s.push(v);
#pragma omp atomic
                nodes_pushed_this_connection++;

                if (v->get_visited_backward() == connections_routed_local)
                {
                  if (v->get_upstream_path_cost() + v->get_upstream_path_cost_back() < L)
                  {
                    omp_set_lock(&l_par);
                    if (v->get_upstream_path_cost() + v->get_upstream_path_cost_back() < L)
                    {
                      solution = v;
                      L = v->get_upstream_path_cost() + v->get_upstream_path_cost_back();
                    }
                    omp_unset_lock(&l_par);
                  }
                }
              }
            }
            u->set_visited(connections_routed_local);
            // children_skipped += u->children_skipped;
          }
          if (!min_heap_s.empty())
          {
#pragma omp atomic write
            F1 = min_heap_s.top()->get_lower_bound_total_path_cost();
          }
          else
          {
#pragma omp atomic write
            finished = true;
          }
        }
      }

#pragma omp section
      {
        // backward search
        std::priority_queue<NodePtr, std::vector<NodePtr>, CompareBackNodePtr> min_heap_t;
        assert(connection->sink_INT()->get_next() == nullptr);
        connection->sink_INT()->set_visited_backward(connections_routed_local);
        connection->sink_INT()->set_upstream_path_cost_back(0);
        connection->sink_INT()->set_lower_bound_total_path_cost_back(0);
        min_heap_t.push(connection->sink_INT());
#pragma omp atomic
        nodes_pushed_this_connection++;

        NodePtr u;
        while (!finished && !min_heap_t.empty())
        {
          u = min_heap_t.top();
          min_heap_t.pop();
#pragma omp atomic
          nodes_popped_this_connection++;
          if (!u->is_visited(connections_routed_local))
          {
            if ((u->get_lower_bound_total_path_cost_back() < L) &&
                (u->get_upstream_path_cost_back() + F1 - compute_forward_future_cost(connection, u) < L))
            {
              for (NodePtr v : u->get_parents())
              {
                if (v->is_visited(connections_routed_local) or
                    (!is_accessible(v, connection)) or
                    can_skip_by_node_type(v, connection))
                  continue;
                evaluate_backward_cost(connection, u, v);
                v->set_visited_backward(connections_routed_local);
                min_heap_t.push(v);
#pragma omp atomic
                nodes_pushed_this_connection++;

                if (v->get_visited_forward() == connections_routed_local)
                {
                  if (v->get_upstream_path_cost() + v->get_upstream_path_cost_back() < L)
                  {
                    omp_set_lock(&l_par);
                    if (v->get_upstream_path_cost() + v->get_upstream_path_cost_back() < L)
                    {
                      solution = v;
                      L = v->get_upstream_path_cost() + v->get_upstream_path_cost_back();
                    }
                    omp_unset_lock(&l_par);
                  }
                }
              }
            }
            u->set_visited(connections_routed_local);
            // children_skipped += u->children_skipped;
          }
          if (!min_heap_t.empty())
          {
#pragma omp atomic write
            F2 = min_heap_t.top()->get_lower_bound_total_path_cost_back();
          }
          else
          {
#pragma omp atomic write
            finished = true;
          }
        }
      }
    }
  }

  nodes_pushed += nodes_pushed_this_connection;
  nodes_popped += nodes_popped_this_connection;
  if (solution != nullptr)
  {
    finish_route_connection_pba(connection, solution);
    assert(connection->sink_INT()->is_routed());
  }
  else
  {
    connection->reset_route();
    assert(!connection->sink_INT()->is_routed());
  }
  connection->set_nodes_popped_last_iter(nodes_popped_this_connection);

  auto connection_route_time = elapsed_milliseconds(begin_time);
  connection->set_route_time_last_iter(connection_route_time);

  if (config.connections_log.size() > 0)
  {
    connections_log << route_iteration << ", " << connection->get_id_indirect()
                    << ", (" << connection->get_xmin_bb() << " "
                    << connection->get_ymin_bb() << " "
                    << connection->get_xmax_bb() << " "
                    << connection->get_ymax_bb() << "), " << connection->hpwl()
                    << ", " << connection->get_rnodes().size() << ", "
                    << connection->sink_INT()->get_upstream_path_cost() << ", "
                    << nodes_pushed_this_connection << "/"
                    << nodes_popped_this_connection << ", "
                    << connection_route_time << ", "
                    << (solution != nullptr ? "success" : "fail") << std::endl;
  }
  return solution != nullptr;
}

// update g1 & f1
inline void AceRoute::evaluate_forward_cost(ConnectionPtr connection, NodePtr rnode, NodePtr child_rnode)
{
  // child_rnode->set_prev(rnode);
  // double d1xy = compute_forward_dis(connection, child_rnode);
  // double new_partial_path_cost =
  //     rnode->get_upstream_path_cost() +
  //     d1xy;

  // double h1y = compute_forward_future_cost(connection, child_rnode);
  // double new_total_path_cost =
  //     new_partial_path_cost +
  //     h1y;

  // child_rnode->set_upstream_path_cost(new_partial_path_cost);
  // child_rnode->set_lower_bound_total_path_cost(new_total_path_cost);
  int count_source_uses =
      child_rnode->count_connections_of_user(connection->net()->id());
  int sharing_factor = 1 + count_source_uses;
  child_rnode->set_prev(rnode);
  double new_partial_path_cost =
      rnode->get_upstream_path_cost() +
      get_node_cost(child_rnode, connection, count_source_uses,
                    sharing_factor) +
      (1 - config.wirelength_weight) * child_rnode->length() / sharing_factor;

  int child_x = child_rnode->end_x();
  int child_y = child_rnode->end_y();
  NodePtr sink = connection->sink_INT();
  int sink_x = sink->begin_x();
  int sink_y = sink->begin_y();
  int delta_x = abs(child_x - sink_x);
  int delta_y = abs(child_y - sink_y);
  int distance_to_sink = delta_x + delta_y;
  double new_total_path_cost =
      new_partial_path_cost +
      config.wirelength_weight * distance_to_sink / sharing_factor;

  child_rnode->set_upstream_path_cost(new_partial_path_cost);
  child_rnode->set_lower_bound_total_path_cost(new_total_path_cost);
}
// update g2 & f2
inline void AceRoute::evaluate_backward_cost(ConnectionPtr connection, NodePtr rnode, NodePtr parent_rnode)
{
  // parent_rnode->set_next(rnode);
  // double d2xy = compute_backward_dis(connection, parent_rnode);
  // double new_partial_path_cost =
  //     rnode->get_upstream_path_cost_back() +
  //     d2xy;

  // double h2y = compute_backward_future_cost(connection, parent_rnode);
  // double new_total_path_cost =
  //     new_partial_path_cost +
  //     h2y;

  // parent_rnode->set_upstream_path_cost_back(new_partial_path_cost);
  // parent_rnode->set_lower_bound_total_path_cost_back(new_total_path_cost);
  int count_source_uses =
      parent_rnode->count_connections_of_user(connection->net()->id());
  int sharing_factor = 1 + count_source_uses;
  parent_rnode->set_next(rnode);
  double new_partial_path_cost =
      rnode->get_upstream_path_cost_back() +
      get_node_cost(parent_rnode, connection, count_source_uses,
                    sharing_factor) +
      (1 - config.wirelength_weight) * parent_rnode->length() / sharing_factor;

  int parent_x = parent_rnode->end_x();
  int parent_y = parent_rnode->end_y();
  NodePtr source = connection->source_INT();
  int source_x = source->begin_x();
  int source_y = source->begin_y();
  int delta_x = abs(parent_x - source_x);
  int delta_y = abs(parent_y - source_y);
  int distance_to_sink = delta_x + delta_y;
  double new_total_path_cost =
      new_partial_path_cost +
      config.wirelength_weight * distance_to_sink / sharing_factor;

  parent_rnode->set_upstream_path_cost_back(new_partial_path_cost);
  parent_rnode->set_lower_bound_total_path_cost_back(new_total_path_cost);
}

// compute h1
inline double AceRoute::compute_forward_future_cost(ConnectionPtr connection, NodePtr child_rnode,
                                                    int count_users)
{
  int count_source_uses =
      child_rnode->count_connections_of_user(connection->net()->id());
  int sharing_factor = 1 + count_source_uses;
  int child_x = child_rnode->end_x();
  int child_y = child_rnode->end_y();
  NodePtr sink = connection->sink_INT();
  int sink_x = sink->begin_x();
  int sink_y = sink->begin_y();
  int delta_x = abs(child_x - sink_x);
  int delta_y = abs(child_y - sink_y);
  int distance_to_sink = delta_x + delta_y;
  return config.wirelength_weight * distance_to_sink / sharing_factor;
}

// compute h2
inline double AceRoute::compute_backward_future_cost(ConnectionPtr connection, NodePtr parent_rnode,
                                                     int count_users)
{
  int count_source_uses =
      parent_rnode->count_connections_of_user(connection->net()->id());
  int sharing_factor = 1 + count_source_uses;
  int parent_x = parent_rnode->end_x();
  int parent_y = parent_rnode->end_y();
  NodePtr source = connection->source_INT();
  int source_x = source->begin_x();
  int source_y = source->begin_y();
  int delta_x = abs(parent_x - source_x);
  int delta_y = abs(parent_y - source_y);
  int distance_to_source = delta_x + delta_y;
  return config.wirelength_weight * distance_to_source / sharing_factor;
}

// compute d1
inline double AceRoute::compute_forward_dis(ConnectionPtr connection, NodePtr v)
{
  int count_source_uses =
      v->count_connections_of_user(connection->net()->id());
  int sharing_factor = 1 + count_source_uses;
  return get_node_cost(v, connection, count_source_uses,
                       sharing_factor) +
         (1 - config.wirelength_weight) * v->length() / sharing_factor;
}

// compute d2
inline double AceRoute::compute_backward_dis(ConnectionPtr connection, NodePtr v)
{
  int count_source_uses =
      v->count_connections_of_user(connection->net()->id());
  int sharing_factor = 1 + count_source_uses;
  return get_node_cost(v, connection, count_source_uses,
                       sharing_factor) +
         (1 - config.wirelength_weight) * v->length() / sharing_factor;
}
inline bool AceRoute::save_routing_pba(ConnectionPtr connection, NodePtr rnode)
{

  assert(rnode->get_prev() != nullptr);
  assert(rnode->get_next() != nullptr);

  connection->reset_route();

  std::vector<NodePtr> path;
  NodePtr node_tmp = rnode;
  while (node_tmp != nullptr)
  {
    path.push_back(node_tmp);
    node_tmp = node_tmp->get_next();
  }
  assert(path.back() == connection->sink_INT());
  for (auto i = path.rbegin(); i != path.rend(); ++i)
    connection->add_rnode(*i);

  node_tmp = rnode->get_prev();
  while (node_tmp != nullptr)
  {
    connection->add_rnode(node_tmp);
    node_tmp = node_tmp->get_prev();
  }

  const auto &rnodes = connection->get_rnodes();
  if (rnodes.size() == 1)
  {
    // No prev pointer from sink rnode -> not routed
    return false;
  }
  assert(rnodes.back() == connection->source_INT());
  return true;
}
inline void AceRoute::finish_route_connection_pba(ConnectionPtr connection, NodePtr rnode)
{
  bool routed = save_routing_pba(connection, rnode);
  if (routed)
  {
    connection->sink_INT()->set_routed(routed);
    update_users_and_present_congestion_cost(connection);
  }
  else
  {
    connection->reset_route();
  }
}