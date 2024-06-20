#include "route-fixer.h"
#include "connection.h"
#include "net.h"
#include <queue>
#include <vector>
using namespace std;

void RouteFixer::build_graph() {
  for (auto connection : net->connections()) {
    auto rnodes = connection->get_rnodes();
    FixNodePtr prev = nullptr;
    for (int i = (int)rnodes.size() - 1; i >= 0; i--) {
      FixNodePtr cur = get_or_create_fix_node(rnodes[i]);
      if (prev == nullptr) {
        sources.insert(cur);
        prev = cur;
        continue;
      }
      if (i == 0) {
        cur->set_sink();
      }
      prev->add_children(cur);
      prev = cur;
    }
  }
}

RouteFixer::FixNode::FixNode(NodePtr node)
    : node_(node), node_cost_(node->base_cost()), path_cost_(0x3f3f3f3f),
      prev_(nullptr), is_visited_(false) {}

RouteFixer::FixNodePtr RouteFixer::get_or_create_fix_node(NodePtr node) {
  if (!node_map.contains(node)) {
    node_map[node] = FixNode::create(node);
  }
  return node_map[node];
}

void RouteFixer::build_shortest_path_tree() {
  priority_queue<FixNodePtr, vector<FixNodePtr>, FixNodePtrComparator> q;
  for (auto node : sources) {
    node->set_path_cost(node->node_cost());
    node->set_prev(nullptr);
    q.push(node);
  }

  while (!q.empty()) {
    FixNodePtr u = q.top();
    q.pop();
    u->set_visited();
    for (auto v : u->children()) {
      double new_cost = u->get_path_cost() + v->node_cost();
      if (v->is_visited()) {
        continue;
      }
      if (new_cost < v->get_path_cost()) {
        v->set_path_cost(new_cost);
        v->set_prev(u);
        q.push(v);
      }
    }
  }
}

void RouteFixer::fix_routes() {
  build_shortest_path_tree();
  for (auto connection : net->connections()) {
    if (connection->get_rnodes().empty())
      continue;
    FixNodePtr u = get_or_create_fix_node(connection->get_rnodes().front());
    connection->clear_rnode();
    while (u != nullptr) {
      connection->add_rnode(u->node());
      u = u->get_prev();
    }
  }
}