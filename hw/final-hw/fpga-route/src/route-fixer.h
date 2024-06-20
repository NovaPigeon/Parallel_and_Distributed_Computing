#ifndef ROUTE_FIXER_H
#define ROUTE_FIXER_H

#include "config.h"
#include "node.h"
#include <set>
#include <unordered_map>

DECLARE_PTR(RouteFixer)
DECLARE_PTR(Net)
DECLARE_PTR(Connection)

class RouteFixer {
public:
  PTR_FACTORY(RouteFixer)
  PTR_RECYCLE(RouteFixer)

  RouteFixer(NetPtr net) : net(net) { build_graph(); }

  void fix_routes();

private:
  DECLARE_PTR(FixNode)

  class FixNode {
  public:
    PTR_FACTORY(FixNode)
    PTR_RECYCLE(FixNode)

    FixNode(NodePtr node);

    void add_children(FixNodePtr v) { children_.insert(v); }

    ACCESSOR_RO(NodePtr, node)
    ACCESSOR_RO(double, node_cost);
    ACCESSOR_RW(double, path_cost);
    ACCESSOR_RO(std::set<FixNodePtr>, children)
    ACCESSOR_RW_BOOL(sink);
    ACCESSOR_RW(FixNodePtr, prev);
    ACCESSOR_RW_BOOL(visited);

  private:
  };

  class FixNodePtrComparator {
  public:
    bool operator()(FixNodePtr a, FixNodePtr b) const {
      return a->get_path_cost() > b->get_path_cost();
    }
  };

  NetPtr net;
  std::unordered_map<NodePtr, FixNodePtr> node_map;
  std::set<FixNodePtr> sources;

  void build_graph();

  FixNodePtr get_or_create_fix_node(NodePtr node);

  void build_shortest_path_tree();
};

#endif // ROUTE_FIXER_H