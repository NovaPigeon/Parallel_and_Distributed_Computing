#ifndef CONNECTION_H
#define CONNECTION_H

#include "config.h"
#include "routing-graph.h"
#include <memory>
#include <vector>

DECLARE_PTR(Connection)
DECLARE_PTR(Net)

class Connection {
public:
  PTR_FACTORY(Connection)
  PTR_RECYCLE(Connection)

  Connection(NetPtr net_, NodePtr source_, NodePtr sink_,
             NodePtr alt_source_ = nullptr, NodePtr source_INT_ = nullptr,
             NodePtr sink_INT_ = nullptr, NodePtr alt_source_INT_ = nullptr)
      : net_(net_), source_(source_), sink_(sink_), alt_source_(alt_source_),
        source_INT_(source_INT_), sink_INT_(sink_INT_),
        alt_source_INT_(alt_source_INT_) {
    hpwl_ = 1 + 1;
    xmin_bb_ = std::numeric_limits<int>::min();
    xmax_bb_ = std::numeric_limits<int>::max();
    ymin_bb_ = std::numeric_limits<int>::min();
    ymax_bb_ = std::numeric_limits<int>::max();
    is_direct_ = false;
    is_use_ba_ = false;
    nodes_popped_last_iter_ = 0;
    route_time_last_iter_ = 0;
  }

  bool swap_source() {
    if (alt_source_ != nullptr) {
      std::swap(source_, alt_source_);
      std::swap(source_INT_, alt_source_INT_);
      return true;
    } else {
      return false;
    }
  }

  bool is_congested();

  void reset_route() {
    rnodes.clear();
    // sink->set_routed(false);
  }

  void add_rnode(NodePtr rnode);

  void clear_rnode() { rnodes.clear(); }

  const std::vector<NodePtr> &get_rnodes() { return rnodes; }

  void compute_bounding_box(int extension_x, int extension_y);

  void enlarge_bounding_box(int enlarge_x, int enlarge_y);

  bool use_rnodes_with_multiple_drivers() {
    for (auto rnode : rnodes) {
      if (rnode->have_multiple_drivers()) {
        return true;
      }
    }
    return false;
  }

  ACCESSOR_RO(NetPtr, net)
  ACCESSOR_RO(NodePtr, source)
  ACCESSOR_RO(NodePtr, sink)
  ACCESSOR_RO(NodePtr, alt_source)
  ACCESSOR_RO(NodePtr, source_INT)
  ACCESSOR_RO(NodePtr, sink_INT)
  ACCESSOR_RO(NodePtr, alt_source_INT)
  ACCESSOR_RO(int, hpwl)

  ACCESSOR_RW(int, xmin_bb)
  ACCESSOR_RW(int, xmax_bb)
  ACCESSOR_RW(int, ymin_bb)
  ACCESSOR_RW(int, ymax_bb)
  ACCESSOR_RW(int, id_indirect)
  ACCESSOR_RW(unsigned long long, nodes_popped_last_iter)
  ACCESSOR_RW(double, route_time_last_iter)
  ACCESSOR_RW_BOOL(use_ba)
  ACCESSOR_RW_BOOL(direct)

  void compute_hpwl() {
    hpwl_ = std::abs(source_INT()->end_x() - sink_INT()->end_x()) + 1 +
            std::abs(source_INT()->end_y() - sink_INT()->end_y()) + 1;
  }

private:
  std::vector<NodePtr> rnodes;
};

#endif // CONNECTION_H