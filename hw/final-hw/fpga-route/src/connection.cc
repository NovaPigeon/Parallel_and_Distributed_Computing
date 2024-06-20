#include "connection.h"
#include "net.h"
#include <cmath>

using namespace std;

bool Connection::is_congested() {
  for (NodePtr node : rnodes) {
    if (node->is_overused()) {
      return true;
    }
  }
  return false;
}

void Connection::compute_bounding_box(int extension_x, int extension_y) {
  int net_x_center = ceil(net()->x_center());
  int net_y_center = ceil(net()->y_center());
  int xmax = max(net_x_center, max(source_INT()->end_x(), sink_INT()->end_x()));
  int xmin = min(net_x_center, min(source_INT()->end_x(), sink_INT()->end_x()));
  int ymax = max(net_y_center, max(source_INT()->end_y(), sink_INT()->end_y()));
  int ymin = min(net_y_center, min(source_INT()->end_y(), sink_INT()->end_y()));
  xmax_bb_ = (xmax + extension_x);
  xmin_bb_ = (max(xmin - extension_x, -1));
  ymax_bb_ = (ymax + extension_y);
  ymin_bb_ = (max(ymin - extension_y, -1));
}

void Connection::enlarge_bounding_box(int enlarge_x, int enlarge_y) {
  xmin_bb_ = max(-1, xmin_bb_ - enlarge_x);
  ymin_bb_ = max(-1, ymin_bb_ - enlarge_y);
  xmax_bb_ += enlarge_x;
  ymax_bb_ += enlarge_y;
}

void Connection::add_rnode(NodePtr rnode) {
  xmin_bb_ = min(xmin_bb_, rnode->begin_x() - 1);
  xmax_bb_ = max(xmax_bb_, rnode->end_x() + 1);
  ymin_bb_ = min(ymin_bb_, rnode->begin_y() - 1);
  ymax_bb_ = max(ymax_bb_, rnode->end_y() + 1);
  rnodes.push_back(rnode);
}