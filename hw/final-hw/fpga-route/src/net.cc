#include "net.h"
#include "connection.h"
#include <limits>

using namespace std;

void Net::compute() {
  int xmin = std::numeric_limits<int>::max();
  int ymin = std::numeric_limits<int>::max();
  int xmax = std::numeric_limits<int>::min();
  int ymax = std::numeric_limits<int>::min();
  int xsum = 0;
  int ysum = 0;
  int count = 0;
  bool source_rnode_added = false;
  for (ConnectionPtr connection : connections()) {
    if (connection->is_direct())
      continue;
    if (!source_rnode_added) {
      source_rnode_added = true;
      int x = connection->source_INT()->end_x();
      int y = connection->source_INT()->end_y();
      xmin = min(xmin, x);
      xmax = max(xmax, x);
      ymin = min(ymin, y);
      ymax = max(ymax, y);
      xsum += x;
      ysum += y;
      count++;
    }
    int x = connection->sink_INT()->end_x();
    int y = connection->sink_INT()->end_y();
    xmin = min(xmin, x);
    xmax = max(xmax, x);
    ymin = min(ymin, y);
    ymax = max(ymax, y);
    xsum += x;
    ysum += y;
    count++;
  }
  double_hpwl_ = ((xmax - xmin + 1) + (ymax - ymin + 1)) * 2;
  x_center_ = (double)xsum / count;
  y_center_ = (double)ysum / count;
}
