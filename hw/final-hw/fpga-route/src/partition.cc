#include "ace-route.h"
#include <cassert>
#include <numeric>
#include <sstream>
#include <vector>

#include <log4cplus/logger.h>
#include <log4cplus/loggingmacros.h>

static log4cplus::Logger logger = log4cplus::Logger::getInstance("ace.route");

int index_offset[10] = {0, 3, 12, 39, 120, 363};

std::vector<std::vector<int>>
AceRoute::connections_partition_local(const std::vector<int> &connections_id,
                                      bool is_vertical) {
  if (connections_id.empty())
    return std::vector<std::vector<int>>{{}, {}, {}};
  int coord_max = std::numeric_limits<int>::min();
  int coord_min = std::numeric_limits<int>::max();
  // WARNING: the bound may need reconsideration
  for (auto &connection : connections_id) {
    int coord_max_local = is_vertical ? connections[connection]->get_xmax_bb()
                                      : connections[connection]->get_ymax_bb();
    int coord_min_local = is_vertical ? connections[connection]->get_xmin_bb()
                                      : connections[connection]->get_ymin_bb();
    assert(coord_max_local >= coord_min_local);
    coord_max = std::max(coord_max, coord_max_local);
    coord_min = std::min(coord_min, coord_min_local);
    // std::cout<<_max<<" "<<_min<<"\n";
  }
  int cut_line = coord_min + (coord_max - coord_min) / 2;
  // std::cout<<min<<" "<<max<<" "<<cut_line<<"\n";
  std::vector<std::vector<int>> result(3, std::vector<int>{});
  for (auto &connection : connections_id) {
    int coord_max_local = is_vertical ? connections[connection]->get_xmax_bb()
                                      : connections[connection]->get_ymax_bb();
    int coord_min_local = is_vertical ? connections[connection]->get_xmin_bb()
                                      : connections[connection]->get_ymin_bb();
    if (coord_max_local < cut_line)
      result[1].emplace_back(connection);
    else if (coord_min_local > cut_line)
      result[2].emplace_back(connection);
    else
      result[0].emplace_back(connection);
  }
  // std::cout<<result[0].size()<<" "<<result[1].size()<<"
  // "<<result[2].size()<<"\n";
  return result;
}

std::vector<std::vector<int>>
AceRoute::connections_partition(int partition_time) {
  assert(partition_time < 6);
  std::vector<std::vector<int>> result;
  bool vertical = false;
  // std::cout<<connections.size()<<"\n";
  std::vector<int> connections_id(connections.size());

  std::iota(connections_id.begin(), connections_id.end(), 0);

  auto temp_result = connections_partition_local(connections_id, vertical);
  vertical = !vertical;
  result.insert(result.end(), temp_result.begin(), temp_result.end());

  for (int i = 1; i < partition_time; i++) {
    for (int j = index_offset[i - 1]; j < index_offset[i]; j++) {
      auto &sub_vector = result[j];
      // std::cout<<"sub_vector "<<j<<" size: "<<sub_vector.size()<<" partition
      // time: "<<i<<"\n";
      auto temp_result = connections_partition_local(sub_vector, vertical);
      vertical = !vertical;
      result.insert(result.end(), temp_result.begin(), temp_result.end());
    }
  }
  std::stringstream ss;
  for (auto &entry : result) {
    ss << entry.size() << " ";
    // for(auto& element: entry) std::cout<<element<<" ";
    // std::cout<<"\n\n\n\n";
  }
  LOG4CPLUS_DEBUG(logger, ss.str().c_str());
  // std::cout << "\n";

  return result;
}
