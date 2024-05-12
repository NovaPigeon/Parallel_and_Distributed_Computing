#ifndef __SSP_H__
#define __SSP_H__

//#define DEBUG

#include <vector>
#include "common/graph.h"

struct solution
{
  std::vector<int> distances;
};


void dijkstra_serial(Graph graph, solution* sol);

void dijkstra_parallel(Graph graph, solution* sol);

void bellmanford_serial(Graph graph, solution* sol);

void bellmanford_parallel(Graph graph, solution* sol);


#endif
