#include "sssp.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cstddef>
#include <omp.h>
#include <queue>
#include <limits>
#include <iostream>
#include <cmath>

#include "../common/CycleTimer.h"
#include "../common/graph.h"

#define ROOT_NODE_ID 0
#define NOT_VISITED_MARKER -1
#define BLOCK_NUM 128


// Implements serial Dijkstra Algorithm.
//
// Result of execution is that, for each node in the graph, the
// distance to the root is stored in sol.distances.

void dijkstra_serial(Graph graph, solution* sol) {
    double start = CycleTimer::currentSeconds();
    // Initialize the solution distances with infinity
    for(int i=0; i<graph->num_nodes; i++)
        sol->distances[i] = std::numeric_limits<int>::max();

    // The source node will have distance 0
    sol->distances[ROOT_NODE_ID] = 0;

    // Use a priority queue to store the nodes to be processed,
    // the shortest distance node will be processed first
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<std::pair<int, int>>> pq;

    // Start from the source node
    pq.push(std::make_pair(0, 0));
    
    while (!pq.empty()) {
        // Get the node with the shortest distance so far
        int u = pq.top().second;
        pq.pop();

        // Process all outgoing edges from the node u
        for (int i = graph->outgoing_starts[u]; i < (u == graph->num_nodes - 1 ? graph->num_edges : graph->outgoing_starts[u + 1]); i++) {
            int v = graph->outgoing_edges[i];
            int weight = graph->outgoing_edges_len[i];
            // If there is a shorter path to v through u
            if (sol->distances[v] > sol->distances[u] + weight) {
                // Update the distance of v
                sol->distances[v] = sol->distances[u] + weight;
                pq.push(std::make_pair(sol->distances[v], v));
            }
        }
    }
}


// Implements parallel Dijkstra Algorithm.
//
// Result of execution is that, for each node in the graph, the
// distance to the root is stored in sol.distances.
void dijkstra_parallel(Graph graph, solution* sol) {
    //print_graph(graph);
    double start = CycleTimer::currentSeconds();
    // Initialize the solution distances with infinity
    int n = num_nodes(graph);
    bool* visited = new bool[n];
    #pragma omp parallel for
    for (int i = 0; i < n; ++i) {
        sol->distances[i] = std::numeric_limits<int>::max();
        visited[i] = false;
    }
    sol->distances[ROOT_NODE_ID] = 0;
    //int BLOCK_NUM=omp_get_num_threads()+1;
    int blk_size=int(n/BLOCK_NUM);

    for (int i = 0; i < n; ++i) {
        // 找到距离源节点最近的未访问节点
        int min_distance = std::numeric_limits<int>::max();
        int u = -1;
        int mds[BLOCK_NUM];
        int us[BLOCK_NUM];
        #pragma omp parallel for
        for(int j=0;j<BLOCK_NUM;++j)
        {
            mds[j]=std::numeric_limits<int>::max();
            us[j]=-1;
            for (int k = j*blk_size; k < (j==BLOCK_NUM-1?n:(j+1)*blk_size); ++k) {
                if (!visited[k] && sol->distances[k] < mds[j]) {
                    mds[j] = sol->distances[k];
                    us[j] = k;
                }
            }
        }
        
        for(int j=0;j<BLOCK_NUM;++j)
        {
            if(mds[j]<min_distance)
            {
                min_distance=mds[j];
                u=us[j];
            }
        }

        // 如果没有找到可达的节点，直接退出循环
        if (u == -1)
            continue;

        visited[u] = true;

        // 更新与节点u相连的节点的距离
        #pragma omp parallel for
        for (const Vertex* it = outgoing_begin(graph, u); it != outgoing_end(graph, u); ++it) {
            int v = *it;
            int weight = graph->outgoing_edges_len[it - graph->outgoing_edges];
            if (sol->distances[u] != std::numeric_limits<int>::max() && sol->distances[u] + weight < sol->distances[v]) {
                sol->distances[v] = sol->distances[u] + weight;
            }
        }
    }

    delete[] visited;
    return;
    

}


// Implements serial Bellman-Ford Algorithm.
//
// Result of execution is that, for each node in the graph, the
// distance to the root is stored in sol.distances.
void bellmanford_serial(Graph graph, solution* sol) {

    // Initialize the solution distances with infinity
    for(int i=0; i<graph->num_nodes; i++)
        sol->distances[i] = std::numeric_limits<int>::max();

    // The source node will have distance 0
    sol->distances[ROOT_NODE_ID] = 0;

    // Relax all edges |V| - 1 times
    for (int i = 1; i <= graph->num_nodes-1; i++) {
        for (int j = 0; j < graph->num_nodes; j++) {
            for (int k = graph->outgoing_starts[j]; k < (j == graph->num_nodes - 1 ? graph->num_edges : graph->outgoing_starts[j + 1]); k++) {
                int u = j;
                int v = graph->outgoing_edges[k];
                int weight = graph->outgoing_edges_len[k];
                if (sol->distances[u] != std::numeric_limits<int>::max() && sol->distances[u] + weight < sol->distances[v])
                    sol->distances[v] = sol->distances[u] + weight;
            }
        }
    }

    // check for negative-weight cycles
    // for (int j = 0; j < g.num_nodes; j++) {
    //     for (int k = g.outgoing_starts[j]; k < (j == g.num_nodes - 1 ? g.num_edges : g.outgoing_starts[j + 1]); k++) {
    //         int u = j;
    //         int v = g.outgoing_edges[k];
    //         int weight = g.outgoing_edges_len[k];
    //         if (sol->distances[u] != INT_MAX && sol->distances[u] + weight < sol->distances[v]) {
    //             printf("Graph contains negative weight cycle");
    //             return;
    //         }
    //     }
    // }

}

// Implements parallel Bellman-Ford Algorithm.
//
// Result of execution is that, for each node in the graph, the
// distance to the root is stored in sol.distances.
void bellmanford_parallel(Graph graph, solution* sol) {
    // Student TO DO
    // Initialize the solution distances with infinity
    #pragma omp parallel for
    for(int i=0; i<graph->num_nodes; i++)
        sol->distances[i] = std::numeric_limits<int>::max();

    // The source node will have distance 0
    sol->distances[ROOT_NODE_ID] = 0;

    // Relax all edges |V| - 1 times
    #pragma omp parallel for
    for (int i = 1; i <= graph->num_nodes-1; i++) {
        #pragma omp parallel for
        for (int j = 0; j < graph->num_nodes; j++) {
            for (int k = graph->outgoing_starts[j]; k < (j == graph->num_nodes - 1 ? graph->num_edges : graph->outgoing_starts[j + 1]); k++) {
                int u = j;
                int v = graph->outgoing_edges[k];
                int weight = graph->outgoing_edges_len[k];
                if (sol->distances[u] != std::numeric_limits<int>::max() && sol->distances[u] + weight < sol->distances[v])
                {
                    //#pragma omp critical
                    {
                    sol->distances[v] = sol->distances[u] + weight;
                    }
                }
            }
        }
    }

    // check for negative-weight cycles
    // for (int j = 0; j < g.num_nodes; j++) {
    //     for (int k = g.outgoing_starts[j]; k < (j == g.num_nodes - 1 ? g.num_edges : g.outgoing_starts[j + 1]); k++) {
    //         int u = j;
    //         int v = g.outgoing_edges[k];
    //         int weight = g.outgoing_edges_len[k];
    //         if (sol->distances[u] != INT_MAX && sol->distances[u] + weight < sol->distances[v]) {
    //             printf("Graph contains negative weight cycle");
    //             return;
    //         }
    //     }
    // }

}