#include <stdio.h>
#include <stdlib.h>
#include <omp.h>
#include <string>
#include <getopt.h>

#include <iostream>
#include <sstream>
#include <vector>

#include "common/CycleTimer.h"
#include "common/graph.h"
#include "sssp.h"

#define USE_BINARY_GRAPH 1


int main(int argc, char** argv) {

    int  num_threads = -1;
    std::string graph_filename;

    if (argc < 2)
    {
        std::cerr << "Usage: <path/to/graph/file> [num_threads]\n";
        std::cerr << "  To run results for all thread counts: <path/to/graph/file>\n";
        std::cerr << "  Run with a certain number of threads (no correctness run): <path/to/graph/file> <num_threads>\n";
        exit(1);
    }

    int thread_count = -1;
    if (argc == 3)
    {
        thread_count = atoi(argv[2]);
    }

    graph_filename = argv[1];

    Graph g;

    printf("----------------------------------------------------------\n");
    printf("Max system threads = %d\n", omp_get_max_threads());
    if (thread_count > 0)
    {
        thread_count = std::min(thread_count, omp_get_max_threads());
        printf("Running with %d threads\n", thread_count);
    }
    printf("----------------------------------------------------------\n");

    printf("Loading graph...\n");
    if (USE_BINARY_GRAPH) {
      g = load_graph_binary(graph_filename.c_str());
    } else {
        g = load_graph(argv[1]);
        printf("storing binary form of graph!\n");
        store_graph_binary(graph_filename.append(".bin").c_str(), g);
        delete g;
        exit(1);
    }
    printf("\n");
    printf("Graph stats:\n");
    printf("  Edges: %d\n", g->num_edges);
    printf("  Nodes: %d\n", g->num_nodes);


    //Static assignment to get consistent usage across trials
    int max_threads = omp_get_max_threads();

    
    std::vector<int> num_threads_vec;

    //dynamic num_threads
    if(thread_count<=-1) {
        for (int i = 1; i < max_threads; i *= 2) {
            num_threads_vec.push_back(i);
        }
        num_threads_vec.push_back(max_threads);
    }
    //static num_threadss
    else {
        num_threads_vec.push_back(1);
        num_threads_vec.push_back(thread_count);
    }
    
    int n_usage = num_threads_vec.size();

    solution sol1;
    sol1.distances = std::vector<int>(g->num_nodes, 0);
    solution sol2;
    sol2.distances = std::vector<int>(g->num_nodes, 0);
    solution sol3;
    sol3.distances = std::vector<int>(g->num_nodes, 0);
    solution sol4;
    sol4.distances = std::vector<int>(g->num_nodes, 0);

    double sol1_time, sol2_time, sol3_time, sol4_time;
    double sol1_time_base, sol2_time_base, sol3_time_base, sol4_time_base;

    double start, end;
    std::stringstream timing;

    bool check1 = true, check2 = true, check3 = true;

    timing          << "Threads  Par-Dijsktra           Par-Bellman-Ford\n";


    std::cout << "Running Serial Dijkstra and Serial Bellman-Ford\n";
    //Run implementations of Serial Dijkstra
    dijkstra_serial(g, &sol1);

    //Run implementations of Serial Bellman-Ford
    bellmanford_serial(g, &sol3);

    std::cout << "Testing Correctness of Serial Dijkstra and Serial Bellman-Ford\n";
    for (int j=0; j<g->num_nodes; j++) {
        if (sol1.distances[j] != sol3.distances[j]) {
            fprintf(stderr, "*** Results disagree at %d: %d, %d\n", j, sol1.distances[j], sol3.distances[j]);
            check1= false;
            break;
        }
    }
    // for(int i=0;i<g->num_nodes;i++) std::cout<<sol1.distances[i]<<" ";
    // std::cout<<"\n";
    // for(int i=0;i<g->num_nodes;i++) std::cout<<sol3.distances[i]<<" ";
    // std::cout<<"\n";

    // make timer initialized
    CycleTimer::currentSeconds();
    //Loop through assignment values;
    for (int i = 0; i < n_usage; i++)
    {
        printf("----------------------------------------------------------\n");
        std::cout << "Running with " << num_threads_vec[i] << " threads" << std::endl;
        //Set thread count
        omp_set_num_threads(num_threads_vec[i]);

        //Run implementations of Parallel Dijkstra
        start = CycleTimer::currentSeconds();
        dijkstra_parallel(g, &sol2);
        end = CycleTimer::currentSeconds();
        sol2_time = end - start;

        //Run implementations of Parallel Bellman-Ford
        start = CycleTimer::currentSeconds();
        bellmanford_parallel(g, &sol4);
        end = CycleTimer::currentSeconds();
        sol4_time = end - start;
        
        
        // Your implementation of Parallel Dijkstra and Serial Dijkstra should have the same 
        std::cout << "Testing Correctness of Serial Dijkstra and Parallel Dijkstra\n";
        for (int j=0; j<g->num_nodes; j++) {
            if (sol1.distances[j] != sol2.distances[j]) {
                fprintf(stderr, "*** Results disagree at %d: %d, %d\n", j, sol1.distances[j], sol2.distances[j]);
                check2= false;
                break;
            }
        }
        // Your implementation of Parallel Bellman-Ford and Serial Bellman-Ford should have the same 
        std::cout << "Testing Correctness of Serial Bellman-Ford and Parallel Bellman-Ford\n";
        for (int j=0; j<g->num_nodes; j++) {
            if (sol3.distances[j] != sol4.distances[j]) {
                fprintf(stderr, "*** Results disagree at %d: %d, %d\n", j, sol3.distances[j], sol4.distances[j]);
                check3= false;
                break;
            }
        }

        if (i == 0) {
            sol2_time_base = sol2_time;
            sol4_time_base = sol4_time;
        }


        char buf[1024];
        char ref_buf[1024];
        char relative_buf[1024];

        sprintf(buf, "%4d:    %.3f (%.2fx)          %.3f (%.2fx)\n",
                num_threads_vec[i], sol2_time, sol2_time_base/sol2_time, 
                sol4_time, sol4_time_base/sol4_time);

        timing << buf;

    }

    printf("----------------------------------------------------------\n");
    std::cout << "Your Code: Timing Summary" << std::endl;
    std::cout << timing.str();
    printf("----------------------------------------------------------\n");
    std::cout << "Correctness: " << std::endl;
    if (!check1)
        std::cout << "Serial Dijkstra and Bellman-Ford have different results" << std::endl;
    if (!check2)
        std::cout << "Parallel Dijkstra is not Correct" << std::endl;
    if (!check3)
        std::cout << "Parallel Bellman-Ford is not Correct" << std::endl;
    
    

    delete g;

    return 0;
}
