all: default

default: main.cpp sssp.cpp
	g++-13 -I../ -std=c++11 -lstdc++ -fopenmp -O3 -g -o sssp main.cpp sssp.cpp ../common/graph.cpp
clean:
	rm -rf sssp  *~ *.*~
