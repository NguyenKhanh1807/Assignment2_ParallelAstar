# Assignment 2 - Parallel Computing. Requires g++ >= 9 (C++17) and pthreads. Linux / macOS.
CXX      ?= g++
CXXFLAGS ?= -O2 -std=c++17 -Wall -Wextra -pthread
HDR = src/graph.hpp src/seq_astar.hpp src/clist.hpp src/par_astar.hpp
BINS = bin/test_seq bin/test_clist bin/test_par bin/bench_seq bin/bench_clist bin/bench_astar bin/route

all: $(BINS)
bin/%: tests/%.cpp $(HDR) | bin ; $(CXX) $(CXXFLAGS) -o $@ $<
bin/%: bench/%.cpp $(HDR) | bin ; $(CXX) $(CXXFLAGS) -o $@ $<
bin/route: src/route.cpp $(HDR) | bin ; $(CXX) $(CXXFLAGS) -o $@ $<
bin: ; mkdir -p bin

test: bin/test_seq bin/test_clist bin/test_par
	./bin/test_seq data
	./bin/test_clist 8
	./bin/test_par data 8 60

tsan: | bin   # data-race detection with ThreadSanitizer
	$(CXX) -O1 -g -std=c++17 -fsanitize=thread -pthread -o bin/test_clist_tsan tests/test_clist.cpp
	$(CXX) -O1 -g -std=c++17 -fsanitize=thread -pthread -o bin/test_par_tsan tests/test_par.cpp
	./bin/test_clist_tsan 4 --no-ttas
	./bin/test_par_tsan data 4 6

graph:  # rebuild data/nodes.csv, data/edges.csv from the raw OSM extracts
	python3 scripts/build_graph.py

clean: ; rm -rf bin
.PHONY: all test tsan graph clean
