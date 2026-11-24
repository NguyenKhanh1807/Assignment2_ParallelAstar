// route.cpp - command-line route planner for central Ho Chi Minh City.
//   ./bin/route <lat1> <lon1> <lat2> <lon2> [--algo dijkstra|astar|alt|par-lazy|par-heap|hda]
//               [--threads T] [--metric length|time] [--geojson out.geojson]
// Example (Ben Thanh Market -> Thao Cam Vien zoo):
//   ./bin/route 10.7725 106.6980 10.7875 106.7053 --algo alt
#include <cstdio>
#include <cstring>
#include "par_astar.hpp"
using namespace hcm;

int main(int argc, char** argv) {
    if (argc < 5) { std::fprintf(stderr, "usage: %s lat1 lon1 lat2 lon2 [--algo A] [--threads T] [--metric length|time] [--geojson F] [--data DIR]\n", argv[0]); return 1; }
    double la1 = std::atof(argv[1]), lo1 = std::atof(argv[2]), la2 = std::atof(argv[3]), lo2 = std::atof(argv[4]);
    std::string algo = "astar", dir = "data", gj; int T = 4, m = LENGTH;
    for (int i = 5; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--algo")) algo = argv[++i];
        else if (!std::strcmp(argv[i], "--threads")) T = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--metric")) m = std::strcmp(argv[++i], "time") ? LENGTH : TIME;
        else if (!std::strcmp(argv[i], "--geojson")) gj = argv[++i];
        else if (!std::strcmp(argv[i], "--data")) dir = argv[++i];
    }
    Graph g = Graph::load(dir);
    int s = g.nearest(la1, lo1), t = g.nearest(la2, lo2);
    Result r;
    EuclidH h{&g, t, m};
    if (algo == "dijkstra") r = dijkstra(g, s, t, Weight{m});
    else if (algo == "astar") r = astar(g, s, t, h, Weight{m});
    else if (algo == "alt") { Landmarks lm = Landmarks::build(g, 8, m); AltH ha{&lm, &g, t}; r = astar(g, s, t, ha, Weight{m}); }
    else if (algo == "par-lazy") r = par_astar_shared<ListPQ<clist::LazyList<clist::BackoffLock>>>(g, s, t, h, T, m);
    else if (algo == "par-heap") r = par_astar_shared<HeapPQ<clist::BackoffLock>>(g, s, t, h, T, m);
    else if (algo == "hda") r = hda_star(g, s, t, h, T, m);
    else { std::fprintf(stderr, "unknown algo\n"); return 1; }
    if (r.path.empty()) { std::printf("no path\n"); return 2; }
    std::printf("from node %d (%.5f,%.5f) to node %d (%.5f,%.5f)\n", s, g.lat[s], g.lon[s], t, g.lat[t], g.lon[t]);
    std::printf("algorithm=%s  length=%.1f m  free-flow time=%.1f s  nodes on path=%zu  expanded=%ld  %.3f ms\n",
                algo.c_str(), g.path_cost(r.path, LENGTH) / 1000.0, g.path_cost(r.path, TIME) / 1000.0, r.path.size(), r.expanded, r.ms);
    if (!gj.empty()) {
        FILE* f = std::fopen(gj.c_str(), "w");
        std::fprintf(f, "{\"type\":\"Feature\",\"properties\":{\"algo\":\"%s\"},\"geometry\":{\"type\":\"LineString\",\"coordinates\":[", algo.c_str());
        for (size_t i = 0; i < r.path.size(); ++i) std::fprintf(f, "%s[%.7f,%.7f]", i ? "," : "", g.lon[r.path[i]], g.lat[r.path[i]]);
        std::fprintf(f, "]}}\n"); std::fclose(f);
    }
}
