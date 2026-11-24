// bench_astar.cpp - Part 3 performance & scalability study of all sequential and parallel
// A* versions. Metrics: wall-clock time (sum over a query set, median of R repetitions),
// expansions (search overhead of parallel versions) and solution quality (cost / optimum).
// Output CSV: platform,version,heuristic,threads,total_ms,avg_expanded,avg_ratio,max_ratio,speedup_vs_seq
#include <cstdio>
#include <cstring>
#include <functional>
#include "../src/par_astar.hpp"
using namespace hcm;
using namespace clist;
template <class T> struct Tag { using type = T; };

int main(int argc, char** argv) {
    std::string dir = "data", platform = "unknown"; int Q = 40, REPS = 3; double MIN_DIST = 3000;
    std::vector<int> Ts = {1, 2, 4, 8};
    bool quick = false, nohdr = false; std::string only_h = "";  // --heuristic X runs one heuristic only
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--data")) dir = argv[++i];
        else if (!std::strcmp(argv[i], "--platform")) platform = argv[++i];
        else if (!std::strcmp(argv[i], "--queries")) Q = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--reps")) REPS = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--quick")) quick = true;
        else if (!std::strcmp(argv[i], "--min-dist")) MIN_DIST = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--heuristic")) only_h = argv[++i];
        else if (!std::strcmp(argv[i], "--no-header")) nohdr = true;
        else if (!std::strcmp(argv[i], "--threads")) {
            Ts.clear(); std::string s = argv[++i]; size_t p = 0;
            while (p < s.size()) { size_t q = s.find(',', p); if (q == std::string::npos) q = s.size(); Ts.push_back(std::stoi(s.substr(p, q - p))); p = q + 1; }
        }
    }
    Graph g = Graph::load(dir);
    Landmarks lm = Landmarks::build(g, 8, LENGTH);
    std::mt19937 rng(2024);
    std::uniform_int_distribution<int> U(0, g.n - 1);
    std::vector<std::pair<int, int>> qs;
    while ((int)qs.size() < Q) { int s = U(rng), t = U(rng); if (g.dist_m(s, t) > MIN_DIST) qs.push_back({s, t}); }  // long trips
    std::vector<Cost> opt(Q);
    for (int i = 0; i < Q; ++i) opt[i] = dijkstra(g, qs[i].first, qs[i].second, Weight{LENGTH}).cost;

    if (!nohdr) std::printf("platform,version,heuristic,threads,total_ms,avg_expanded,avg_ratio,max_ratio\n");
    auto run = [&](const std::string& ver, const std::string& heur, int T, std::function<Result(int s, int t)> f) {
        std::vector<double> times; double ex = 0, rs = 0, rmax = 0;
        for (int r = 0; r < REPS; ++r) {
            double t0 = now_ms(); ex = 0; rs = 0; rmax = 0;
            for (int i = 0; i < Q; ++i) {
                Result res = f(qs[i].first, qs[i].second);
                ex += res.expanded; double ratio = res.cost / (double)opt[i]; rs += ratio; rmax = std::max(rmax, ratio);
            }
            times.push_back(now_ms() - t0);
        }
        std::sort(times.begin(), times.end());
        std::printf("%s,%s,%s,%d,%.3f,%.1f,%.6f,%.6f\n", platform.c_str(), ver.c_str(), heur.c_str(), T, times[times.size() / 2], ex / Q, rs / Q, rmax);
        std::fflush(stdout);
    };
    auto suite = [&](const std::string& H, auto mkH) {
        if (!only_h.empty() && H != only_h) return;
        // sequential baseline: binary-heap A* (Dijkstra when h = 0)
        run("seq-binary-heap", H, 1, [&](int s, int t) { auto h = mkH(t); return astar(g, s, t, h, Weight{LENGTH}); });
        for (int T : Ts) {
            auto P = [&](auto tag) {
                using PQ = typename decltype(tag)::type;
                run(PQ::name(), H, T, [&](int s, int t) { auto h = mkH(t); return (Result)par_astar_shared<PQ>(g, s, t, h, T); });
            };
            if (!(quick && H == "zero")) {  // sorted lists are O(n) per insert: very slow for h = 0
                P(Tag<ListPQ<CoarseList<MutexLock>>>{}); P(Tag<ListPQ<CoarseList<BackoffLock>>>{});
                P(Tag<ListPQ<FineList<MutexLock>>>{});   P(Tag<ListPQ<FineList<BackoffLock>>>{});
                P(Tag<ListPQ<OptimisticList<MutexLock>>>{}); P(Tag<ListPQ<OptimisticList<BackoffLock>>>{});
                P(Tag<ListPQ<LazyList<MutexLock>>>{});   P(Tag<ListPQ<LazyList<BackoffLock>>>{});
            }
            P(Tag<HeapPQ<MutexLock>>{}); P(Tag<HeapPQ<BackoffLock>>{});
            run("hda", H, T, [&](int s, int t) { auto h = mkH(t); return (Result)hda_star(g, s, t, h, T); });
            run("hda-window", H, T, [&](int s, int t) { auto h = mkH(t); return (Result)hda_star(g, s, t, h, T, LENGTH, 0.03); });
        }
    };
    suite("zero", [&](int) { return ZeroH{}; });
    suite("euclid", [&](int t) { return EuclidH{&g, t, LENGTH}; });
    suite("alt8", [&](int t) { return AltH{&lm, &g, t}; });
    suite("euclid-costly", [&](int t) { return CostlyH{EuclidH{&g, t, LENGTH}, 400}; });
}
