// test_par.cpp - correctness tests for Part 3 (parallel A*).
// For random (s,t) pairs and 1..maxT threads, every parallel version must return
//  * exactly the Dijkstra-optimal cost, and
//  * a valid s->t walk in the graph whose cost equals the returned cost.
// Run repeatedly (different interleavings) and under ThreadSanitizer (make tsan).
#include <cstdio>
#include "../src/par_astar.hpp"
using namespace hcm;
using namespace clist;

static int fails = 0, runs = 0;

template <class F>
void check(const char* name, const Graph& g, int s, int t, Cost opt, F f) {
    ParResult r = f();
    runs++;
    Cost pc = g.path_cost(r.path, LENGTH);
    bool ok = r.cost == opt && !r.path.empty() && r.path.front() == s && r.path.back() == t && pc == r.cost;
    if (!ok) { fails++; std::printf("  FAIL %-28s T=%d %d->%d cost=%lld opt=%lld pathcost=%lld\n", name, r.threads, s, t, (long long)r.cost, (long long)opt, (long long)pc); }
}

int main(int argc, char** argv) {
    std::string dir = argc > 1 ? argv[1] : "data";
    int maxT = argc > 2 ? std::atoi(argv[2]) : 8;
    int pairs = argc > 3 ? std::atoi(argv[3]) : 40;
    unsigned seed = argc > 4 ? (unsigned)std::atoi(argv[4]) : 99;
    Graph g = Graph::load(dir);
    Landmarks lm = Landmarks::build(g, 8, LENGTH);
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> U(0, g.n - 1);
    for (int i = 0; i < pairs; ++i) {
        int s = U(rng), t = U(rng);
        if (i == 0) t = s;  // degenerate case
        Cost opt = dijkstra(g, s, t, Weight{LENGTH}).cost;
        EuclidH he{&g, t, LENGTH}; AltH ha{&lm, &g, t};
        for (int T = 1; T <= maxT; T *= 2) {
            check("list-lazy-mutex/euclid", g, s, t, opt, [&] { return par_astar_shared<ListPQ<LazyList<MutexLock>>>(g, s, t, he, T); });
            check("list-optimistic-backoff", g, s, t, opt, [&] { return par_astar_shared<ListPQ<OptimisticList<BackoffLock>>>(g, s, t, he, T); });
            check("list-fine-mutex", g, s, t, opt, [&] { return par_astar_shared<ListPQ<FineList<MutexLock>>>(g, s, t, he, T); });
            check("list-coarse-backoff", g, s, t, opt, [&] { return par_astar_shared<ListPQ<CoarseList<BackoffLock>>>(g, s, t, he, T); });
            check("list-lazy-backoff/alt", g, s, t, opt, [&] { return par_astar_shared<ListPQ<LazyList<BackoffLock>>>(g, s, t, ha, T); });
            check("heap-mutex/euclid", g, s, t, opt, [&] { return par_astar_shared<HeapPQ<MutexLock>>(g, s, t, he, T); });
            check("heap-backoff/alt", g, s, t, opt, [&] { return par_astar_shared<HeapPQ<BackoffLock>>(g, s, t, ha, T); });
            check("hda/euclid", g, s, t, opt, [&] { return hda_star(g, s, t, he, T); });
            check("hda/alt", g, s, t, opt, [&] { return hda_star(g, s, t, ha, T); });
            check("hda-window/euclid", g, s, t, opt, [&] { return hda_star(g, s, t, he, T, LENGTH, 0.03); });
            check("hda-window/alt", g, s, t, opt, [&] { return hda_star(g, s, t, ha, T, LENGTH, 0.0); });
        }
        if ((i + 1) % 10 == 0) { std::printf("  %d pairs done, %d runs, %d failures\n", i + 1, runs, fails); std::fflush(stdout); }
    }
    std::printf("\n%s (%d runs, %d failures)\n", fails ? "PARALLEL A* TESTS FAILED" : "ALL PARALLEL A* TESTS PASSED", runs, fails);
    return fails ? 1 : 0;
}
