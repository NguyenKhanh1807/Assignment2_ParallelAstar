// bench_clist.cpp - Part 2 performance evaluation of the concurrent sorted lists.
// Usage scenario: an ordered in-memory index shared by worker threads, e.g. a route
// planner's shared set of "active" road segments / an open list. Three workload mixes:
//   read  : 90% contains,  5% add,  5% remove   (lookup-dominated index)
//   update: 50% contains, 25% add, 25% remove   (balanced)
//   pq    : 50% add(random key), 50% removeMin  (priority-queue usage, as in parallel A*)
// Output: CSV  platform,list,threads,mix,range,mops_per_s   (median of R repetitions)
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include "../src/clist.hpp"
using namespace clist;

static double DUR = 0.4; static int REPS = 3; static int RANGE = 1000;
static std::string PLATFORM = "unknown";
static std::string ONLY = ""; static bool NOHDR = false;  // run only lists whose name contains this substring

template <class List>
double run_once(int T, const std::string& mix) {
    List L;
    std::mt19937 r0(7);
    for (int i = 0; i < RANGE / 2; ++i) L.add(r0() % RANGE);
    std::atomic<bool> go{false}, stop{false};
    std::atomic<int> ready{0};
    std::vector<long> ops(T * 16, 0);  // padded to avoid false sharing
    std::vector<std::thread> th;
    for (int t = 0; t < T; ++t) th.emplace_back([&, t] {
        std::minstd_rand rng(1000 + t); long n = 0; Key m;
        ready++; while (!go.load()) {}
        while (!stop.load(std::memory_order_relaxed)) {
            for (int b = 0; b < 32; ++b) {
                Key k = rng() % RANGE; unsigned p = rng() % 100;
                if (mix == "read") { if (p < 90) L.contains(k); else if (p < 95) L.add(k); else L.remove(k); }
                else if (mix == "update") { if (p < 50) L.contains(k); else if (p < 75) L.add(k); else L.remove(k); }
                else { if (p < 50) L.add(k); else L.removeMin(m); }
                n++;
            }
        }
        ops[t * 16] = n;
    });
    while (ready.load() < T) {}
    auto t0 = std::chrono::steady_clock::now();
    go = true;
    std::this_thread::sleep_for(std::chrono::duration<double>(DUR));
    stop = true;
    for (auto& x : th) x.join();
    double sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    long tot = 0; for (int t = 0; t < T; ++t) tot += ops[t * 16];
    return tot / sec / 1e6;
}

template <class List>
void bench(const std::vector<int>& Ts) {
    if (!ONLY.empty() && List::name().find(ONLY) == std::string::npos) return;
    for (const char* mix : {"read", "update", "pq"})
        for (int T : Ts) {
            std::vector<double> v;
            for (int r = 0; r < REPS; ++r) v.push_back(run_once<List>(T, mix));
            std::sort(v.begin(), v.end());
            std::printf("%s,%s,%d,%s,%d,%.4f\n", PLATFORM.c_str(), List::name().c_str(), T, mix, RANGE, v[v.size() / 2]);
            std::fflush(stdout);
        }
}

int main(int argc, char** argv) {
    std::vector<int> Ts = {1, 2, 4, 8};
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--platform")) PLATFORM = argv[++i];
        else if (!std::strcmp(argv[i], "--only")) ONLY = argv[++i];
        else if (!std::strcmp(argv[i], "--no-header")) NOHDR = true;
        else if (!std::strcmp(argv[i], "--dur")) DUR = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--reps")) REPS = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--range")) RANGE = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--threads")) {
            Ts.clear(); std::string s = argv[++i]; size_t p = 0;
            while (p < s.size()) { size_t q = s.find(',', p); if (q == std::string::npos) q = s.size(); Ts.push_back(std::stoi(s.substr(p, q - p))); p = q + 1; }
        }
    }
    if (!NOHDR) std::printf("platform,list,threads,mix,range,mops\n");
    bench<CoarseList<MutexLock>>(Ts);  bench<CoarseList<TTASLock>>(Ts);  bench<CoarseList<BackoffLock>>(Ts);
    bench<FineList<MutexLock>>(Ts);    bench<FineList<TTASLock>>(Ts);    bench<FineList<BackoffLock>>(Ts);
    bench<OptimisticList<MutexLock>>(Ts); bench<OptimisticList<TTASLock>>(Ts); bench<OptimisticList<BackoffLock>>(Ts);
    bench<LazyList<MutexLock>>(Ts);    bench<LazyList<TTASLock>>(Ts);    bench<LazyList<BackoffLock>>(Ts);
}
