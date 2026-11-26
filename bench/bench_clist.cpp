// bench_clist.cpp - Part 2 performance evaluation of the concurrent sorted lists.
// Usage scenario: an ordered in-memory index shared by worker threads, e.g. a route
// planner's shared set of "active" road segments / an open list. Three workload mixes:
//   read  : 90% contains,  5% add,  5% remove   (lookup-dominated index)
//   update: 50% contains, 25% add, 25% remove   (balanced)
//   pq    : 50% add(random key), 50% removeMin  (priority-queue usage, as in parallel A*)
// Output: CSV  platform,list,threads,mix,range,mops   (median of R repetitions)
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include "../src/par_astar.hpp"
using namespace clist;

static double DUR = 0.4; static int REPS = 3; static int RANGE = 1000;
static std::string PLATFORM = "unknown";
static std::string ONLY = "", STATS_OUT = "";
static bool NOHDR = false, PIN = false, INCLUDE_HEAP = false, NO_TTAS = false;

template <class List>
struct SetAdaptor {
    static constexpr bool pq_only = false;
    List impl;
    static std::string name() { return List::name(); }
    bool add(Key k) { return impl.add(k); }
    bool remove(Key k) { return impl.remove(k); }
    bool contains(Key k) { return impl.contains(k); }
    bool removeMin(Key& m) { return impl.removeMin(m); }
};
template <class Lock>
struct HeapAdaptor {
    static constexpr bool pq_only = true;
    hcm::HeapPQ<Lock> impl;
    static std::string name() { return hcm::HeapPQ<Lock>::name(); }
    bool add(Key k) { return impl.push(k); }
    bool remove(Key) { return false; }
    bool contains(Key) { return false; }
    bool removeMin(Key& m) { return impl.pop(m); }
};

template <class Container>
double run_once(int T, const std::string& mix) {
    Container L;
    std::mt19937 r0(7);
    for (int i = 0; i < RANGE / 2; ++i) L.add(r0() % RANGE);
    std::atomic<bool> go{false}, stop{false};
    std::atomic<int> ready{0};
    std::vector<long> ops(T * 16, 0);  // padded to avoid false sharing
    std::vector<std::thread> th;
    for (int t = 0; t < T; ++t) th.emplace_back([&, t] {
        hcm::AffinityGuard ag(PIN, t, false);
        std::minstd_rand rng(1000 + t); long n = 0; Key m;
        ready++; while (!go.load()) {}
        while (!stop.load(std::memory_order_relaxed)) {
            for (int b = 0; b < 32; ++b) {
                Key k = rng() % RANGE; unsigned p = rng() % 100;
                if constexpr (!Container::pq_only) {
                    if (mix == "read") { if (p < 90) L.contains(k); else if (p < 95) L.add(k); else L.remove(k); }
                    else if (mix == "update") { if (p < 50) L.contains(k); else if (p < 75) L.add(k); else L.remove(k); }
                    else { if (p < 50) L.add(k); else L.removeMin(m); }
                } else {
                    if (p < 50) L.add(k); else L.removeMin(m);
                }
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

static FILE* SF = nullptr;

template <class Container>
void bench(const std::vector<int>& Ts) {
    if (!ONLY.empty() && Container::name().find(ONLY) == std::string::npos) return;
    for (const char* mix : {"read", "update", "pq"}) {
        if constexpr (Container::pq_only) if (std::strcmp(mix, "pq") != 0) continue;
        for (int T : Ts) {
            std::vector<double> v;
            for (int r = 0; r < REPS; ++r) v.push_back(run_once<Container>(T, mix));
            hcm::Stats st = hcm::compute_stats(v);
            std::printf("%s,%s,%d,%s,%d,%.4f\n", PLATFORM.c_str(), Container::name().c_str(), T, mix, RANGE, st.median);
            std::fflush(stdout);
            if (SF) {
                std::fprintf(SF, "%s,%s,%d,%s,%d,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%ld\n",
                             PLATFORM.c_str(), Container::name().c_str(), T, mix, RANGE, REPS,
                             st.median, st.mean, st.std, st.min, st.max, st.iqr, hcm::current_rss_kb());
                std::fflush(SF);
            }
        }
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
        else if (!std::strcmp(argv[i], "--pin")) PIN = true;
        else if (!std::strcmp(argv[i], "--include-heap")) INCLUDE_HEAP = true;
        else if (!std::strcmp(argv[i], "--no-ttas")) NO_TTAS = true;
        else if (!std::strcmp(argv[i], "--stats-out")) STATS_OUT = argv[++i];
        else if (!std::strcmp(argv[i], "--threads")) {
            Ts.clear(); std::string s = argv[++i]; size_t p = 0;
            while (p < s.size()) {
                size_t q = s.find(',', p); if (q == std::string::npos) q = s.size();
                if (q > p) Ts.push_back(std::max(1, std::stoi(s.substr(p, q - p))));
                p = q + 1;
            }
        }
    }
    if (!NOHDR) std::printf("platform,list,threads,mix,range,mops\n");
    if (!STATS_OUT.empty()) {
        SF = std::fopen(STATS_OUT.c_str(), NOHDR ? "a" : "w");
        if (!NOHDR && SF) std::fprintf(SF, "platform,list,threads,mix,range,reps,median_mops,mean_mops,std_mops,min_mops,max_mops,iqr_mops,rss_kb\n");
    }
    bench<SetAdaptor<CoarseList<MutexLock>>>(Ts);     bench<SetAdaptor<FineList<MutexLock>>>(Ts);
    bench<SetAdaptor<OptimisticList<MutexLock>>>(Ts); bench<SetAdaptor<LazyList<MutexLock>>>(Ts);
    if (!NO_TTAS) {
        bench<SetAdaptor<CoarseList<TTASLock>>>(Ts);     bench<SetAdaptor<FineList<TTASLock>>>(Ts);
        bench<SetAdaptor<OptimisticList<TTASLock>>>(Ts); bench<SetAdaptor<LazyList<TTASLock>>>(Ts);
    }
    bench<SetAdaptor<CoarseList<BackoffLock>>>(Ts);     bench<SetAdaptor<FineList<BackoffLock>>>(Ts);
    bench<SetAdaptor<OptimisticList<BackoffLock>>>(Ts); bench<SetAdaptor<LazyList<BackoffLock>>>(Ts);
    if (INCLUDE_HEAP) {
        bench<HeapAdaptor<MutexLock>>(Ts);
        if (!NO_TTAS) bench<HeapAdaptor<TTASLock>>(Ts);
        bench<HeapAdaptor<BackoffLock>>(Ts);
    }
    if (SF) std::fclose(SF);
}
