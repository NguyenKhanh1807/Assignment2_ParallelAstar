// bench_astar.cpp - Part 3 performance & scalability study of all sequential and parallel
// A* versions. Metrics: wall-clock time (sum over a query set, median of R repetitions),
// expansions (search overhead of parallel versions) and solution quality (cost / optimum).
// Output CSV: platform,version,heuristic,threads,total_ms,avg_expanded,avg_ratio,max_ratio
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <functional>
#include <iomanip>
#include <memory>
#include <set>
#include <sstream>
#include "../src/par_astar.hpp"
using namespace hcm;
using namespace clist;
template <class T> struct Tag { using type = T; };

static std::vector<int> parse_ints(const std::string& s) {
    std::vector<int> out; size_t p = 0;
    while (p < s.size()) {
        size_t q = s.find(',', p); if (q == std::string::npos) q = s.size();
        if (q > p) out.push_back(std::max(1, std::stoi(s.substr(p, q - p))));
        p = q + 1;
    }
    return out;
}
static std::vector<double> parse_doubles(const std::string& s) {
    std::vector<double> out; size_t p = 0;
    while (p < s.size()) {
        size_t q = s.find(',', p); if (q == std::string::npos) q = s.size();
        if (q > p) out.push_back(std::stod(s.substr(p, q - p)));
        p = q + 1;
    }
    return out;
}

int main(int argc, char** argv) {
    std::string dir = "data", platform = "unknown"; int Q = 40, REPS = 3, metric = LENGTH; double MIN_DIST = 3000;
    std::vector<int> Ts = {1, 2, 4, 8};
    std::vector<double> windows;
    std::set<std::string> exact_vers;
    bool quick = false, nohdr = false, use_pool = false, use_pin = false, include_ttas = false, skip_fine_ttas = false;
    std::string only_h = "", only_ver = "", stats_out = "";
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--data")) dir = argv[++i];
        else if (!std::strcmp(argv[i], "--platform")) platform = argv[++i];
        else if (!std::strcmp(argv[i], "--queries")) Q = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--reps")) REPS = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--quick")) quick = true;
        else if (!std::strcmp(argv[i], "--min-dist")) MIN_DIST = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--heuristic")) only_h = argv[++i];
        else if (!std::strcmp(argv[i], "--only-ver")) only_ver = argv[++i];
        else if (!std::strcmp(argv[i], "--vers")) {
            std::string s = argv[++i]; size_t p = 0;
            while (p < s.size()) { size_t q = s.find(',', p); if (q == std::string::npos) q = s.size(); exact_vers.insert(s.substr(p, q - p)); p = q + 1; }
        }
        else if (!std::strcmp(argv[i], "--metric")) metric = std::strcmp(argv[++i], "time") ? LENGTH : TIME;
        else if (!std::strcmp(argv[i], "--pool")) use_pool = true;
        else if (!std::strcmp(argv[i], "--pin")) use_pin = true;
        else if (!std::strcmp(argv[i], "--include-ttas")) include_ttas = true;
        else if (!std::strcmp(argv[i], "--skip-fine-ttas")) skip_fine_ttas = true;
        else if (!std::strcmp(argv[i], "--windows")) windows = parse_doubles(argv[++i]);
        else if (!std::strcmp(argv[i], "--stats-out")) stats_out = argv[++i];
        else if (!std::strcmp(argv[i], "--no-header")) nohdr = true;
        else if (!std::strcmp(argv[i], "--threads")) Ts = parse_ints(argv[++i]);
    }
    Graph g = Graph::load(dir);
    Landmarks lm = Landmarks::build(g, 8, metric);
    std::mt19937 rng(2024);
    std::uniform_int_distribution<int> U(0, g.n - 1);
    std::vector<std::pair<int, int>> qs;
    while ((int)qs.size() < Q) { int s = U(rng), t = U(rng); if (g.dist_m(s, t) > MIN_DIST) qs.push_back({s, t}); }  // long trips
    std::vector<Cost> opt(Q);
    for (int i = 0; i < Q; ++i) opt[i] = dijkstra(g, qs[i].first, qs[i].second, Weight{metric}).cost;

    const char* mname = metric == TIME ? "time" : "length";
    if (!nohdr) std::printf("platform,version,heuristic,threads,total_ms,avg_expanded,avg_ratio,max_ratio\n");
    FILE* sf = nullptr;
    if (!stats_out.empty()) {
        sf = std::fopen(stats_out.c_str(), nohdr ? "a" : "w");
        if (!nohdr && sf) std::fprintf(sf, "platform,version,heuristic,metric,threads,pool,pin,reps,median_ms,mean_ms,std_ms,min_ms,max_ms,iqr_ms,q_p50_ms,q_p95_ms,q_p99_ms,avg_expanded,avg_retired_kb,rss_kb,max_rss_kb,avg_ratio,max_ratio\n");
    }
    auto run = [&](const std::string& ver, const std::string& heur, int T, std::function<ParResult(int s, int t)> f) {
        if (!exact_vers.empty() && !exact_vers.count(ver)) return;
        if (!only_ver.empty() && ver.find(only_ver) == std::string::npos) return;
        std::vector<double> times, qms;
        qms.reserve((size_t)Q * REPS);
        double ex = 0, ret_kb = 0, rs = 0, rmax = 0;
        for (int r = 0; r < REPS; ++r) {
            double t0 = now_ms();
            for (int i = 0; i < Q; ++i) {
                ParResult res = f(qs[i].first, qs[i].second);
                qms.push_back(res.ms);
                ex += res.expanded;
                ret_kb += res.retired_bytes / 1024.0;
                double ratio = res.cost / (double)opt[i]; rs += ratio; rmax = std::max(rmax, ratio);
            }
            times.push_back(now_ms() - t0);
        }
        double denom = (double)Q * REPS;
        Stats st = compute_stats(times);
        std::sort(qms.begin(), qms.end());
        std::printf("%s,%s,%s,%d,%.3f,%.1f,%.6f,%.6f\n", platform.c_str(), ver.c_str(), heur.c_str(), T, st.median, ex / denom, rs / denom, rmax);
        std::fflush(stdout);
        if (sf) {
            int row_pool = ver == "seq-binary-heap" ? 0 : (int)use_pool;
            int row_pin = ver == "seq-binary-heap" ? 0 : (int)use_pin;
            std::fprintf(sf, "%s,%s,%s,%s,%d,%d,%d,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.1f,%.2f,%ld,%ld,%.6f,%.6f\n",
                         platform.c_str(), ver.c_str(), heur.c_str(), mname, T, row_pool, row_pin, REPS,
                         st.median, st.mean, st.std, st.min, st.max, st.iqr,
                         percentile_sorted(qms, 0.50), percentile_sorted(qms, 0.95), percentile_sorted(qms, 0.99),
                         ex / denom, ret_kb / denom, current_rss_kb(), max_rss_kb(), rs / denom, rmax);
            std::fflush(sf);
        }
    };
    auto suite = [&](const std::string& H, auto mkH) {
        if (!only_h.empty() && H != only_h) return;
        // sequential baseline: binary-heap A* (Dijkstra when h = 0)
        run("seq-binary-heap", H, 1, [&](int s, int t) { auto h = mkH(t); ParResult r; static_cast<Result&>(r) = astar(g, s, t, h, Weight{metric}); return r; });
        for (int T : Ts) {
            std::unique_ptr<ThreadPool> pool = use_pool ? std::make_unique<ThreadPool>(T, use_pin) : nullptr;
            auto P = [&](auto tag) {
                using PQ = typename decltype(tag)::type;
                run(PQ::name(), H, T, [&](int s, int t) { auto h = mkH(t); return par_astar_shared<PQ>(g, s, t, h, T, metric, pool.get(), use_pin); });
            };
            if (!(quick && H == "zero")) {  // sorted lists are O(n) per insert: very slow for h = 0
                P(Tag<ListPQ<CoarseList<MutexLock>>>{}); P(Tag<ListPQ<CoarseList<BackoffLock>>>{});
                P(Tag<ListPQ<FineList<MutexLock>>>{});   P(Tag<ListPQ<FineList<BackoffLock>>>{});
                P(Tag<ListPQ<OptimisticList<MutexLock>>>{}); P(Tag<ListPQ<OptimisticList<BackoffLock>>>{});
                P(Tag<ListPQ<LazyList<MutexLock>>>{});   P(Tag<ListPQ<LazyList<BackoffLock>>>{});
                if (include_ttas) {
                    P(Tag<ListPQ<CoarseList<TTASLock>>>{});
                    if (!skip_fine_ttas) P(Tag<ListPQ<FineList<TTASLock>>>{});
                    P(Tag<ListPQ<OptimisticList<TTASLock>>>{});
                    P(Tag<ListPQ<LazyList<TTASLock>>>{});
                }
            }
            P(Tag<HeapPQ<MutexLock>>{}); P(Tag<HeapPQ<BackoffLock>>{});
            if (include_ttas) P(Tag<HeapPQ<TTASLock>>{});
            run("hda", H, T, [&](int s, int t) { auto h = mkH(t); return hda_star(g, s, t, h, T, metric, -1.0, pool.get(), use_pin); });
            if (windows.empty()) {
                run("hda-window", H, T, [&](int s, int t) { auto h = mkH(t); return hda_star(g, s, t, h, T, metric, 0.03, pool.get(), use_pin); });
            } else {
                for (double w : windows) {
                    std::ostringstream os; os << "hda-w" << std::fixed << std::setprecision(2) << w;
                    run(os.str(), H, T, [&](int s, int t) { auto h = mkH(t); return hda_star(g, s, t, h, T, metric, w, pool.get(), use_pin); });
                }
            }
        }
    };
    suite("zero", [&](int) { return ZeroH{}; });
    suite("euclid", [&](int t) { return EuclidH{&g, t, metric}; });
    suite("alt8", [&](int t) { return AltH{&lm, &g, t}; });
    suite("euclid-costly", [&](int t) { return CostlyH{EuclidH{&g, t, metric}, 400}; });
    if (sf) std::fclose(sf);
}
