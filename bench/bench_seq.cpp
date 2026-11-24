// bench_seq.cpp - Part 1 (+ Part 4 ALT) experiments with the sequential A* family.
//  (a) static / uncertain / dynamic heuristics and ALT: time, expansions, solution quality
//  (b) vector-valued f (NAMOA*): Pareto-front size and run time
//  (c) dynamic world: traffic jams appear while driving; replanning A* vs. static plan
#include <cstdio>
#include <cstring>
#include <map>
#include "../src/seq_astar.hpp"
using namespace hcm;

int main(int argc, char** argv) {
    std::string dir = "data", out = "results", platform = "unknown", parts = "abcd";
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--data")) dir = argv[++i];
        else if (!std::strcmp(argv[i], "--out")) out = argv[++i];
        else if (!std::strcmp(argv[i], "--platform")) platform = argv[++i];
        else if (!std::strcmp(argv[i], "--parts")) parts = argv[++i];   // subset of "abcd"
    }
    Graph g = Graph::load(dir);
    std::mt19937 rng(7);
    std::uniform_int_distribution<int> U(0, g.n - 1);
    const int Q = 200;
    std::vector<std::pair<int, int>> qs;
    while ((int)qs.size() < Q) { int s = U(rng), t = U(rng); if (g.dist_m(s, t) > 1000) qs.push_back({s, t}); }

    if (parts.find('a') != std::string::npos) {
    // ---------------- (a)
    FILE* fa = std::fopen((out + "/seq_heuristics_" + platform + ".csv").c_str(), "w");
    std::fprintf(fa, "platform,metric,algorithm,eps,avg_ms,avg_expanded,avg_reopened,avg_stale,avg_ratio,max_ratio\n");
    double t0 = now_ms();
    std::vector<Landmarks> LMs;
    std::vector<double> lm_build_ms;
    for (int k : {4, 8, 16}) { double a = now_ms(); LMs.push_back(Landmarks::build(g, k, LENGTH)); lm_build_ms.push_back(now_ms() - a); }
    std::printf("ALT preprocessing: k=4 %.0f ms, k=8 %.0f ms, k=16 %.0f ms (graph load+build %.0f ms)\n", lm_build_ms[0], lm_build_ms[1], lm_build_ms[2], now_ms() - t0);
    for (int m : {LENGTH, TIME}) {
        std::vector<Cost> opt(Q);
        auto report = [&](const char* name, double eps, auto run) {
            double ms = 0, ex = 0, ro = 0, st = 0, rs = 0, rmax = 1;
            for (int i = 0; i < Q; ++i) {
                Result r = run(i);
                ms += r.ms; ex += r.expanded; ro += r.reopened; st += r.stale;
                double ratio = opt[i] ? r.cost / (double)opt[i] : 1.0;
                rs += ratio; rmax = std::max(rmax, ratio);
            }
            std::fprintf(fa, "%s,%s,%s,%.2f,%.4f,%.1f,%.2f,%.2f,%.6f,%.6f\n", platform.c_str(), m == LENGTH ? "length" : "time",
                         name, eps, ms / Q, ex / Q, ro / Q, st / Q, rs / Q, rmax);
        };
        for (int i = 0; i < Q; ++i) opt[i] = dijkstra(g, qs[i].first, qs[i].second, Weight{m}).cost;
        report("dijkstra", 0, [&](int i) { return dijkstra(g, qs[i].first, qs[i].second, Weight{m}); });
        report("astar-euclid", 0, [&](int i) { EuclidH h{&g, qs[i].second, m}; return astar(g, qs[i].first, qs[i].second, h, Weight{m}); });
        if (m == LENGTH) {
            const char* nm[] = {"astar-alt4", "astar-alt8", "astar-alt16"};
            for (int k = 0; k < 3; ++k)
                report(nm[k], 0, [&](int i) { AltH h{&LMs[k], &g, qs[i].second}; return astar(g, qs[i].first, qs[i].second, h, Weight{m}); });
        }
        for (double eps : {0.1, 0.3, 0.5, 1.0, 2.0}) {
            report("astar-uncertain", eps, [&](int i) { UncertainH h(&g, qs[i].second, m, eps, 31 * i + 1); return astar(g, qs[i].first, qs[i].second, h, Weight{m}); });
            report("astar-dynamic", eps, [&](int i) { DynamicH h{&g, qs[i].second, m, eps, 300}; return astar(g, qs[i].first, qs[i].second, h, Weight{m}); });
        }
    }
    std::fclose(fa);

    }
    if (parts.find('b') != std::string::npos) {
    rng.seed(11);
    // ---------------- (b) NAMOA*
    FILE* fb = std::fopen((out + "/seq_namoa_" + platform + ".csv").c_str(), "w");
    std::fprintf(fb, "platform,s,t,straight_m,front_size,namoa_ms,namoa_expanded,zero_h_ms,zero_h_expanded,front\n");
    int done = 0;
    while (done < 40) {
        int s = U(rng), t = U(rng); double d = g.dist_m(s, t);
        double lo = 500 + 100.0 * done, hi = lo + 400;
        if (d < lo || d > hi) continue;
        EuclidH h1{&g, t, LENGTH}, h2{&g, t, CONGESTED};
        auto P = namoa(g, s, t, LENGTH, CONGESTED, h1, h2);
        ZeroH z1, z2;
        auto Z = namoa(g, s, t, LENGTH, CONGESTED, z1, z2);
        std::string fr;
        for (auto& v : P.front) fr += std::to_string(v.a / 1000.0).substr(0, 7) + ":" + std::to_string(v.b / 1000.0).substr(0, 7) + ";";
        std::fprintf(fb, "%s,%d,%d,%.0f,%zu,%.3f,%ld,%.3f,%ld,%s\n", platform.c_str(), s, t, d, P.front.size(), P.ms, P.expanded, Z.ms, Z.expanded, fr.c_str());
        done++;
    }
    std::fclose(fb);

    }
    if (parts.find('c') != std::string::npos) {
    rng.seed(12);
    // ---------------- (c) dynamic traffic: replanning vs static plan
    FILE* fc = std::fopen((out + "/seq_dynamic_" + platform + ".csv").c_str(), "w");
    std::fprintf(fc, "platform,trip,static_s,replan_s,free_flow_s,replans,avg_replan_ms\n");
    std::vector<double> mult(g.adj.size(), 1.0);
    auto WT = [&](const Edge& e) -> Cost { return (Cost)std::llround(e.w[TIME] * mult[&e - g.adj.data()]); };
    auto cost_edge = [&](int u, int v) { Cost b = INF; for (auto e = g.begin(u); e != g.end(u); ++e) if (e->to == v) b = std::min(b, WT(*e)); return b; };
    std::vector<int> esrc(g.adj.size());
    for (int u = 0; u < g.n; ++u) for (int k = g.off[u]; k < g.off[u + 1]; ++k) esrc[k] = u;
    for (int trip = 0; trip < 40; ++trip) {
        int s, t; do { s = U(rng); t = U(rng); } while (g.dist_m(s, t) < 3500);
        std::fill(mult.begin(), mult.end(), 1.0);
        EuclidH h{&g, t, TIME};
        auto plan0 = astar(g, s, t, h, WT);
        double freeflow = plan0.cost / 1000.0;
        // Pre-generate the jam scenario (same for both strategies): every 15 edges driven along
        // the static plan, 3 jams (x3..x8 slower, radius 300 m) appear further ahead on that plan.
        std::mt19937 jr(1000 + trip);
        struct Jam { int at_step; int centre; double f; };
        std::vector<Jam> jams;
        for (int step = 5; step < (int)plan0.path.size(); step += 15)
            for (int j = 0; j < 3; ++j) {
                int ahead = std::min((int)plan0.path.size() - 1, step + 5 + (int)(jr() % 40));
                jams.push_back({step, plan0.path[ahead], 3.0 + (jr() % 6)});
            }
        auto apply_jams = [&](int step) {
            for (auto& J : jams) if (J.at_step == step)
                for (size_t k = 0; k < g.adj.size(); ++k)
                    if (g.dist_m(esrc[k], J.centre) < 300) mult[k] = std::max(mult[k], J.f);
        };
        auto drive = [&](bool replan, int& nre, double& rms) {
            std::fill(mult.begin(), mult.end(), 1.0);
            std::vector<int> path = plan0.path; size_t pos = 0; double total = 0; int step = 0; nre = 0; rms = 0;
            while (path[pos] != t) {
                int before = 0; for (auto& J : jams) before += J.at_step == step;
                apply_jams(step);
                if (replan && before) {
                    auto r = astar(g, path[pos], t, h, WT); nre++; rms += r.ms;
                    path = r.path; pos = 0;
                }
                total += cost_edge(path[pos], path[pos + 1]) / 1000.0;
                pos++; step++;
            }
            return total;
        };
        int n1, n2; double r1, r2;
        double st = drive(false, n1, r1), dy = drive(true, n2, r2);
        std::fprintf(fc, "%s,%d,%.2f,%.2f,%.2f,%d,%.4f\n", platform.c_str(), trip, st, dy, freeflow, n2, n2 ? r2 / n2 : 0);
    }
    std::fclose(fc);
    }
    if (parts.find('d') != std::string::npos) {
    // ---------------- (d) time-dependent travel times (lecturer's speed profile)
    {
        SpeedProfile P; double vmax = P.vmax();
        FILE* fd = std::fopen((out + "/seq_td_" + platform + ".csv").c_str(), "w");
        std::fprintf(fd, "platform,heuristic,avg_ms,avg_expanded,avg_reopened,avg_ratio,max_ratio,frac_suboptimal\n");
        const double T0[] = {0.0, 4.5, 9.0, 13.5};
        struct Acc { double ms = 0, ex = 0, ro = 0, rs = 0, rmax = 0, sub = 0; int n = 0; };
        std::map<std::string, Acc> acc;
        for (int i = 0; i < Q; ++i) {
            int s = qs[i].first, t = qs[i].second;
            auto dt = sssp(g, t, LENGTH, true);       // for the exact-remaining-length heuristic
            for (double t0 : T0) {
                double opt = td_astar(g, s, t, t0, P, [](int, double) { return 0.0; }).minutes;
                auto add = [&](const std::string& name, const TDResult& r) {
                    Acc& a = acc[name]; double q = r.minutes / opt;
                    a.ms += r.ms; a.ex += r.expanded; a.ro += r.reopened; a.rs += q; a.rmax = std::max(a.rmax, q);
                    a.sub += q > 1 + 1e-9; a.n++;
                };
                add("zero (TD-Dijkstra)", td_astar(g, s, t, t0, P, [](int, double) { return 0.0; }));
                add("great-circle / vmax", td_astar(g, s, t, t0, P, [&](int v, double) { return g.dist_m(v, t) / vmax; }));
                add("exact length / vmax", td_astar(g, s, t, t0, P, [&](int v, double) { return dt[v] / 1000.0 / vmax; }));
                add("lecturer: great-circle / speed(t)", td_astar(g, s, t, t0, P, [&](int v, double tn) { return g.dist_m(v, t) / P.speed(tn); }));
            }
        }
        for (auto& [name, a] : acc)
            std::fprintf(fd, "%s,%s,%.4f,%.1f,%.1f,%.6f,%.6f,%.4f\n", platform.c_str(), name.c_str(), a.ms / a.n, a.ex / a.n,
                         a.ro / a.n, a.rs / a.n, a.rmax, a.sub / a.n);
        std::fclose(fd);
        // departure-time sweep for the longest query
        int bi = 0; for (int i = 1; i < Q; ++i) if (g.dist_m(qs[i].first, qs[i].second) > g.dist_m(qs[bi].first, qs[bi].second)) bi = i;
        int s = qs[bi].first, t = qs[bi].second;
        FILE* fs = std::fopen((out + "/seq_td_sweep_" + platform + ".csv").c_str(), "w");
        std::fprintf(fs, "platform,depart_min,optimal_min,lecturer_h_min,departure_speed_model_min\n");
        auto dl = dijkstra(g, s, t, Weight{LENGTH});
        for (double t0 = 0; t0 <= 20.0 + 1e-9; t0 += 0.25) {
            double o = td_astar(g, s, t, t0, P, [&](int v, double) { return g.dist_m(v, t) / vmax; }).minutes;
            double l = td_astar(g, s, t, t0, P, [&](int v, double tn) { return g.dist_m(v, t) / P.speed(tn); }).minutes;
            double tt = t0;  // the notebook's edge model along the same (shortest) route
            for (size_t k = 0; k + 1 < dl.path.size(); ++k) {
                Cost w = INF; for (auto e = g.begin(dl.path[k]); e != g.end(dl.path[k]); ++e) if (e->to == dl.path[k + 1]) w = std::min(w, e->w[LENGTH]);
                tt = P.arrive_departure_speed(tt, w / 1000.0);
            }
            std::fprintf(fs, "%s,%.2f,%.4f,%.4f,%.4f\n", platform.c_str(), t0, o, l, tt - t0);
        }
        std::fclose(fs);
    }
    }
    std::printf("sequential experiments written to %s/\n", out.c_str());
}
