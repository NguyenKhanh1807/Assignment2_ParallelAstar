// test_seq.cpp - correctness tests for Part 1 (sequential A* family).
#include <cassert>
#include <cstdio>
#include "../src/seq_astar.hpp"
using namespace hcm;

static int fails = 0;
#define CHECK(c, ...) do { if (!(c)) { fails++; std::printf("  FAIL: " __VA_ARGS__); std::printf("\n"); } } while (0)

static bool valid_path(const Graph& g, const Result& r, int s, int t, Weight W) {
    if (r.path.empty() || r.path.front() != s || r.path.back() != t) return false;
    Cost c = 0;
    for (size_t i = 0; i + 1 < r.path.size(); ++i) {
        Cost best = -1;
        for (auto e = g.begin(r.path[i]); e != g.end(r.path[i]); ++e)
            if (e->to == r.path[i + 1] && (best < 0 || W(*e) < best)) best = W(*e);
        if (best < 0) return false;
        c += best;
    }
    return c == r.cost;
}

int main(int argc, char** argv) {
    std::string dir = argc > 1 ? argv[1] : "data";
    Graph g = Graph::load(dir);
    std::printf("graph: %d nodes, %zu edges\n", g.n, g.adj.size());
    std::mt19937 rng(2026);
    std::uniform_int_distribution<int> U(0, g.n - 1);

    // T1: admissibility/consistency of the straight-line heuristic on EVERY edge (several targets)
    std::printf("[T1] consistency of Euclid heuristic on all edges\n");
    for (int k = 0; k < 5; ++k) {
        int t = U(rng);
        for (int m = 0; m < NUM_METRICS; ++m) {
            EuclidH h{&g, t, m}; long bad = 0;
            for (int u = 0; u < g.n; ++u)
                for (auto e = g.begin(u); e != g.end(u); ++e)
                    if (h(u) > e->w[m] + h(e->to)) bad++;
            CHECK(bad == 0, "metric %d target %d: %ld inconsistent edges", m, t, bad);
        }
    }
    // T2: ALT consistency + admissibility against exact distances
    std::printf("[T2] ALT landmarks: consistency & admissibility\n");
    Landmarks lm = Landmarks::build(g, 8, LENGTH);
    for (int k = 0; k < 3; ++k) {
        int t = U(rng); AltH h{&lm, &g, t};
        auto dt = sssp(g, t, LENGTH, true);  // exact d(v,t)
        long bad = 0, inadm = 0;
        for (int u = 0; u < g.n; ++u) {
            if (h(u) > dt[u]) inadm++;
            for (auto e = g.begin(u); e != g.end(u); ++e) if (h(u) > e->w[LENGTH] + h(e->to)) bad++;
        }
        CHECK(bad == 0 && inadm == 0, "ALT target %d: %ld inconsistent, %ld inadmissible", t, bad, inadm);
    }
    // T3: A* (Euclid, ALT) == Dijkstra on random pairs, all metrics
    const int PAIRS = 300;
    std::printf("[T3] A* vs Dijkstra on %d random pairs x %d metrics\n", PAIRS, NUM_METRICS);
    long exp_d = 0, exp_a = 0, exp_alt = 0;
    for (int i = 0; i < PAIRS; ++i) {
        int s = U(rng), t = U(rng);
        for (int m = 0; m < NUM_METRICS; ++m) {
            Weight W{m};
            auto d = dijkstra(g, s, t, W);
            EuclidH h{&g, t, m};
            auto a = astar(g, s, t, h, W);
            CHECK(a.cost == d.cost, "pair %d->%d metric %d: A* %lld != Dijkstra %lld", s, t, m, (long long)a.cost, (long long)d.cost);
            CHECK(valid_path(g, a, s, t, W), "pair %d->%d metric %d: invalid A* path", s, t, m);
            CHECK(a.reopened == 0, "consistent h must never re-open (got %ld)", a.reopened);
            if (m == LENGTH) {
                AltH ha{&lm, &g, t};
                auto b = astar(g, s, t, ha, W);
                CHECK(b.cost == d.cost && valid_path(g, b, s, t, W), "ALT mismatch %d->%d", s, t);
                exp_d += d.expanded; exp_a += a.expanded; exp_alt += b.expanded;
            }
        }
    }
    std::printf("     avg expansions (length): Dijkstra %.0f, A* %.0f, ALT %.0f\n",
                exp_d / (double)PAIRS, exp_a / (double)PAIRS, exp_alt / (double)PAIRS);
    // T4: uncertain & dynamic heuristics respect the (1+eps) bound and return valid paths
    std::printf("[T4] uncertain / dynamic heuristics: optimal <= cost <= (1+eps)*optimal\n");
    for (double eps : {0.0, 0.1, 0.3, 0.6}) {
        double worst_u = 1, worst_d = 1;
        for (int i = 0; i < 100; ++i) {
            int s = U(rng), t = U(rng); Weight W{TIME};
            auto d = dijkstra(g, s, t, W);
            UncertainH hu(&g, t, TIME, eps, 1000 + i);
            auto a = astar(g, s, t, hu, W);
            DynamicH hd{&g, t, TIME, eps, 500};
            auto b = astar(g, s, t, hd, W);
            for (auto* r : {&a, &b}) {
                CHECK(valid_path(g, *r, s, t, W), "invalid path eps=%.2f", eps);
                CHECK(r->cost >= d.cost && r->cost <= (Cost)std::ceil((1 + eps) * d.cost) + 1,
                      "bound violated eps=%.2f cost=%lld opt=%lld", eps, (long long)r->cost, (long long)d.cost);
            }
            if (d.cost > 0) { worst_u = std::max(worst_u, a.cost / (double)d.cost); worst_d = std::max(worst_d, b.cost / (double)d.cost); }
            if (eps == 0.0) CHECK(a.cost == d.cost && b.cost == d.cost, "eps=0 must be optimal");
        }
        std::printf("     eps=%.1f  worst ratio: uncertain %.4f, dynamic %.4f\n", eps, worst_u, worst_d);
    }
    // T5: NAMOA* (vector f) vs exhaustive multi-objective label-setting (h = 0) and scalarisations
    std::printf("[T5] NAMOA* Pareto front (length, congested time)\n");
    int tested = 0;
    while (tested < 25) {
        int s = U(rng), t = U(rng);
        if (g.dist_m(s, t) > 2500) continue;  // keep exhaustive baseline affordable
        EuclidH h1{&g, t, LENGTH}, h2{&g, t, CONGESTED};
        auto P = namoa(g, s, t, LENGTH, CONGESTED, h1, h2);
        ZeroH z1, z2;
        auto Q = namoa(g, s, t, LENGTH, CONGESTED, z1, z2);
        CHECK(P.front.size() == Q.front.size(), "front size %zu vs %zu", P.front.size(), Q.front.size());
        for (size_t i = 0; i < std::min(P.front.size(), Q.front.size()); ++i)
            CHECK(P.front[i].a == Q.front[i].a && P.front[i].b == Q.front[i].b, "front point %zu differs", i);
        for (size_t i = 0; i < P.front.size(); ++i) {
            for (size_t j = 0; j < P.front.size(); ++j) CHECK(i == j || !weakly_dom(P.front[j], P.front[i]), "front not non-dominated");
            CHECK(g.path_cost(P.paths[i], LENGTH) == P.front[i].a && g.path_cost(P.paths[i], CONGESTED) == P.front[i].b, "path/vector mismatch");
        }
        auto d1 = dijkstra(g, s, t, Weight{LENGTH}), d2 = dijkstra(g, s, t, Weight{CONGESTED});
        CHECK(P.front.front().a == d1.cost && P.front.back().b == d2.cost, "extreme points != single-objective optima");
        for (int a = 1; a <= 4; ++a) for (int b = 1; b <= 4; ++b) {  // supported solutions
            Weight W{LENGTH, (double)a, (double)b, CONGESTED};
            auto ws = dijkstra(g, s, t, W);
            Cost best = INF;
            for (auto& v : P.front) best = std::min(best, a * v.a + b * v.b);
            CHECK(best == ws.cost, "weighted sum (%d,%d): front %lld vs Dijkstra %lld", a, b, (long long)best, (long long)ws.cost);
        }
        tested++;
    }
    // T6: time-dependent A* with the lecturer's speed profile
    std::printf("[T6] time-dependent A* (lecturer's speed profile)\n");
    {
        SpeedProfile P;
        std::uniform_real_distribution<double> Ut(0.0, 40.0), UL(0.0, 20000.0);
        long nonfifo = 0, nonfifo_dep = 0;
        for (int i = 0; i < 200000; ++i) {   // FIFO: leaving later never arrives earlier
            double t = Ut(rng), L = UL(rng), dt = 0.01 + 0.5 * Ut(rng) / 40.0;
            if (P.arrive(t + dt, L) < P.arrive(t, L) - 1e-9) nonfifo++;
            if (P.arrive_departure_speed(t + dt, L) < P.arrive_departure_speed(t, L) - 1e-9) nonfifo_dep++;
        }
        CHECK(nonfifo == 0, "integrated model violates FIFO %ld times", nonfifo);
        std::printf("     FIFO violations: integrated model %ld, departure-speed model (original notebook) %ld of 200000\n", nonfifo, nonfifo_dep);
        double vmax = P.vmax(); long worse = 0;
        for (int i = 0; i < 100; ++i) {
            int s = U(rng), t = U(rng); double t0 = Ut(rng);
            // oracle: speed is the same on every road at any moment and travel is FIFO, so the fastest
            // route is the shortest-length route and its duration is arrive(t0, L*) - t0
            auto d = dijkstra(g, s, t, Weight{LENGTH});
            double oracle = P.arrive(t0, d.cost / 1000.0) - t0;
            auto ha = [&](int v, double) { return g.dist_m(v, t) / vmax; };
            auto hz = [&](int, double) { return 0.0; };
            auto hl = [&](int v, double tn) { return g.dist_m(v, t) / P.speed(tn); };   // lecturer's h
            auto a = td_astar(g, s, t, t0, P, ha), z = td_astar(g, s, t, t0, P, hz), l = td_astar(g, s, t, t0, P, hl);
            CHECK(std::fabs(a.minutes - oracle) < 1e-6 && std::fabs(z.minutes - oracle) < 1e-6,
                  "TD-A* %.9f / TD-Dijkstra %.9f != oracle %.9f", a.minutes, z.minutes, oracle);
            CHECK(!a.path.empty() && a.path.front() == s && a.path.back() == t, "TD-A* invalid path");
            CHECK(a.expanded <= z.expanded, "admissible h expanded more than h = 0");
            CHECK(l.minutes >= oracle - 1e-6, "lecturer's h returned less than optimum");
            if (l.minutes > oracle + 1e-6) worse++;
        }
        std::printf("     lecturer's h = d/speed(t): sub-optimal on %ld of 100 queries\n", worse);
    }
    std::printf("\n%s (%d failures)\n", fails ? "SEQUENTIAL TESTS FAILED" : "ALL SEQUENTIAL TESTS PASSED", fails);
    return fails ? 1 : 0;
}
