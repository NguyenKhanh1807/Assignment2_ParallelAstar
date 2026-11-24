// seq_astar.hpp - Part 1: sequential A* family on the HCMC road graph.
//  * dijkstra()            : exact oracle used by all correctness tests
//  * astar()               : generic A* (binary heap, re-opening, lazy re-keying for dynamic h)
//  * EuclidH / UncertainH / DynamicH / AltH : static, uncertain, dynamic and landmark heuristics
//  * namoa()               : multi-objective A* (vector-valued f), returns the Pareto front
#pragma once
#include <chrono>
#include <queue>
#include <random>
#include <functional>
#include <atomic>
#include "graph.hpp"

namespace hcm {

struct Result {
    Cost cost = INF;
    std::vector<int> path;
    long expanded = 0, generated = 0, reopened = 0, stale = 0;
    double ms = 0;
};

inline double now_ms() {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

inline std::vector<int> build_path(const std::vector<int>& parent, int s, int t) {
    std::vector<int> p;
    for (int v = t; v != -1; v = parent[v]) { p.push_back(v); if (v == s) break; if ((int)p.size() > (int)parent.size()) return {}; }
    std::reverse(p.begin(), p.end());
    return (p.empty() || p.front() != s) ? std::vector<int>{} : p;
}

// Edge-weight accessor; a weighted sum of two metrics is used for scalarisation tests.
struct Weight {
    int m = LENGTH; double a = 1.0, b = 0.0; int m2 = TIME;
    Cost operator()(const Edge& e) const {
        return b == 0.0 ? e.w[m] : (Cost)std::llround(a * e.w[m] + b * e.w[m2]);
    }
};

// ---------------------------------------------------------------- Dijkstra (oracle)
inline Result dijkstra(const Graph& g, int s, int t, Weight W = {}) {
    double t0 = now_ms();
    Result r;
    std::vector<Cost> d(g.n, INF); std::vector<int> par(g.n, -1);
    using P = std::pair<Cost, int>;
    std::priority_queue<P, std::vector<P>, std::greater<P>> pq;
    d[s] = 0; pq.push({0, s});
    while (!pq.empty()) {
        auto [du, u] = pq.top(); pq.pop();
        if (du != d[u]) continue;
        r.expanded++;
        if (u == t) break;
        for (auto e = g.begin(u); e != g.end(u); ++e) {
            Cost nd = du + W(*e);
            if (nd < d[e->to]) { d[e->to] = nd; par[e->to] = u; pq.push({nd, e->to}); r.generated++; }
        }
    }
    if (t >= 0 && d[t] < INF) { r.cost = d[t]; r.path = build_path(par, s, t); }
    r.ms = now_ms() - t0;
    return r;
}

// Single-source distances (forward or on the reverse graph) - used by ALT preprocessing.
inline std::vector<Cost> sssp(const Graph& g, int s, int m, bool reverse) {
    std::vector<Cost> d(g.n, INF);
    using P = std::pair<Cost, int>;
    std::priority_queue<P, std::vector<P>, std::greater<P>> pq;
    d[s] = 0; pq.push({0, s});
    while (!pq.empty()) {
        auto [du, u] = pq.top(); pq.pop();
        if (du != d[u]) continue;
        const Edge* b = reverse ? g.rbegin(u) : g.begin(u);
        const Edge* e2 = reverse ? g.rend(u) : g.end(u);
        for (auto e = b; e != e2; ++e)
            if (du + e->w[m] < d[e->to]) { d[e->to] = du + e->w[m]; pq.push({d[e->to], e->to}); }
    }
    return d;
}

// ---------------------------------------------------------------- heuristics
// Every heuristic exposes: Cost operator()(int v)  and  void tick() (called once per expansion).
struct ZeroH { Cost operator()(int) const { return 0; } void tick() {} static constexpr bool dynamic = false; };

struct EuclidH {  // static, admissible, consistent
    const Graph* g; int t; int m;
    Cost operator()(int v) const { return g->lower_bound(v, t, m); }
    void tick() {}
    static constexpr bool dynamic = false;
};

// Uncertain heuristic: the straight-line estimate is perturbed by a fresh random error
// in [-eps, +eps] (relative) at every evaluation, e.g. a noisy travel-time predictor.
// Because h <= (1+eps) h*, A* with re-opening returns a path with cost <= (1+eps) C*.
struct UncertainH {
    const Graph* g; int t; int m; double eps; std::mt19937_64 rng;
    UncertainH(const Graph* g_, int t_, int m_, double e, uint64_t seed) : g(g_), t(t_), m(m_), eps(e), rng(seed) {}
    Cost operator()(int v) {
        std::uniform_real_distribution<double> U(-1.0, 1.0);
        return (Cost)std::llround(g->lower_bound(v, t, m) * (1.0 + eps * U(rng)));
    }
    void tick() {}
    static constexpr bool dynamic = true;  // repeated evaluations of the same node can differ
};

// Dynamic heuristic: the estimate is scaled by a factor lambda(t) that changes while the search
// runs (e.g. a traffic forecast refreshed every `period` expansions). lambda in [1-eps, 1+eps].
// Keys stored in the open list become stale; astar() re-evaluates them lazily on pop.
struct DynamicH {
    const Graph* g; int t; int m; double eps; long period; long step = 0;
    double lambda() const { return 1.0 + eps * std::sin(2 * M_PI * (double)step / (double)period); }
    Cost operator()(int v) const { return (Cost)std::llround(g->lower_bound(v, t, m) * lambda()); }
    void tick() { ++step; }
    static constexpr bool dynamic = true;
};

// Expensive heuristic: same value as EuclidH, but each evaluation also performs `work`
// dependent floating-point operations. It stands for a costly heuristic such as a learned
// travel-time predictor queried online, and is used to study when parallel A* pays off.
struct CostlyH {
    EuclidH base; int work;
    Cost operator()(int v) const {
        double x = 1.0 + v * 1e-9;
        for (int i = 0; i < work; ++i) x = x * 0.999999 + 1e-7;
        return base(v) + (x > 1e300 ? 1 : 0);   // never true: keeps the loop from being optimised away
    }
    void tick() {}
    static constexpr bool dynamic = false;
};

// ALT (A*, Landmarks, Triangle inequality) - Part 4 optimisation. Consistent & admissible.
struct Landmarks {
    int m = LENGTH;
    std::vector<int> L;
    std::vector<std::vector<Cost>> from, to;  // from[i][v] = d(L_i, v), to[i][v] = d(v, L_i)
    static Landmarks build(const Graph& g, int k, int m, int seed = 1) {
        Landmarks lm; lm.m = m;
        std::vector<Cost> mind(g.n, INF);
        int cur = seed % g.n;
        // farthest-point selection on the undirected-ish (forward) distance
        auto d0 = sssp(g, cur, m, false);
        cur = (int)(std::max_element(d0.begin(), d0.end()) - d0.begin());
        for (int i = 0; i < k; ++i) {
            lm.L.push_back(cur);
            lm.from.push_back(sssp(g, cur, m, false));
            lm.to.push_back(sssp(g, cur, m, true));
            for (int v = 0; v < g.n; ++v) mind[v] = std::min(mind[v], lm.from.back()[v] + lm.to.back()[v]);
            cur = (int)(std::max_element(mind.begin(), mind.end()) - mind.begin());
        }
        return lm;
    }
};
struct AltH {
    const Landmarks* lm; const Graph* g; int t;
    Cost operator()(int v) const {
        Cost h = g->lower_bound(v, t, lm->m);
        for (size_t i = 0; i < lm->L.size(); ++i) {
            Cost a = lm->from[i][t] - lm->from[i][v];   // d(v,t) >= d(L,t) - d(L,v)
            Cost b = lm->to[i][v] - lm->to[i][t];       // d(v,t) >= d(v,L) - d(t,L)
            h = std::max(h, std::max(a, b));
        }
        return h;
    }
    void tick() {}
    static constexpr bool dynamic = false;
};

// Memoised wrapper for a static heuristic: h(v) is evaluated at most once per node per query
// (thread-safe: benign races resolved with relaxed atomics, h is deterministic).
template <class H>
struct MemoH {
    const H* h; std::vector<std::atomic<Cost>>* cache;
    Cost operator()(int v) const {
        Cost c = (*cache)[v].load(std::memory_order_relaxed);
        if (c < 0) { c = (*h)(v); (*cache)[v].store(c, std::memory_order_relaxed); }
        return c;
    }
    void tick() {}
    static constexpr bool dynamic = false;
};
inline std::vector<std::atomic<Cost>> make_hcache(int n) {
    std::vector<std::atomic<Cost>> c(n);
    for (auto& x : c) x.store(-1, std::memory_order_relaxed);
    return c;
}

// ---------------------------------------------------------------- generic A*
// Re-opening of closed nodes is allowed, so the algorithm stays correct (with the usual
// bounds) for inconsistent / inadmissible heuristics. For a dynamic heuristic the f-key of
// a popped node is recomputed; if it grew past the next key the node is re-inserted.
template <class H, class WF = Weight>
Result astar_impl(const Graph& g, int s, int t, H& h, WF W);
template <class H, class WF = Weight>
Result astar(const Graph& g, int s, int t, H& h, WF W = WF{}) {
    if constexpr (!H::dynamic) {
        double t0 = now_ms();
        auto cache = make_hcache(g.n);
        MemoH<H> m{&h, &cache};
        Result r = astar_impl(g, s, t, m, W);
        r.ms = now_ms() - t0;
        return r;
    } else return astar_impl(g, s, t, h, W);
}
template <class H, class WF>
Result astar_impl(const Graph& g, int s, int t, H& h, WF W) {
    double t0 = now_ms();
    Result r;
    std::vector<Cost> gs(g.n, INF); std::vector<int> par(g.n, -1);
    std::vector<char> closed(g.n, 0);
    using P = std::pair<Cost, int>;  // (f, v)
    std::priority_queue<P, std::vector<P>, std::greater<P>> open;
    gs[s] = 0; open.push({h(s), s});
    while (!open.empty()) {
        auto [f, u] = open.top(); open.pop();
        if (closed[u]) continue;  // duplicate entry
        if (H::dynamic) {
            Cost fnow = gs[u] + h(u);
            if (fnow > f && !open.empty() && fnow > open.top().first) { open.push({fnow, u}); r.stale++; continue; }
        } else if (f > gs[u] + h(u)) continue;  // outdated duplicate
        closed[u] = 1; r.expanded++; h.tick();
        if (u == t) break;
        for (auto e = g.begin(u); e != g.end(u); ++e) {
            int v = e->to; Cost ng = gs[u] + W(*e);
            if (ng < gs[v]) {
                if (closed[v]) { closed[v] = 0; r.reopened++; }
                gs[v] = ng; par[v] = u; open.push({ng + h(v), v}); r.generated++;
            }
        }
    }
    if (gs[t] < INF) {
        r.path = build_path(par, s, t);
        // With an inconsistent h a re-opened ancestor may have improved after t was reached;
        // the walk given by the parent pointers is then cheaper than gs[t]: report its true cost.
        r.cost = 0;
        for (size_t i = 0; i + 1 < r.path.size(); ++i) {
            Cost best = INF;
            for (auto e = g.begin(r.path[i]); e != g.end(r.path[i]); ++e)
                if (e->to == r.path[i + 1]) best = std::min(best, W(*e));
            r.cost += best;
        }
    }
    r.ms = now_ms() - t0;
    return r;
}

// ---------------------------------------------------------------- time-dependent A* (lecturer's model)
// The lecturer's notebook (osm-astar.ipynb) uses one network-wide speed per time slot:
// speeds[k] metres/minute while floor(t / slot) == k (t in minutes). We keep that profile but
// (i) make it cyclic, so trips longer than 20 slots do not run past the array, and
// (ii) integrate the speed over every slot an edge spans. With (ii) travel is FIFO: leaving
// later never means arriving earlier, which label-setting A* needs to be exact.
struct SpeedProfile {
    std::vector<double> v{350, 350, 500, 500, 600, 600, 550, 500, 400, 300,
                          600, 550, 500, 400, 300, 600, 550, 500, 400, 300};
    double slot = 1.0;  // minutes per slot
    double vmax() const { return *std::max_element(v.begin(), v.end()); }
    double speed(double t) const {
        long k = (long)std::floor(t / slot);
        long n = (long)v.size();
        return v[((k % n) + n) % n];
    }
    // arrival time (minutes) after driving L metres from time t
    double arrive(double t, double L) const {
        for (;;) {
            double sp = speed(t);
            double end = (std::floor(t / slot) + 1) * slot;
            double reach = sp * (end - t);
            if (L <= reach) return t + L / sp;
            L -= reach; t = end;
        }
    }
    // the lecturer's edge model: speed at departure only (not FIFO)
    double arrive_departure_speed(double t, double L) const { return t + L / speed(t); }
};

struct TDResult { double minutes = 1e300; std::vector<int> path; long expanded = 0, reopened = 0; double ms = 0; };

// h(v, tnow) -> lower estimate (minutes) of the remaining travel time
template <class HF>
TDResult td_astar(const Graph& g, int s, int t, double t0, const SpeedProfile& P, HF h) {
    double c0 = now_ms();
    TDResult R;
    std::vector<double> arr(g.n, 1e300); std::vector<int> par(g.n, -1); std::vector<char> closed(g.n, 0);
    using Q = std::pair<double, int>;
    std::priority_queue<Q, std::vector<Q>, std::greater<Q>> open;
    arr[s] = t0; open.push({t0 + h(s, t0), s});
    while (!open.empty()) {
        auto [f, u] = open.top(); open.pop();
        if (closed[u]) continue;
        if (f > arr[u] + h(u, arr[u]) + 1e-12) continue;          // outdated entry
        closed[u] = 1; R.expanded++;
        if (u == t) break;
        for (auto e = g.begin(u); e != g.end(u); ++e) {
            int v = e->to;
            double tv = P.arrive(arr[u], e->w[LENGTH] / 1000.0);
            if (tv < arr[v]) {
                if (closed[v]) { closed[v] = 0; R.reopened++; }
                arr[v] = tv; par[v] = u; open.push({tv + h(v, tv), v});
            }
        }
    }
    if (arr[t] < 1e299) { R.minutes = arr[t] - t0; R.path = build_path(par, s, t); }
    R.ms = now_ms() - c0;
    return R;
}

// ---------------------------------------------------------------- NAMOA* (vector f)
// Bi-objective A*: minimises (c1, c2) simultaneously, returns all Pareto-optimal costs + paths.
struct Vec2 { Cost a, b; };
inline bool dominates(const Vec2& x, const Vec2& y) { return x.a <= y.a && x.b <= y.b && (x.a < y.a || x.b < y.b); }
inline bool weakly_dom(const Vec2& x, const Vec2& y) { return x.a <= y.a && x.b <= y.b; }

struct ParetoResult {
    std::vector<Vec2> front;                 // sorted by first objective
    std::vector<std::vector<int>> paths;
    long expanded = 0, generated = 0;
    double ms = 0;
};

template <class H1, class H2>
ParetoResult namoa(const Graph& g, int s, int t, int m1, int m2, H1& h1, H2& h2) {
    double t0 = now_ms();
    struct Label { Vec2 g; int node; int parent; bool alive; };
    std::vector<Label> labels;
    std::vector<std::vector<int>> Gop(g.n), Gcl(g.n);
    struct Key { Cost f1, f2; int id; bool operator>(const Key& o) const { return f1 != o.f1 ? f1 > o.f1 : (f2 != o.f2 ? f2 > o.f2 : id > o.id); } };
    std::priority_queue<Key, std::vector<Key>, std::greater<Key>> open;
    ParetoResult R; std::vector<int> sol;
    labels.push_back({{0, 0}, s, -1, true}); Gop[s].push_back(0);
    open.push({h1(s), h2(s), 0});
    while (!open.empty()) {
        Key k = open.top(); open.pop();
        Label& L = labels[k.id];
        if (!L.alive) continue;
        // move label from Gop to Gcl
        auto& go = Gop[L.node]; go.erase(std::find(go.begin(), go.end(), k.id));
        Gcl[L.node].push_back(k.id);
        bool filtered = false;
        for (int c : sol) if (weakly_dom(labels[c].g, Vec2{k.f1, k.f2}) && c != k.id) { filtered = true; break; }
        if (filtered) continue;
        if (L.node == t) { sol.push_back(k.id); continue; }
        R.expanded++;
        int u = L.node; Vec2 gu = L.g;
        for (auto e = g.begin(u); e != g.end(u); ++e) {
            int v = e->to;
            Vec2 gv{gu.a + e->w[m1], gu.b + e->w[m2]};
            bool skip = false;
            for (int id : Gop[v]) if (weakly_dom(labels[id].g, gv)) { skip = true; break; }
            if (!skip) for (int id : Gcl[v]) if (weakly_dom(labels[id].g, gv)) { skip = true; break; }
            if (skip) continue;
            Vec2 fv{gv.a + h1(v), gv.b + h2(v)};
            for (int c : sol) if (weakly_dom(labels[c].g, fv)) { skip = true; break; }
            if (skip) continue;
            auto prune = [&](std::vector<int>& lst) {
                lst.erase(std::remove_if(lst.begin(), lst.end(), [&](int id) {
                    if (dominates(gv, labels[id].g)) { labels[id].alive = false; return true; } return false; }), lst.end());
            };
            prune(Gop[v]); prune(Gcl[v]);
            int nid = (int)labels.size();
            labels.push_back({gv, v, k.id, true});
            Gop[v].push_back(nid);
            open.push({fv.a, fv.b, nid}); R.generated++;
        }
    }
    std::sort(sol.begin(), sol.end(), [&](int a, int b) { return labels[a].g.a < labels[b].g.a; });
    for (int id : sol) {
        R.front.push_back(labels[id].g);
        std::vector<int> p;
        for (int x = id; x != -1; x = labels[x].parent) p.push_back(labels[x].node);
        std::reverse(p.begin(), p.end());
        R.paths.push_back(p);
    }
    R.ms = now_ms() - t0;
    return R;
}

}  // namespace hcm
