// par_astar.hpp - Part 3: parallel A* on shared memory.
//
// Adaptations required to merge sequential A* (Part 1) with the concurrent sorted list (Part 2):
//  (1) Open list = concurrent sorted SET keyed by a unique 64-bit key  (f << 24) | node.
//      Equal f-values of different nodes stay distinct; a node re-inserted with the same f
//      is a harmless duplicate (add() returns false). removeMin() was added to every list.
//  (2) No CLOSED list: g-values live in an array of atomics (g and parent packed in one
//      64-bit word) and are lowered with a CAS loop. A node is (re-)inserted whenever its g
//      improves, so a node can be expanded more than once (counted as search overhead).
//  (3) Stale entries: a popped (f,v) with f > g[v]+h(v) is outdated and skipped.
//  (4) The first path reaching the goal is NOT necessarily optimal (other threads may still
//      hold cheaper nodes). An incumbent cost U is kept; entries with f >= U are pruned.
//  (5) Termination: a counter `pending` holds the number of outstanding work items = entries
//      in the open list + entries a thread has popped and is still processing. It is raised
//      BEFORE children are inserted and lowered only after the popped entry is finished, so
//      it can only reach 0 when no work exists and none can appear. Then U is optimal for a
//      consistent, admissible h.
// Three parallel designs are provided:
//  * par_astar_shared<ListPQ<List>>  - shared concurrent sorted list (4 list variants)
//  * par_astar_shared<HeapPQ<Lock>>  - shared binary heap behind one lock (baseline)
//  * hda_star                        - Hash-Distributed A*: per-thread heaps, nodes owned by
//                                      hash(node) % T, generated nodes sent via mailboxes.
#pragma once
#include <atomic>
#include <condition_variable>
#include <functional>
#include <map>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <thread>
#ifdef __linux__
#include <fstream>
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#endif
#ifdef _WIN32
#include <windows.h>
#include <psapi.h>
#else
#include <sys/resource.h>
#endif
#include "clist.hpp"
#include "seq_astar.hpp"

namespace hcm {

// ------------------------------------------------------------------ benchmark stats & RSS
struct Stats { double median, mean, std, min, max, iqr; };
inline double percentile_sorted(const std::vector<double>& s, double p) {
    if (s.empty()) return 0.0;
    double idx = p * (double)(s.size() - 1);
    size_t lo = (size_t)idx, hi = std::min(s.size() - 1, lo + 1);
    double frac = idx - (double)lo;
    return s[lo] * (1.0 - frac) + s[hi] * frac;
}
inline Stats compute_stats(std::vector<double> v) {
    if (v.empty()) return {};
    std::sort(v.begin(), v.end());
    double sum = 0; for (double x : v) sum += x;
    double mean = sum / (double)v.size(), var = 0;
    for (double x : v) var += (x - mean) * (x - mean);
    double sd = v.size() > 1 ? std::sqrt(var / (double)(v.size() - 1)) : 0.0;
    return {percentile_sorted(v, 0.5), mean, sd, v.front(), v.back(),
            percentile_sorted(v, 0.75) - percentile_sorted(v, 0.25)};
}
#ifdef _WIN32
inline long max_rss_kb() {
    PROCESS_MEMORY_COUNTERS pmc{};
    if (!GetProcessMemoryInfo(GetCurrentProcess(), &pmc, sizeof(pmc))) return 0;
    return (long)(pmc.PeakWorkingSetSize / 1024);
}
#else
inline long max_rss_kb() {
    struct rusage ru{};
    if (getrusage(RUSAGE_SELF, &ru) != 0) return 0;
#ifdef __APPLE__
    return (long)(ru.ru_maxrss / 1024);
#else
    return (long)ru.ru_maxrss;
#endif
}
#endif
inline long current_rss_kb() {
#ifdef __linux__
    long sz = 0, res = 0;
    std::ifstream f("/proc/self/statm");
    if (f >> sz >> res) return res * (sysconf(_SC_PAGESIZE) / 1024);
#endif
    return max_rss_kb();
}

constexpr int PBITS = 24;                       // parent / node id bits
constexpr uint64_t PMASK = (1ull << PBITS) - 1;
constexpr uint64_t GINF = (1ull << (64 - PBITS)) - 1;
inline uint64_t pack(uint64_t g, uint64_t p) { return (g << PBITS) | (p & PMASK); }
inline uint64_t unpack_g(uint64_t x) { return x >> PBITS; }
inline int unpack_p(uint64_t x) { uint64_t p = x & PMASK; return p == PMASK ? -1 : (int)p; }
inline clist::Key mkkey(Cost f, int v) { return (clist::Key)(((uint64_t)f << PBITS) | (uint64_t)v); }
inline Cost key_f(clist::Key k) { return (Cost)((uint64_t)k >> PBITS); }
inline int key_v(clist::Key k) { return (int)((uint64_t)k & PMASK); }

struct ParResult : Result { int threads = 1; long pushes = 0; long retired_bytes = 0; };

// ------------------------------------------------------------------ CPU affinity & thread pool
#ifdef __linux__
// Orders allowed CPUs so distinct physical cores come before SMT siblings.
inline const std::vector<int>& physical_first_cpus() {
    static const std::vector<int> order = [] {
        cpu_set_t mask; CPU_ZERO(&mask);
        std::vector<int> avail;
        if (pthread_getaffinity_np(pthread_self(), sizeof(mask), &mask) == 0) {
            for (int c = 0; c < CPU_SETSIZE; ++c) if (CPU_ISSET(c, &mask)) avail.push_back(c);
        }
        if (avail.empty()) {
            int n = (int)std::thread::hardware_concurrency();
            for (int c = 0; c < std::max(1, n); ++c) avail.push_back(c);
        }
        std::vector<int> primary, smt;
        std::map<std::pair<int, int>, bool> seen;
        for (int c : avail) {
            int pkg = 0, core = c;
            std::ifstream fp("/sys/devices/system/cpu/cpu" + std::to_string(c) + "/topology/physical_package_id");
            std::ifstream fc("/sys/devices/system/cpu/cpu" + std::to_string(c) + "/topology/core_id");
            fp >> pkg; fc >> core;
            if (!seen[{pkg, core}]) { seen[{pkg, core}] = true; primary.push_back(c); }
            else smt.push_back(c);
        }
        primary.insert(primary.end(), smt.begin(), smt.end());
        return primary;
    }();
    return order;
}
struct AffinityGuard {
    bool active = false;
    cpu_set_t prev{};
    AffinityGuard(bool enable, int worker_id, bool restore = true) {
        if (!enable) return;
        const auto& cpus = physical_first_cpus();
        if (cpus.empty()) return;
        if (restore) active = (pthread_getaffinity_np(pthread_self(), sizeof(prev), &prev) == 0);
        cpu_set_t s; CPU_ZERO(&s);
        CPU_SET(cpus[worker_id % (int)cpus.size()], &s);
        pthread_setaffinity_np(pthread_self(), sizeof(s), &s);
    }
    ~AffinityGuard() { if (active) pthread_setaffinity_np(pthread_self(), sizeof(prev), &prev); }
};
#else
struct AffinityGuard { AffinityGuard(bool, int, bool = true) {} };
#endif

// Reusable worker pool: thread 0 executes on the caller; workers 1..T-1 wait on a CV.
struct ThreadPool {
    int T;
    bool pin;
    std::mutex mu;
    std::condition_variable cv_start, cv_done;
    std::function<void(int)> job;
    uint64_t gen = 0;
    int pending_workers = 0;
    bool stop = false;
    std::vector<std::thread> workers;

    explicit ThreadPool(int threads, bool pin_cores = false) : T(std::max(1, threads)), pin(pin_cores) {
        for (int i = 1; i < T; ++i) {
            workers.emplace_back([this, i] {
                AffinityGuard ag(pin, i, false);
                uint64_t seen = 0;
                for (;;) {
                    std::function<void(int)> fn;
                    {
                        std::unique_lock<std::mutex> lk(mu);
                        cv_start.wait(lk, [&] { return stop || gen != seen; });
                        if (stop) return;
                        seen = gen;
                        fn = job;
                    }
                    fn(i);
                    std::lock_guard<std::mutex> lk(mu);
                    if (--pending_workers == 0) cv_done.notify_one();
                }
            });
        }
    }
    ~ThreadPool() {
        { std::lock_guard<std::mutex> lk(mu); stop = true; gen++; }
        cv_start.notify_all();
        for (auto& w : workers) w.join();
    }
    ThreadPool(const ThreadPool&) = delete;
    int size() const { return T; }
    template <class F>
    void run(F&& fn) {
        AffinityGuard ag(pin, 0, true);
        if (T <= 1) { fn(0); return; }
        {
            std::lock_guard<std::mutex> lk(mu);
            job = [&](int id) { fn(id); };
            pending_workers = T - 1;
            gen++;
        }
        cv_start.notify_all();
        fn(0);
        std::unique_lock<std::mutex> lk(mu);
        cv_done.wait(lk, [&] { return pending_workers == 0; });
    }
};

template <class F>
inline void run_parallel(int T, ThreadPool* pool, bool pin, F&& fn) {
    T = std::max(1, T);
    if (pool) {
        if (pool->size() != T) throw std::invalid_argument("ThreadPool size does not match thread count T");
        pool->run(std::forward<F>(fn));
        return;
    }
    AffinityGuard ag(pin, 0, true);
    std::vector<std::thread> th;
    th.reserve(std::max(0, T - 1));
    for (int i = 1; i < T; ++i) th.emplace_back([&, i] { AffinityGuard wg(pin, i, false); fn(i); });
    fn(0);
    for (auto& x : th) x.join();
}

// ------------------------------------------------------------------ priority-queue adaptors
template <class List>
struct ListPQ {
    List l;
    static std::string name() { return "list-" + List::name(); }
    bool push(clist::Key k) { return l.add(k); }   // false: duplicate key, nothing added
    bool pop(clist::Key& k) { return l.removeMin(k); }
    bool empty() { return l.isEmpty(); }
    long retired_bytes() const { return l.retired_count() * (long)List::node_bytes(); }
};
template <class Lock>
struct HeapPQ {
    Lock L;
    std::priority_queue<clist::Key, std::vector<clist::Key>, std::greater<clist::Key>> q;
    static std::string name() { return std::string("heap-") + Lock::name(); }
    bool push(clist::Key k) { std::lock_guard<Lock> g(L); q.push(k); return true; }
    bool pop(clist::Key& k) { std::lock_guard<Lock> g(L); if (q.empty()) return false; k = q.top(); q.pop(); return true; }
    bool empty() { std::lock_guard<Lock> g(L); return q.empty(); }
    long retired_bytes() const { return 0; }
};

inline bool cas_min_g(std::atomic<uint64_t>& a, uint64_t ng, int parent) {
    uint64_t cur = a.load(std::memory_order_acquire);
    while (unpack_g(cur) > ng)
        if (a.compare_exchange_weak(cur, pack(ng, (uint64_t)parent), std::memory_order_acq_rel)) return true;
    return false;
}
inline void atomic_min(std::atomic<Cost>& a, Cost v) {
    Cost cur = a.load();
    while (v < cur && !a.compare_exchange_weak(cur, v)) {}
}

// ------------------------------------------------------------------ shared open list A*
template <class PQ, class H>
ParResult par_astar_shared(const Graph& g, int s, int t, const H& h0, int T, int metric = LENGTH,
                           ThreadPool* pool = nullptr, bool pin = false) {
    double t0 = now_ms();
    auto hcache = make_hcache(g.n);
    MemoH<H> h{&h0, &hcache};
    ParResult R; R.threads = T;
    std::vector<std::atomic<uint64_t>> G(g.n);
    for (auto& x : G) x.store(pack(GINF, PMASK), std::memory_order_relaxed);
    PQ open;
    std::atomic<Cost> U{INF};
    std::atomic<long> pending{1};      // outstanding work items: open entries + entries being processed
    std::atomic<bool> done{false};
    std::atomic<long> expanded{0}, pushes{0};
    G[s].store(pack(0, PMASK)); open.push(mkkey(h(s), s));   // the one initial work item (pending == 1)
    if (s == t) U = 0;
    auto worker = [&](int) {
        clist::Backoff bo(1, 1024);
        long my_exp = 0, my_push = 0;
        while (!done.load(std::memory_order_acquire)) {
            clist::Key k;
            if (!open.pop(k)) {
                // pending == 0: the open list is empty AND no thread holds a popped entry, so no
                // thread can insert anything any more. (Testing "open empty, then no thread busy"
                // with two separate reads is not enough: between the two reads another thread can
                // insert its children and finish, and the search would stop with work left.)
                if (pending.load(std::memory_order_acquire) == 0) { done.store(true); break; }
                bo.backoff();
                std::this_thread::yield();   // idle: give the CPU to threads that hold work (oversubscription)
                continue;
            }
            bo = clist::Backoff(1, 1024);
            Cost f = key_f(k); int u = key_v(k);
            Cost gu = (Cost)unpack_g(G[u].load(std::memory_order_acquire));
            if (f > gu + h(u) || f >= U.load(std::memory_order_acquire)) { pending.fetch_sub(1); continue; }  // stale / pruned
            my_exp++;
            if (u == t) { atomic_min(U, gu); pending.fetch_sub(1); continue; }
            // Reserve one work item per possible child BEFORE inserting any of them, so that
            // pending never under-counts; the unused reservations and the popped entry itself
            // are released together once all children are in the open list.
            long reserved = g.degree(u), added = 0;
            pending.fetch_add(reserved);
            for (auto e = g.begin(u); e != g.end(u); ++e) {
                int v = e->to; Cost ng = gu + e->w[metric];
                if (cas_min_g(G[v], (uint64_t)ng, u)) {
                    if (v == t) atomic_min(U, ng);
                    Cost fv = ng + h(v);
                    if (fv < U.load(std::memory_order_acquire)) { if (open.push(mkkey(fv, v))) added++; my_push++; }
                }
            }
            pending.fetch_sub(reserved - added + 1);
        }
        expanded += my_exp; pushes += my_push;
    };
    run_parallel(T, pool, pin, worker);
    R.expanded = expanded; R.pushes = pushes; R.retired_bytes = open.retired_bytes();
    Cost gt = (Cost)unpack_g(G[t].load());
    if (gt < (Cost)GINF) {
        R.cost = gt;
        std::vector<int> par(g.n, -1);
        for (int v = t, guard = 0; v != -1 && v != s && guard < g.n; v = par[v], ++guard) par[v] = unpack_p(G[v].load());
        R.path = build_path(par, s, t);
    }
    R.ms = now_ms() - t0;
    return R;
}

// ------------------------------------------------------------------ HDA*
// window < 0 : plain HDA*. window >= 0 : a thread only expands its local best node if
// f <= fmin * (1 + window), fmin = smallest f published by any thread. This keeps threads
// roughly synchronised on the global f-order and avoids exploring far beyond the optimal
// f-layer when one owner thread is slow (e.g. descheduled when threads > cores).
template <class H>
ParResult hda_star(const Graph& g, int s, int t, const H& h0, int T, int metric = LENGTH, double window = -1.0,
                   ThreadPool* pool = nullptr, bool pin = false) {
    double t0 = now_ms();
    auto hcache = make_hcache(g.n);
    MemoH<H> h{&h0, &hcache};
    ParResult R; R.threads = T;
    struct Msg { int v; Cost gv; int p; };
    struct alignas(64) Mailbox { std::mutex m; std::vector<Msg> in; };
    std::vector<Mailbox> box(T);
    std::vector<Cost> G(g.n, INF); std::vector<int> par(g.n, -1);   // written only by the owner thread
    auto owner = [T](int v) { return (int)(((uint32_t)v * 2654435761u) >> 7) % T; };
    std::atomic<Cost> U{INF};
    std::atomic<long> W{1};            // outstanding work items = heap entries + messages in flight
    std::atomic<long> expanded{0}, pushes{0};
    std::atomic<bool> done{false};
    struct alignas(64) Pub { std::atomic<Cost> f{INF}; };
    std::vector<Pub> pub(T);
    if (s == t) U = 0;
    { std::lock_guard<std::mutex> l(box[owner(s)].m); box[owner(s)].in.push_back({s, 0, -1}); }
    auto worker = [&](int id) {
        using P = std::pair<Cost, int>;
        std::priority_queue<P, std::vector<P>, std::greater<P>> open;
        std::vector<std::vector<Msg>> out(T);
        std::vector<Msg> inbuf;
        clist::Backoff bo(1, 1024);
        long my_exp = 0, my_push = 0;
        auto accept = [&](int v, Cost gv, int p) -> bool {   // owner-local relaxation
            if (gv >= G[v]) return false;
            G[v] = gv; par[v] = p;
            if (v == t) atomic_min(U, gv);
            open.push({gv + h(v), v}); my_push++;
            return true;
        };
        while (!done.load(std::memory_order_acquire)) {
            { std::lock_guard<std::mutex> l(box[id].m); inbuf.swap(box[id].in); }
            for (auto& m : inbuf) if (!accept(m.v, m.gv, m.p)) W.fetch_sub(1);  // accepted msg becomes a heap entry
            inbuf.clear();
            if (open.empty()) {
                if (window >= 0) pub[id].f.store(INF, std::memory_order_relaxed);
                if (W.load() == 0) { done.store(true); break; }
                bo.backoff();
                std::this_thread::yield();   // idle: give the CPU to threads that hold work (oversubscription)
                continue;
            }
            if (window >= 0) {
                Cost top = open.top().first, fmin = top;
                pub[id].f.store(top, std::memory_order_relaxed);
                for (int i = 0; i < T; ++i) fmin = std::min(fmin, pub[i].f.load(std::memory_order_relaxed));
                if (top > fmin + (Cost)(fmin * window) && top < U.load(std::memory_order_relaxed)) {
                    std::this_thread::yield();   // too far ahead of the global f-frontier: wait
                    continue;
                }
            }
            bo = clist::Backoff(1, 1024);
            auto [f, u] = open.top(); open.pop();
            if (f >= U.load(std::memory_order_acquire)) {          // everything left here is pruned
                W.fetch_sub(1 + (long)open.size());
                open = decltype(open)();
                continue;
            }
            if (f > G[u] + h(u) || u == t) { W.fetch_sub(1); continue; }  // stale / goal
            my_exp++;
            for (auto e = g.begin(u); e != g.end(u); ++e) {
                int v = e->to; Cost ng = G[u] + e->w[metric];
                if (ng + h(v) >= U.load(std::memory_order_relaxed)) continue;
                int o = owner(v);
                if (o == id) { if (accept(v, ng, u)) W.fetch_add(1); }
                else { W.fetch_add(1); out[o].push_back({v, ng, u}); }
            }
            for (int o = 0; o < T; ++o) if (!out[o].empty()) {
                std::lock_guard<std::mutex> l(box[o].m);
                box[o].in.insert(box[o].in.end(), out[o].begin(), out[o].end());
                out[o].clear();
            }
            W.fetch_sub(1);  // the expanded entry (children already accounted for)
        }
        expanded += my_exp; pushes += my_push;
    };
    run_parallel(T, pool, pin, worker);
    R.expanded = expanded; R.pushes = pushes;
    if (G[t] < INF) { R.cost = G[t]; R.path = build_path(par, s, t); }
    R.ms = now_ms() - t0;
    return R;
}

}  // namespace hcm
