// test_clist.cpp - correctness tests for Part 2 (concurrent sorted-list sets).
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <set>
#include <string>
#include "../src/clist.hpp"
using namespace clist;

static int fails = 0;
#define CHECK(c, ...) do { if (!(c)) { fails++; std::printf("    FAIL: " __VA_ARGS__); std::printf("\n"); } } while (0)

template <class List>
void run_tests(int T) {
    auto t0 = std::chrono::steady_clock::now();
    struct Done { std::chrono::steady_clock::time_point t0; std::string n; int T; ~Done() {
        std::printf("  %-22s threads=%d  %.2fs\n", n.c_str(), T, std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count()); } } done_{t0, List::name(), T};
    // (a) sequential specification vs std::set
    {
        List L; std::set<Key> ref; std::mt19937 rng(1);
        for (int i = 0; i < 20000; ++i) {
            Key k = rng() % 200; int op = rng() % 4;
            if (op == 0) CHECK(L.add(k) == ref.insert(k).second, "seq add");
            else if (op == 1) CHECK(L.remove(k) == (ref.erase(k) == 1), "seq remove");
            else if (op == 2) CHECK(L.contains(k) == (ref.count(k) == 1), "seq contains");
            else { Key m; bool ok = L.removeMin(m); CHECK(ok == !ref.empty() && (!ok || m == *ref.begin()), "seq removeMin");
                   if (ok) ref.erase(ref.begin()); }
        }
        CHECK(L.keys() == std::vector<Key>(ref.begin(), ref.end()), "final contents");
    }
    // (b) disjoint concurrent inserts then removes
    {
        List L; const int PER = 2000; std::vector<std::thread> th;
        for (int t = 0; t < T; ++t) th.emplace_back([&, t] { for (int i = 0; i < PER; ++i) CHECK(L.add((Key)i * T + t), "disjoint add"); });
        for (auto& x : th) x.join();
        th.clear();
        CHECK(L.size() == (long)PER * T && L.check_sorted_unmarked(), "size after disjoint adds: %ld", L.size());
        for (int t = 0; t < T; ++t) th.emplace_back([&, t] { for (int i = 0; i < PER; ++i) CHECK(L.contains((Key)i * T + t) && L.remove((Key)i * T + t), "disjoint rm"); });
        for (auto& x : th) x.join();
        CHECK(L.size() == 0, "not empty after removes");
    }
    // (c) overlapping random ops: per-key net count of successful add/remove must equal final membership
    {
        const int R = 64, OPS = 30000; List L;
        std::vector<int> init(R, 0);
        for (int k = 0; k < R; k += 2) { L.add(k); init[k] = 1; }
        std::vector<std::vector<int>> net(T, std::vector<int>(R, 0));
        // keys >= 1000 are never inserted, keys -1..-10 are never removed
        for (int k = -10; k < 0; ++k) L.add(k);
        std::vector<std::thread> th;
        for (int t = 0; t < T; ++t) th.emplace_back([&, t] {
            std::mt19937 rng(100 + t);
            for (int i = 0; i < OPS; ++i) {
                Key k = rng() % R; int op = rng() % 10;
                if (op < 3) { if (L.add(k)) net[t][k]++; }
                else if (op < 6) { if (L.remove(k)) net[t][k]--; }
                else if (op < 8) { L.contains(k); }
                else { CHECK(L.contains(-1 - (Key)(rng() % 10)) && !L.contains(1000 + (Key)(rng() % 10)), "stable keys"); }
            }
        });
        for (auto& x : th) x.join();
        for (int k = 0; k < R; ++k) {
            int c = init[k]; for (int t = 0; t < T; ++t) c += net[t][k];
            CHECK((c == 0 || c == 1) && (c == 1) == L.contains(k), "key %d: net=%d member=%d", k, c, (int)L.contains(k));
        }
        CHECK(L.check_sorted_unmarked(), "list not sorted / marked node reachable");
    }
    // (d) concurrent removeMin: every key removed exactly once, per-thread sequence increasing
    {
        List L; const int NK = 6000;
        for (int k = NK - 1; k >= 0; --k) L.add(k * 3);
        std::vector<std::vector<Key>> got(T);
        std::vector<std::thread> th;
        for (int t = 0; t < T; ++t) th.emplace_back([&, t] { Key m; while (L.removeMin(m)) got[t].push_back(m); });
        for (auto& x : th) x.join();
        std::vector<Key> all;
        for (auto& v : got) { CHECK(std::is_sorted(v.begin(), v.end()) && std::adjacent_find(v.begin(), v.end()) == v.end(), "per-thread order"); all.insert(all.end(), v.begin(), v.end()); }
        std::sort(all.begin(), all.end());
        bool ok = (int)all.size() == NK;
        for (int i = 0; ok && i < NK; ++i) ok = all[i] == (Key)i * 3;
        CHECK(ok && L.isEmpty(), "removeMin lost/duplicated keys (%zu of %d)", all.size(), NK);
    }
    // (e) producers + consumers through removeMin (priority-queue usage)
    {
        List L; const int PER = 3000; std::atomic<int> consumed{0}; std::atomic<bool> done{false};
        std::vector<std::atomic<int>> seen(PER * T);
        std::vector<std::thread> th;
        int P = std::max(1, T / 2), C = std::max(1, T - P);
        for (int p = 0; p < P; ++p) th.emplace_back([&, p] { for (int i = p; i < PER * T; i += P) L.add(i); });
        for (int c = 0; c < C; ++c) th.emplace_back([&] { Key m; while (!done.load() || !L.isEmpty()) { if (L.removeMin(m)) { seen[m]++; consumed++; } } });
        for (int p = 0; p < P; ++p) th[p].join();
        done = true;
        for (int c = 0; c < C; ++c) th[P + c].join();
        bool ok = consumed.load() == PER * T;
        for (auto& s : seen) { ok = ok && s.load() == 1; }
        CHECK(ok, "producer/consumer: consumed %d of %d", consumed.load(), PER * T);
    }
}

template <class Lock> void all_lists(int T) {
    run_tests<CoarseList<Lock>>(T); run_tests<FineList<Lock>>(T);
    run_tests<OptimisticList<Lock>>(T); run_tests<LazyList<Lock>>(T);
}

int main(int argc, char** argv) {
    int maxT = argc > 1 ? std::atoi(argv[1]) : 8;
    bool skip_ttas = argc > 2 && std::string(argv[2]) == "--no-ttas";  // pure spinning is very slow when oversubscribed
    for (int T : {1, 2, 4, maxT}) {
        if (T > maxT) continue;
        std::printf("== %d threads\n", T);
        all_lists<MutexLock>(T); if (!skip_ttas) all_lists<TTASLock>(T); all_lists<BackoffLock>(T);
        if (T == maxT) break;
    }
    std::printf("\n%s (%d failures)\n", fails ? "CONCURRENT LIST TESTS FAILED" : "ALL CONCURRENT LIST TESTS PASSED", fails);
    return fails ? 1 : 0;
}
