// clist.hpp - Part 2: concurrent set as a sorted linked list (Herlihy & Shavit, ch. 9).
// Four synchronisation strategies, each templated on the lock type:
//   CoarseList      - one lock for the whole list
//   FineList        - hand-over-hand (lock coupling) on individual nodes
//   OptimisticList  - lock-free traversal, lock pred/curr, re-traverse to validate
//   LazyList        - lock-free traversal + logical deletion mark, wait-free contains()
// Every list also offers removeMin(), which makes it usable as a concurrent priority
// queue (open list) for the parallel A* of Part 3.
// Locks: std::mutex, TTASLock (spin), BackoffLock (TTAS + exponential backoff).
// Memory: unlinked nodes are retired to a lock-free stack and freed in the destructor
// (no node is ever freed while another thread might still be traversing it).
#pragma once
#include <atomic>
#include <climits>
#include <mutex>
#include <random>
#include <thread>
#include <vector>
#if defined(__x86_64__) || defined(__i386__)
#include <immintrin.h>
#define CPU_RELAX() _mm_pause()
#elif defined(__aarch64__)
#define CPU_RELAX() asm volatile("yield" ::: "memory")
#else
#define CPU_RELAX() std::this_thread::yield()
#endif

namespace clist {

using Key = long long;
constexpr Key KMIN = LLONG_MIN, KMAX = LLONG_MAX;

// ------------------------------------------------------------------ locks
struct TTASLock {
    std::atomic<bool> state{false};
    void lock() {
        for (;;) {
            while (state.load(std::memory_order_relaxed)) CPU_RELAX();
            if (!state.exchange(true, std::memory_order_acquire)) return;
        }
    }
    void unlock() { state.store(false, std::memory_order_release); }
    static const char* name() { return "ttas"; }
};

// Exponential backoff (Assignment 1): after a failed acquisition, wait a random time in
// [0, limit) and double the limit up to MAX_DELAY - spreads out competing threads.
struct Backoff {
    unsigned limit, maxd; std::minstd_rand rng;
    static unsigned seed() {
        static std::atomic<unsigned> ctr{12345};
        thread_local unsigned s = ctr.fetch_add(7919);
        return s += 104729;
    }
    explicit Backoff(unsigned mn = 4, unsigned mx = 1024) : limit(mn), maxd(mx), rng(seed()) {}
    void backoff() {
        unsigned d = rng() % limit;
        limit = std::min(maxd, limit * 2);
        for (unsigned i = 0; i < d; ++i) CPU_RELAX();
        if (limit == maxd) std::this_thread::yield();  // be nice when oversubscribed
    }
};
struct BackoffLock {
    std::atomic<bool> state{false};
    void lock() {
        Backoff b;
        for (;;) {  // back off both when the lock is observed busy and after a failed attempt
            if (!state.load(std::memory_order_relaxed) && !state.exchange(true, std::memory_order_acquire)) return;
            b.backoff();
        }
    }
    void unlock() { state.store(false, std::memory_order_release); }
    static const char* name() { return "backoff"; }
};
struct MutexLock {
    std::mutex m;
    void lock() { m.lock(); }
    void unlock() { m.unlock(); }
    static const char* name() { return "mutex"; }
};

// ------------------------------------------------------------------ node & base
template <class Lock>
struct Node {
    Key key;
    std::atomic<Node*> next{nullptr};
    std::atomic<bool> marked{false};
    Node* retired_next = nullptr;
    Lock lk;
    explicit Node(Key k) : key(k) {}
    void lock() { lk.lock(); }
    void unlock() { lk.unlock(); }
};

template <class Lock>
struct ListBase {
    using N = Node<Lock>;
    N* head; N* tail;
    std::atomic<N*> retired{nullptr};
    std::atomic<long> retired_cnt{0};
    ListBase() { head = new N(KMIN); tail = new N(KMAX); head->next.store(tail); }
    ~ListBase() {
        N* c = head;
        while (c) { N* n = c->next.load(); delete c; c = n; }
        N* r = retired.load();
        while (r) { N* n = r->retired_next; delete r; r = n; }
    }
    ListBase(const ListBase&) = delete;
    void retire(N* x) {
        retired_cnt.fetch_add(1, std::memory_order_relaxed);
        N* old = retired.load(std::memory_order_relaxed);
        do { x->retired_next = old; } while (!retired.compare_exchange_weak(old, x, std::memory_order_release, std::memory_order_relaxed));
    }
    long retired_count() const { return retired_cnt.load(std::memory_order_relaxed); }
    static constexpr size_t node_bytes() { return sizeof(N); }
    bool isEmpty() const { return head->next.load() == tail; }
    // ---- quiescent-state helpers for tests (call only when no thread is operating)
    long size() const { long c = 0; for (N* x = head->next.load(); x != tail; x = x->next.load()) c++; return c; }
    bool check_sorted_unmarked() const {
        for (N* x = head; x != tail; x = x->next.load()) {
            N* n = x->next.load();
            if (!n || n->key <= x->key || n->marked.load()) return false;
        }
        return true;
    }
    std::vector<Key> keys() const { std::vector<Key> v; for (N* x = head->next.load(); x != tail; x = x->next.load()) v.push_back(x->key); return v; }
};

// ------------------------------------------------------------------ 1. coarse-grained
template <class Lock = MutexLock>
struct CoarseList : ListBase<Lock> {
    using B = ListBase<Lock>; using N = typename B::N;
    Lock L;
    static std::string name() { return std::string("coarse-") + Lock::name(); }
    bool add(Key k) {
        std::lock_guard<Lock> g(L);
        N* pred = this->head; N* curr = pred->next.load(std::memory_order_relaxed);
        while (curr->key < k) { pred = curr; curr = curr->next.load(std::memory_order_relaxed); }
        if (curr->key == k) return false;
        N* n = new N(k); n->next.store(curr, std::memory_order_relaxed);
        pred->next.store(n, std::memory_order_relaxed);
        return true;
    }
    bool remove(Key k) {
        std::lock_guard<Lock> g(L);
        N* pred = this->head; N* curr = pred->next.load(std::memory_order_relaxed);
        while (curr->key < k) { pred = curr; curr = curr->next.load(std::memory_order_relaxed); }
        if (curr->key != k) return false;
        pred->next.store(curr->next.load(std::memory_order_relaxed), std::memory_order_relaxed);
        this->retire(curr);
        return true;
    }
    bool contains(Key k) {
        std::lock_guard<Lock> g(L);
        N* curr = this->head->next.load(std::memory_order_relaxed);
        while (curr->key < k) curr = curr->next.load(std::memory_order_relaxed);
        return curr->key == k;
    }
    bool removeMin(Key& out) {
        std::lock_guard<Lock> g(L);
        N* first = this->head->next.load(std::memory_order_relaxed);
        if (first == this->tail) return false;
        out = first->key;
        this->head->next.store(first->next.load(std::memory_order_relaxed), std::memory_order_relaxed);
        this->retire(first);
        return true;
    }
    bool isEmpty() { std::lock_guard<Lock> g(L); return this->head->next.load() == this->tail; }
};

// ------------------------------------------------------------------ 2. fine-grained (hand-over-hand)
template <class Lock = MutexLock>
struct FineList : ListBase<Lock> {
    using B = ListBase<Lock>; using N = typename B::N;
    static std::string name() { return std::string("fine-") + Lock::name(); }
    // returns (pred, curr) both locked, pred->key < k <= curr->key
    void find(Key k, N*& pred, N*& curr) {
        pred = this->head; pred->lock();
        curr = pred->next.load(std::memory_order_acquire); curr->lock();
        while (curr->key < k) {
            pred->unlock(); pred = curr;
            curr = curr->next.load(std::memory_order_acquire); curr->lock();
        }
    }
    bool add(Key k) {
        N *pred, *curr; find(k, pred, curr);
        bool ok = curr->key != k;
        if (ok) { N* n = new N(k); n->next.store(curr); pred->next.store(n, std::memory_order_release); }
        curr->unlock(); pred->unlock();
        return ok;
    }
    bool remove(Key k) {
        N *pred, *curr; find(k, pred, curr);
        bool ok = curr->key == k;
        if (ok) { pred->next.store(curr->next.load(), std::memory_order_release); curr->marked.store(true); }
        curr->unlock(); pred->unlock();
        if (ok) this->retire(curr);
        return ok;
    }
    bool contains(Key k) {
        N *pred, *curr; find(k, pred, curr);
        bool ok = curr->key == k;
        curr->unlock(); pred->unlock();
        return ok;
    }
    bool removeMin(Key& out) {
        N* pred = this->head; pred->lock();
        N* curr = pred->next.load(std::memory_order_acquire); curr->lock();
        bool ok = curr != this->tail;
        if (ok) { out = curr->key; pred->next.store(curr->next.load(), std::memory_order_release); curr->marked.store(true); }
        curr->unlock(); pred->unlock();
        if (ok) this->retire(curr);
        return ok;
    }
};

// ------------------------------------------------------------------ 3. optimistic
template <class Lock = MutexLock>
struct OptimisticList : ListBase<Lock> {
    using B = ListBase<Lock>; using N = typename B::N;
    static std::string name() { return std::string("optimistic-") + Lock::name(); }
    void search(Key k, N*& pred, N*& curr) {
        pred = this->head; curr = pred->next.load(std::memory_order_acquire);
        while (curr->key < k) { pred = curr; curr = curr->next.load(std::memory_order_acquire); }
    }
    // pred still reachable from head and still points to curr
    bool validate(N* pred, N* curr) {
        N* node = this->head;
        while (node->key <= pred->key) {
            if (node == pred) return pred->next.load(std::memory_order_acquire) == curr;
            node = node->next.load(std::memory_order_acquire);
        }
        return false;
    }
    template <class F>
    auto locked(Key k, F f) {
        Backoff b(1, 256);
        for (;;) {
            N *pred, *curr; search(k, pred, curr);
            pred->lock(); curr->lock();
            if (validate(pred, curr)) { auto r = f(pred, curr); curr->unlock(); pred->unlock(); return r; }
            curr->unlock(); pred->unlock();
            b.backoff();
        }
    }
    bool add(Key k) {
        return locked(k, [&](N* pred, N* curr) {
            if (curr->key == k) return false;
            N* n = new N(k); n->next.store(curr); pred->next.store(n, std::memory_order_release); return true; });
    }
    bool remove(Key k) {
        N* victim = nullptr;
        bool ok = locked(k, [&](N* pred, N* curr) {
            if (curr->key != k) return false;
            pred->next.store(curr->next.load(), std::memory_order_release); curr->marked.store(true); victim = curr; return true; });
        if (ok) this->retire(victim);
        return ok;
    }
    bool contains(Key k) { return locked(k, [&](N*, N* curr) { return curr->key == k; }); }
    bool removeMin(Key& out) {
        Backoff b(1, 256);
        for (;;) {
            N* pred = this->head; N* curr = pred->next.load(std::memory_order_acquire);
            if (curr == this->tail) return false;
            pred->lock(); curr->lock();
            if (pred->next.load(std::memory_order_acquire) == curr) {  // validation (pred == head is always reachable)
                out = curr->key; pred->next.store(curr->next.load(), std::memory_order_release); curr->marked.store(true);
                curr->unlock(); pred->unlock(); this->retire(curr); return true;
            }
            curr->unlock(); pred->unlock();
            b.backoff();
        }
    }
};

// ------------------------------------------------------------------ 4. lazy
template <class Lock = MutexLock>
struct LazyList : ListBase<Lock> {
    using B = ListBase<Lock>; using N = typename B::N;
    static std::string name() { return std::string("lazy-") + Lock::name(); }
    bool validate(N* pred, N* curr) {
        return !pred->marked.load(std::memory_order_acquire) && !curr->marked.load(std::memory_order_acquire) &&
               pred->next.load(std::memory_order_acquire) == curr;
    }
    bool add(Key k) {
        Backoff b(1, 256);
        for (;;) {
            N* pred = this->head; N* curr = pred->next.load(std::memory_order_acquire);
            while (curr->key < k) { pred = curr; curr = curr->next.load(std::memory_order_acquire); }
            pred->lock(); curr->lock();
            if (validate(pred, curr)) {
                bool ok = curr->key != k;
                if (ok) { N* n = new N(k); n->next.store(curr); pred->next.store(n, std::memory_order_release); }
                curr->unlock(); pred->unlock(); return ok;
            }
            curr->unlock(); pred->unlock(); b.backoff();
        }
    }
    bool remove(Key k) {
        Backoff b(1, 256);
        for (;;) {
            N* pred = this->head; N* curr = pred->next.load(std::memory_order_acquire);
            while (curr->key < k) { pred = curr; curr = curr->next.load(std::memory_order_acquire); }
            pred->lock(); curr->lock();
            if (validate(pred, curr)) {
                bool ok = curr->key == k;
                if (ok) {
                    curr->marked.store(true, std::memory_order_release);          // logical delete
                    pred->next.store(curr->next.load(), std::memory_order_release); // physical delete
                }
                curr->unlock(); pred->unlock();
                if (ok) this->retire(curr);
                return ok;
            }
            curr->unlock(); pred->unlock(); b.backoff();
        }
    }
    bool contains(Key k) {  // wait-free
        N* curr = this->head;
        while (curr->key < k) curr = curr->next.load(std::memory_order_acquire);
        return curr->key == k && !curr->marked.load(std::memory_order_acquire);
    }
    bool removeMin(Key& out) {
        Backoff b(1, 256);
        for (;;) {
            N* pred = this->head; N* curr = pred->next.load(std::memory_order_acquire);
            if (curr == this->tail) return false;
            pred->lock(); curr->lock();
            if (validate(pred, curr)) {
                out = curr->key;
                curr->marked.store(true, std::memory_order_release);
                pred->next.store(curr->next.load(), std::memory_order_release);
                curr->unlock(); pred->unlock(); this->retire(curr); return true;
            }
            curr->unlock(); pred->unlock(); b.backoff();
        }
    }
};

}  // namespace clist
