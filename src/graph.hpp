// graph.hpp - directed road graph (CSR) of central Ho Chi Minh City.
// All costs are integers (millimetres / milliseconds) so that every algorithm
// (sequential or parallel) can be compared exactly, without FP tolerance.
#pragma once
#include <cmath>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <algorithm>

namespace hcm {

using Cost = int64_t;
constexpr Cost INF = INT64_MAX / 4;

enum Metric { LENGTH = 0, TIME = 1, CONGESTED = 2 };  // CONGESTED = time with rush-hour factors
constexpr int NUM_METRICS = 3;

struct Edge { int to; Cost w[NUM_METRICS]; uint8_t cls; };

inline double haversine_m(double la1, double lo1, double la2, double lo2) {
    const double R = 6371008.8, d2r = M_PI / 180.0;
    double p1 = la1 * d2r, p2 = la2 * d2r, dp = p2 - p1, dl = (lo2 - lo1) * d2r;
    double a = std::sin(dp / 2) * std::sin(dp / 2) + std::cos(p1) * std::cos(p2) * std::sin(dl / 2) * std::sin(dl / 2);
    return 2 * R * std::asin(std::sqrt(a));
}

// Road classes ordered by importance; rush-hour congestion factor per class.
inline int road_class(const std::string& h) {
    static const char* names[] = {"trunk", "primary", "secondary", "tertiary", "residential", "service", "other"};
    for (int i = 0; i < 6; ++i)
        if (h.rfind(names[i], 0) == 0) return i;
    return 6;
}
inline double congestion_factor(int cls) {
    static const double f[] = {2.6, 2.4, 1.9, 1.5, 1.1, 1.0, 1.2};
    return f[cls];
}

struct Graph {
    int n = 0;
    std::vector<double> lat, lon;
    std::vector<int> off;     // CSR offsets (forward)
    std::vector<Edge> adj;
    std::vector<int> roff;    // CSR offsets (reverse graph)
    std::vector<Edge> radj;
    double vmax_mps[NUM_METRICS] = {1, 1, 1};  // max "speed" = min cost per metre, for admissible h

    int degree(int u) const { return off[u + 1] - off[u]; }
    const Edge* begin(int u) const { return adj.data() + off[u]; }
    const Edge* end(int u) const { return adj.data() + off[u + 1]; }
    const Edge* rbegin(int u) const { return radj.data() + roff[u]; }
    const Edge* rend(int u) const { return radj.data() + roff[u + 1]; }

    double dist_m(int a, int b) const { return haversine_m(lat[a], lon[a], lat[b], lon[b]); }

    // Admissible & consistent straight-line lower bound for metric m (integer units).
    // A tiny safety margin absorbs mm/ms rounding of the stored edge costs.
    Cost lower_bound(int a, int b, int m) const {
        double d = dist_m(a, b);
        double v = (m == LENGTH) ? d * 1000.0 : d / vmax_mps[m] * 1000.0;
        Cost c = (Cost)std::floor(v * (1.0 - 1e-9)) - 1;
        return c > 0 ? c : 0;
    }

    int nearest(double la, double lo) const {
        int best = 0; double bd = 1e300;
        for (int i = 0; i < n; ++i) { double d = haversine_m(la, lo, lat[i], lon[i]); if (d < bd) { bd = d; best = i; } }
        return best;
    }

    // Path cost under metric m; returns -1 if the path is not a valid walk in the graph.
    Cost path_cost(const std::vector<int>& p, int m) const {
        if (p.empty()) return -1;
        Cost c = 0;
        for (size_t i = 0; i + 1 < p.size(); ++i) {
            Cost best = -1;
            for (auto e = begin(p[i]); e != end(p[i]); ++e)
                if (e->to == p[i + 1] && (best < 0 || e->w[m] < best)) best = e->w[m];
            if (best < 0) return -1;
            c += best;
        }
        return c;
    }

    static Graph load(const std::string& dir) {
        Graph g;
        std::ifstream fn(dir + "/nodes.csv"), fe(dir + "/edges.csv");
        if (!fn || !fe) throw std::runtime_error("cannot open " + dir + "/nodes.csv or edges.csv");
        std::string line;
        std::getline(fn, line);
        while (std::getline(fn, line)) {
            std::stringstream ss(line); std::string a, b, c;
            std::getline(ss, a, ','); std::getline(ss, b, ','); std::getline(ss, c, ',');
            g.lat.push_back(std::stod(b)); g.lon.push_back(std::stod(c));
        }
        g.n = (int)g.lat.size();
        struct Raw { int u, v; double L, T; int cls; };
        std::vector<Raw> raw;
        std::getline(fe, line);
        while (std::getline(fe, line)) {
            std::stringstream ss(line); std::string u, v, L, T, h;
            std::getline(ss, u, ','); std::getline(ss, v, ','); std::getline(ss, L, ',');
            std::getline(ss, T, ','); std::getline(ss, h, ',');
            raw.push_back({std::stoi(u), std::stoi(v), std::stod(L), std::stod(T), road_class(h)});
        }
        auto build = [&](bool rev, std::vector<int>& o, std::vector<Edge>& a) {
            o.assign(g.n + 1, 0);
            for (auto& r : raw) o[(rev ? r.v : r.u) + 1]++;
            for (int i = 0; i < g.n; ++i) o[i + 1] += o[i];
            a.resize(raw.size());
            std::vector<int> pos(o.begin(), o.end() - 1);
            for (auto& r : raw) {
                Edge e;
                e.to = rev ? r.u : r.v;
                e.cls = (uint8_t)r.cls;
                // Edge length is recomputed from coordinates so that h is provably consistent.
                double L = std::max(r.L, g.dist_m(r.u, r.v));
                double T = r.T * (L / std::max(r.L, 1e-9));
                e.w[LENGTH] = (Cost)std::ceil(L * 1000.0);
                e.w[TIME] = (Cost)std::ceil(T * 1000.0);
                e.w[CONGESTED] = (Cost)std::ceil(T * congestion_factor(r.cls) * 1000.0);
                a[pos[rev ? r.v : r.u]++] = e;
            }
        };
        build(false, g.off, g.adj);
        build(true, g.roff, g.radj);
        for (int m = 1; m < NUM_METRICS; ++m) {
            double v = 0;
            for (auto& r : raw) {
                double L = std::max(r.L, g.dist_m(r.u, r.v));
                double cost_s = r.T * (L / std::max(r.L, 1e-9)) * (m == CONGESTED ? congestion_factor(r.cls) : 1.0);
                if (cost_s > 0) v = std::max(v, L / cost_s);
            }
            g.vmax_mps[m] = v * (1.0 + 1e-6);
        }
        return g;
    }
};

}  // namespace hcm
