#!/usr/bin/env python3
"""Build the Ho Chi Minh City drive graph from an OpenStreetMap extract (.osm.pbf or .osm),
e.g. Geofabrik's vietnam-latest.osm.pbf. Used when the Overpass API is unavailable.

The graph is simplified like OSMnx: nodes are only kept where a road ends, where roads meet,
or where a road leaves the bounding box; the edge between two kept nodes gets the summed
great-circle length of the road segments in between.
Output: <out>/nodes.csv id,lat,lon,osm_id ; <out>/edges.csv u,v,length_m,time_s,highway
Usage:  python3 pbf_to_graph.py vietnam-latest.osm.pbf data_city
"""
import math, sys, time
from collections import defaultdict
import osmium

# urban Ho Chi Minh City; contains the lecturer's example (10.768856,106.725019)->(10.839613,106.845798)
WEST, SOUTH, EAST, NORTH = 106.55, 10.65, 106.88, 10.92
DRIVABLE = {  # highway class -> default speed (km/h), same as build_graph.py; OSMnx "drive" classes + service
    "motorway": 60, "motorway_link": 40, "trunk": 50, "trunk_link": 35,
    "primary": 40, "primary_link": 30, "secondary": 35, "secondary_link": 25,
    "tertiary": 30, "tertiary_link": 25, "unclassified": 25, "residential": 20,
    "living_street": 10, "service": 15,
}
SKIP_SERVICE = {"parking", "parking_aisle", "driveway", "private", "emergency_access", "drive-through"}

def inside(lat, lon):
    return SOUTH <= lat <= NORTH and WEST <= lon <= EAST

def hav(la1, lo1, la2, lo2):
    R = 6371008.8
    p1, p2 = math.radians(la1), math.radians(la2)
    dp, dl = p2 - p1, math.radians(lo2 - lo1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R * math.asin(math.sqrt(a))

class Ways(osmium.SimpleHandler):
    def __init__(self):
        super().__init__()
        self.ways = []                      # (node ids, hw, kmh, oneway)
        self.coord = {}
        self.seen = 0
    def way(self, w):
        hw = w.tags.get("highway")
        if hw not in DRIVABLE:
            return
        t = w.tags
        if t.get("area") == "yes" or t.get("access") in ("no", "private") or t.get("motor_vehicle") == "no" \
                or t.get("motorcar") == "no" or (hw == "service" and t.get("service") in SKIP_SERVICE):
            return
        self.seen += 1
        ids, keep = [], False
        for n in w.nodes:
            if not n.location.valid():
                ids.append(None); continue
            lat, lon = n.location.lat, n.location.lon
            ids.append(n.ref)
            if inside(lat, lon):
                keep = True
                self.coord[n.ref] = (lat, lon)
        if not keep:
            return
        try:
            spd = float(t.get("maxspeed", "").split()[0])
        except (ValueError, IndexError):
            spd = DRIVABLE[hw]
        ow = t.get("oneway", "no")
        if t.get("junction") in ("roundabout", "circular") and ow == "no":
            ow = "yes"
        if ow == "-1":
            ids = ids[::-1]; ow = "yes"
        self.ways.append((ids, hw, spd, ow in ("yes", "true", "1")))

def strongly_connected_largest(n, adj):
    """Iterative Tarjan; returns the node set of the largest SCC."""
    index = [-1] * n; low = [0] * n; on = [False] * n; stack = []; best = []; idx = 0
    for root in range(n):
        if index[root] != -1:
            continue
        work = [(root, 0)]
        while work:
            v, i = work.pop()
            if i == 0:
                index[v] = low[v] = idx; idx += 1; stack.append(v); on[v] = True
            recurse = False
            nb = adj[v]
            while i < len(nb):
                w = nb[i]; i += 1
                if index[w] == -1:
                    work.append((v, i)); work.append((w, 0)); recurse = True; break
                elif on[w]:
                    low[v] = min(low[v], index[w])
            if recurse:
                continue
            if low[v] == index[v]:
                comp = []
                while True:
                    w = stack.pop(); on[w] = False; comp.append(w)
                    if w == v: break
                if len(comp) > len(best): best = comp
            if work:
                u = work[-1][0]; low[u] = min(low[u], low[v])
    return set(best)

def main():
    src, out = sys.argv[1], (sys.argv[2] if len(sys.argv) > 2 else "data_city")
    import os; os.makedirs(out, exist_ok=True)
    t0 = time.time()
    h = Ways()
    h.apply_file(src, locations=True, idx="sparse_mem_array")
    print(f"read {src} in {time.time() - t0:.0f} s: {h.seen} drivable ways in file, {len(h.ways)} touch the box", flush=True)
    # runs of consecutive in-box nodes
    runs = []
    use = defaultdict(int)
    for ids, hw, spd, ow in h.ways:
        cur = []
        for r in ids:
            if r is not None and r in h.coord:
                cur.append(r)
            else:
                if len(cur) > 1: runs.append((cur, hw, spd, ow))
                cur = []
        if len(cur) > 1: runs.append((cur, hw, spd, ow))
    for nodes, *_ in runs:
        for k, r in enumerate(nodes):
            use[r] += 2 if k in (0, len(nodes) - 1) else 1
    keep = {r for r, c in use.items() if c >= 2}          # endpoints and junctions
    best = {}
    for nodes, hw, spd, ow in runs:
        a, L = nodes[0], 0.0
        for p, q in zip(nodes, nodes[1:]):
            L += hav(*h.coord[p], *h.coord[q])
            if q in keep:
                if a != q and L > 0:
                    T = L / (spd / 3.6)
                    for (u, v) in ([(a, q)] if ow else [(a, q), (q, a)]):
                        if (u, v) not in best or best[(u, v)][0] > L:
                            best[(u, v)] = (L, T, hw)
                a, L = q, 0.0
    ids = sorted({u for u, _ in best} | {v for _, v in best})
    pos = {r: i for i, r in enumerate(ids)}
    adj = [[] for _ in ids]
    for (u, v) in best: adj[pos[u]].append(pos[v])
    scc = strongly_connected_largest(len(ids), adj)
    final = [ids[i] for i in sorted(scc)]
    fid = {r: i for i, r in enumerate(final)}
    with open(f"{out}/nodes.csv", "w") as f:
        f.write("id,lat,lon,osm_id\n")
        for r in final:
            la, lo = h.coord[r]; f.write(f"{fid[r]},{la:.7f},{lo:.7f},{r}\n")
    ne = 0
    with open(f"{out}/edges.csv", "w") as f:
        f.write("u,v,length_m,time_s,highway\n")
        for (u, v), (L, T, hw) in best.items():
            if u in fid and v in fid:
                f.write(f"{fid[u]},{fid[v]},{L:.3f},{T:.3f},{hw}\n"); ne += 1
    print(f"written {out}: {len(final)} nodes, {ne} edges ({time.time() - t0:.0f} s)")

if __name__ == "__main__":
    main()
