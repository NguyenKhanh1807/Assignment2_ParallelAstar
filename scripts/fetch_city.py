#!/usr/bin/env python3
"""Download the Ho Chi Minh City drive network with OSMnx (as in the lecturer's notebook)
and write it in this project's format.

Output: <out>/nodes.csv  id,lat,lon,osm_id
        <out>/edges.csv  u,v,length_m,time_s,highway
Only the largest strongly connected component is kept, so every (s, t) pair has a route.

The lecturer's notebook geocodes the place name through Nominatim. Some networks block
Nominatim, so this script downloads a bounding box instead (no geocoding) and tries several
public Overpass servers in turn. If every Overpass server is blocked it says so and stops.
Usage:  python3 fetch_city.py [out_dir]       (default: data_city)
"""
import os, sys, time
import requests
import networkx as nx

# urban Ho Chi Minh City (former boundary): districts 1-12, Binh Thanh, Phu Nhuan, Go Vap, Tan Binh,
# Tan Phu, Binh Tan, Thu Duc city, Nha Be and the urban parts of Binh Chanh / Hoc Mon.
# It contains the lecturer's example (10.768856, 106.725019) -> (10.839613, 106.845798).
WEST, SOUTH, EAST, NORTH = 106.55, 10.65, 106.88, 10.92
OVERPASS = [
    "https://overpass-api.de/api",
    "https://overpass.kumi.systems/api",
    "https://overpass.private.coffee/api",
    "https://maps.mail.ru/osm/tools/overpass/api",
]
DEFAULT_KMH = {  # same defaults as scripts/build_graph.py
    "motorway": 60, "motorway_link": 40, "trunk": 50, "trunk_link": 35,
    "primary": 40, "primary_link": 30, "secondary": 35, "secondary_link": 25,
    "tertiary": 30, "tertiary_link": 25, "unclassified": 25, "residential": 20,
    "living_street": 10, "service": 15,
}

def first(x):
    return x[0] if isinstance(x, list) else x

def kmh(data):
    hw = first(data.get("highway", "unclassified"))
    try:
        return hw, float(str(first(data.get("maxspeed", ""))).split()[0])
    except (ValueError, IndexError):
        return hw, DEFAULT_KMH.get(hw, 20)

def reachable(url):
    try:
        r = requests.get(url + "/status", timeout=15)
        return r.status_code < 500, f"HTTP {r.status_code}"
    except Exception as e:
        return False, type(e).__name__

def download():
    import osmnx as ox
    ox.settings.requests_timeout = 600
    ox.settings.use_cache = True
    print("network check:")
    for name, url in [("PyPI", "https://pypi.org"), ("Nominatim", "https://nominatim.openstreetmap.org"),
                      ("Geofabrik", "https://download.geofabrik.de")]:
        try:
            code = requests.get(url, timeout=15).status_code; print(f"  {name:10s} HTTP {code}")
        except Exception as e:
            print(f"  {name:10s} {type(e).__name__}")
    usable = []
    for url in OVERPASS:
        ok, msg = reachable(url)
        print(f"  {url:45s} {msg}")
        if ok: usable.append(url)
    for url in usable:
        ox.settings.overpass_url = url
        try:
            print(f"downloading bbox W{WEST} S{SOUTH} E{EAST} N{NORTH} from {url} ... (several minutes)", flush=True)
            t0 = time.time()
            G = ox.graph_from_bbox(bbox=(WEST, SOUTH, EAST, NORTH), network_type="drive")
            print(f"downloaded in {time.time() - t0:.0f} s: {G.number_of_nodes()} nodes, {G.number_of_edges()} edges", flush=True)
            return G
        except Exception as e:
            print(f"  failed on {url}: {type(e).__name__}: {str(e)[:200]}", flush=True)
    sys.exit("\nNo Overpass server could be used from this network. Copy the 'network check' lines above to Claude.")

def write(G, out):
    os.makedirs(out, exist_ok=True)
    best = {}
    for u, v, d in G.edges(data=True):
        if u == v:
            continue
        L = float(d.get("length", 0.0))
        if L <= 0:
            continue
        if (u, v) not in best or best[(u, v)][0] > L:
            hw, s = kmh(d)
            best[(u, v)] = (L, L / (s / 3.6), hw)
    H = nx.DiGraph(); H.add_edges_from(best.keys())
    scc = max(nx.strongly_connected_components(H), key=len)
    idx = {osm: i for i, osm in enumerate(sorted(scc))}
    with open(f"{out}/nodes.csv", "w") as f:
        f.write("id,lat,lon,osm_id\n")
        for osm, i in sorted(idx.items(), key=lambda x: x[1]):
            f.write(f"{i},{G.nodes[osm]['y']:.7f},{G.nodes[osm]['x']:.7f},{osm}\n")
    n_e = 0
    with open(f"{out}/edges.csv", "w") as f:
        f.write("u,v,length_m,time_s,highway\n")
        for (u, v), (L, T, hw) in best.items():
            if u in idx and v in idx:
                f.write(f"{idx[u]},{idx[v]},{L:.3f},{T:.3f},{hw}\n"); n_e += 1
    print(f"written {out}/nodes.csv ({len(idx)} nodes), {out}/edges.csv ({n_e} edges)")

def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "data_city"
    write(download(), out)

if __name__ == "__main__":
    main()
