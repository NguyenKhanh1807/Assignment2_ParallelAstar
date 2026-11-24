#!/usr/bin/env python3
"""Build a directed road graph of central Ho Chi Minh City from raw OSM XML extracts.

Input : data/osm/*.osm   (OpenStreetMap exports, (c) OpenStreetMap contributors, ODbL)
Output: data/nodes.csv   id,lat,lon
        data/edges.csv   u,v,length_m,time_s,highway
Only the largest strongly connected component is kept, so every (s,t) pair is reachable.
"""
import glob, math, os, sys
import xml.etree.ElementTree as ET
import networkx as nx

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DRIVABLE = {  # highway class -> default speed (km/h) for motorbikes in HCMC
    "motorway": 60, "motorway_link": 40, "trunk": 50, "trunk_link": 35,
    "primary": 40, "primary_link": 30, "secondary": 35, "secondary_link": 25,
    "tertiary": 30, "tertiary_link": 25, "unclassified": 25, "residential": 20,
    "living_street": 10, "service": 15,
}

def hav(lat1, lon1, lat2, lon2):
    R = 6371008.8
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = p2 - p1, math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R * math.asin(math.sqrt(a))

def parse_speed(tag, default):
    try:
        return float(tag.split()[0])
    except Exception:
        return default

def main():
    coords, ways = {}, {}
    for f in sorted(glob.glob(os.path.join(ROOT, "data/osm/*.osm"))):
        for ev, el in ET.iterparse(f, events=("end",)):
            if el.tag == "node":
                coords[int(el.get("id"))] = (float(el.get("lat")), float(el.get("lon")))
            elif el.tag == "way":
                tags = {t.get("k"): t.get("v") for t in el.findall("tag")}
                hw = tags.get("highway")
                if hw in DRIVABLE and tags.get("access") not in ("no", "private") \
                        and tags.get("motor_vehicle") != "no":
                    ways[int(el.get("id"))] = ([int(n.get("ref")) for n in el.findall("nd")], tags)
                el.clear()
            elif el.tag == "relation":
                el.clear()
    G = nx.DiGraph()
    for nds, tags in ways.values():
        hw = tags["highway"]
        spd = parse_speed(tags.get("maxspeed", ""), DRIVABLE[hw])
        ow = tags.get("oneway", "no")
        if tags.get("junction") in ("roundabout", "circular") and ow == "no":
            ow = "yes"
        if ow == "-1":
            nds = nds[::-1]; ow = "yes"
        for a, b in zip(nds, nds[1:]):
            if a not in coords or b not in coords or a == b:
                continue
            L = hav(*coords[a], *coords[b])
            T = L / (spd / 3.6)
            for (u, v) in ([(a, b)] if ow in ("yes", "true", "1") else [(a, b), (b, a)]):
                if not G.has_edge(u, v) or G[u][v]["length"] > L:
                    G.add_edge(u, v, length=L, time=T, highway=hw)
    scc = max(nx.strongly_connected_components(G), key=len)
    G = G.subgraph(scc).copy()
    idx = {osm: i for i, osm in enumerate(sorted(G.nodes))}
    with open(os.path.join(ROOT, "data/nodes.csv"), "w") as f:
        f.write("id,lat,lon,osm_id\n")
        for osm, i in sorted(idx.items(), key=lambda x: x[1]):
            f.write(f"{i},{coords[osm][0]:.7f},{coords[osm][1]:.7f},{osm}\n")
    with open(os.path.join(ROOT, "data/edges.csv"), "w") as f:
        f.write("u,v,length_m,time_s,highway\n")
        for u, v, d in G.edges(data=True):
            f.write(f"{idx[u]},{idx[v]},{d['length']:.3f},{d['time']:.3f},{d['highway']}\n")
    print(f"ways={len(ways)} nodes={G.number_of_nodes()} edges={G.number_of_edges()}")

if __name__ == "__main__":
    main()
