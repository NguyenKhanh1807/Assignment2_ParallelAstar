# Parallel Computing – Assignment 2: sequential & parallel A* on the HCMC road network

Report: `report/Assignment2_Report.pdf`

## Build & run (Linux or macOS, g++ ≥ 9 / clang, C++17, pthreads)

```bash
make                     # builds everything into bin/
make test                # correctness tests: sequential A*, concurrent lists, parallel A*
make tsan                # same tests under ThreadSanitizer (data-race detection)
./run_all.sh my-laptop   # all tests + benchmarks on the central graph (Part 2 results used in the report)
./run_city.sh my-laptop  # Parts 1 and 3 on the whole-city graph -> results_city/*_my-laptop.*
bash run_mac_city.sh     # the same on macOS (Apple clang)
cd report/latex && python3 scripts/make_figures.py ../../results_city ../../results ../../data_city figures \
  && python3 scripts/make_tables.py ../../results_city ../../results tables && latexmk -pdf main.tex   # figures, tables, PDF
./bin/route 10.768856 106.725019 10.839613 106.845798 --algo alt --data data_city   # the lecturer's example
```

`--algo` accepts `dijkstra | astar | alt | par-lazy | par-heap | hda`, `--threads T`, `--metric length|time`.

## Layout

| Path | Content |
|---|---|
| `src/graph.hpp` | CSR road graph, integer costs (mm / ms), admissible straight-line bound |
| `src/seq_astar.hpp` | Part 1: Dijkstra oracle, A*, uncertain / dynamic / ALT / costly heuristics, NAMOA* (vector f) |
| `src/clist.hpp` | Part 2: coarse, fine-grained, optimistic, lazy sorted lists × {mutex, TTAS, TTAS+exp. backoff}, `removeMin` |
| `src/par_astar.hpp` | Part 3: parallel A* over any shared list / locked heap, HDA*, HDA* with f-window |
| `tests/` | correctness tests (`test_seq`, `test_clist`, `test_par`) |
| `bench/` | experiments (`bench_seq`, `bench_clist`, `bench_astar`) |
| `data_city/` | graph of urban Ho Chi Minh City (203,396 nodes), used by the report for Parts 1 and 3 |
| `data/` | small central-HCMC graph (33,718 nodes) used during development; `osm/` its raw extracts |
| `scripts/pbf_to_graph.py` | builds `data_city/` from Geofabrik's `vietnam-latest.osm.pbf` (needs pyosmium) |
| `scripts/fetch_city.py` | same via OSMnx/Overpass, when those services are reachable |
| `scripts/build_graph.py` | rebuilds `data/*.csv` from `data/osm/*.osm` |
| `notebooks/osm-astar-fixed.ipynb` | the lecturer's notebook, checked and corrected (10 bugs), with tests |
| `results/`, `results_city/` | raw results of the three platforms (Part 2 / Parts 1 and 3) |
| `report/latex/` | LaTeX source of the report (`main.tex`, `sections/`, `figures/`, `tables/`, `scripts/`) |

Map data © OpenStreetMap contributors, ODbL.
