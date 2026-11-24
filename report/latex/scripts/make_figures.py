"""Regenerate the report figures.
Usage: python3 make_figures.py <results_city_dir> <results_part2_dir> <graph_dir> <figures_dir>
Parts 1 and 3 use the whole-city graph (results_city); Part 2 does not use the graph (results)."""
import json, os, sys
import pandas as pd, matplotlib
matplotlib.use('Agg'); import matplotlib.pyplot as plt
res, res2, data, out = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4]; os.makedirs(out, exist_ok=True)
PLAT = [('macos-native-apple-m5-10c', 'A: macOS, Apple M5 (4P+6E)', 10),
        ('apple-silicon-arm64-4vcpu', 'B: Linux VM on M5 (4 vCPU)', 4),
        ('cloud-x86-2vcpu', 'C: Linux, Xeon cloud VM (2 vCPU)', 2)]
plt.rcParams.update({'font.size': 8.5, 'axes.spines.top': False, 'axes.spines.right': False, 'axes.grid': True,
    'grid.color': '#e4e3df', 'grid.linewidth': 0.6, 'axes.edgecolor': '#8a8984', 'xtick.color': '#52514e',
    'ytick.color': '#52514e', 'legend.frameon': False, 'lines.linewidth': 1.6, 'lines.markersize': 4.5})
C = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#e87ba4', '#008300', '#7b5ea7']
MK = ['o', 's', '^', 'D', 'v', 'P', 'X']
XT = (1, 2, 4, 8, 16)
def rd(kind, p):
    d = pd.read_csv(f'{res2 if kind == "clist" else res}/{kind}_{p}.csv')
    return d.drop_duplicates(subset=[c for c in ('version', 'heuristic', 'threads', 'list', 'mix', 'metric', 'algorithm', 'eps') if c in d.columns], keep='first') if kind in ('astar', 'clist') else d
def xaxis(ax, hw=None, xt=XT):
    ax.set_xscale('log', base=2); ax.set_xticks(list(xt)); ax.set_xticklabels([str(x) for x in xt])
    ax.minorticks_off()
    if hw and hw < xt[-1]: ax.axvspan(hw * 1.12, xt[-1] * 1.06, color='#f2f1ec', zorder=0)
def legend_below(fig, axs, ncol=None):
    h, l = [], []
    for ax in axs.flat:
        for hh, ll in zip(*ax.get_legend_handles_labels()):
            if ll not in l: h.append(hh); l.append(ll)
    fig.legend(h, l, loc='lower center', ncol=ncol or len(l), bbox_to_anchor=(0.5, 0))

# ---- fig1: road graph + the lecturer's example route
from matplotlib.collections import LineCollection
nodes = pd.read_csv(f'{data}/nodes.csv'); edges = pd.read_csv(f'{data}/edges.csv')
lat, lon = nodes.lat.values, nodes.lon.values
fig, ax = plt.subplots(figsize=(6.4, 5.6))
major = edges.highway.str.match('motorway|trunk|primary|secondary')
for sel, c, w in [(~major, '#d6d4cf', 0.15), (major, '#6f6d68', 0.55)]:
    e = edges[sel]
    segs = [[(lon[u], lat[u]), (lon[v], lat[v])] for u, v in zip(e.u.values, e.v.values)]
    ax.add_collection(LineCollection(segs, colors=c, linewidths=w, rasterized=True))
gj = json.load(open(f'{res}/route_teacher.geojson'))['geometry']['coordinates']
ax.plot([p[0] for p in gj], [p[1] for p in gj], color=C[1], lw=2.4, label="lecturer's example, 19.1 km (A* with ALT)")
ax.scatter([gj[0][0], gj[-1][0]], [gj[0][1], gj[-1][1]], color='k', zorder=5, s=16)
ax.annotate('Thu Thiem', (gj[0][0], gj[0][1]), xytext=(-52, -12), textcoords='offset points', fontsize=8)
ax.annotate('Thu Duc', (gj[-1][0], gj[-1][1]), xytext=(6, 3), textcoords='offset points', fontsize=8)
ax.set_xlim(lon.min() - 0.005, lon.max() + 0.005); ax.set_ylim(lat.min() - 0.005, lat.max() + 0.005)
ax.set_xlabel('longitude'); ax.set_ylabel('latitude'); ax.set_aspect(1 / 0.983); ax.grid(False)
ax.legend(loc='lower left'); fig.tight_layout(); fig.savefig(f'{out}/fig1_map.pdf', dpi=300); plt.close(fig)

# ---- fig2: uncertain / dynamic heuristic (platform C; identical counts on B)
d = rd('seq_heuristics', PLAT[2][0])
fig, axs = plt.subplots(1, 3, figsize=(9, 2.9))
i = 0
for alg, lab in [('astar-uncertain', 'uncertain h'), ('astar-dynamic', 'dynamic h')]:
    for m, ls in [('length', '-'), ('time', '--')]:
        x = d[(d.algorithm == alg) & (d.metric == m)].sort_values('eps')
        kw = dict(color=C[0 if alg.endswith('uncertain') else 1], ls=ls, marker=MK[i], label=f'{lab}, {m}')
        axs[0].plot(x.eps, x.avg_ratio, **kw); axs[1].plot(x.eps, x.max_ratio, **kw); axs[2].plot(x.eps, x.avg_expanded, **kw)
        i += 1
for ax in axs[:2]: ax.plot([0, 2], [1, 3], color='#8a8984', lw=0.8, ls=':', label='bound $1+\\varepsilon$')
axs[0].set_ylim(0.99, 1.25); axs[1].set_ylim(0.99, 2.3)
axs[0].set_ylabel('mean cost / optimum'); axs[1].set_ylabel('worst cost / optimum'); axs[2].set_ylabel('expanded nodes (mean)')
for ax in axs: ax.set_xlabel('heuristic error $\\varepsilon$')
legend_below(fig, axs); fig.tight_layout(rect=(0, 0.09, 1, 1)); fig.savefig(f'{out}/fig2_eps.pdf'); plt.close(fig)

# ---- fig3: NAMOA*
n = rd('seq_namoa', PLAT[2][0])
fig, axs = plt.subplots(1, 3, figsize=(9, 2.8))
axs[0].scatter(n.straight_m, n.front_size, s=12, color=C[0]); axs[0].set_yscale('log')
axs[0].set_xlabel('straight-line distance $s$--$t$ (m)'); axs[0].set_ylabel('Pareto-optimal routes')
axs[1].scatter(n.straight_m, n.namoa_ms, s=12, color=C[1], marker='s', label='NAMOA* (vector $h$)')
axs[1].scatter(n.straight_m, n.zero_h_ms, s=14, color='#8a8984', marker='x', label='$h=0$')
axs[1].set_yscale('log'); axs[1].set_xlabel('straight-line distance (m)'); axs[1].set_ylabel('time (ms, log)'); axs[1].legend(fontsize=7.5)
row = n.loc[n.front_size.idxmax()]
pts = [tuple(map(float, p.split(':'))) for p in row.front.strip(';').split(';')]
axs[2].step([p[0] for p in pts], [p[1] / 60 for p in pts], where='post', color=C[2], alpha=0.5, lw=1)
axs[2].scatter([p[0] for p in pts], [p[1] / 60 for p in pts], s=10, color=C[2])
axs[2].set_xlabel('route length (m)'); axs[2].set_ylabel('congested time (min)')
axs[2].set_title(f'largest front: {len(pts)} routes', fontsize=8.5)
fig.tight_layout(); fig.savefig(f'{out}/fig3_namoa.pdf'); plt.close(fig)

# ---- fig4: dynamic traffic re-planning (all platforms give identical travel times on B and C)
t = rd('seq_dynamic', PLAT[2][0]).sort_values('trip')
fig, ax = plt.subplots(figsize=(9, 2.5))
ax.bar(t.trip - 0.2, t.static_s / 60, 0.4, color='#bdbbb5', label='follow the initial plan')
ax.bar(t.trip + 0.2, t.replan_s / 60, 0.4, color=C[2], label='re-plan with A* after every traffic update')
ax.plot(t.trip, t.free_flow_s / 60, 'k_', ms=7, label='free-flow time')
ax.set_xlabel('trip'); ax.set_ylabel('travel time (min)'); ax.legend(ncol=3, fontsize=7.5, loc='upper left')
fig.tight_layout(); fig.savefig(f'{out}/fig4_dynamic.pdf'); plt.close(fig)

# ---- fig5: concurrent lists (rows = platforms, cols = workload mixes)
MIX = [('read', '90% contains'), ('update', '50% contains / 50% add+remove'), ('pq', '50% add / 50% removeMin')]
FAM = {'coarse': 0, 'fine': 1, 'optimistic': 2, 'lazy': 3}
LS = {'mutex': '-', 'backoff': '--', 'ttas': ':'}
fig, axs = plt.subplots(3, 3, figsize=(9, 7.6), sharex=True)
for r, (p, name, hw) in enumerate(PLAT):
    c = rd('clist', p)
    for k, (mix, title) in enumerate(MIX):
        ax = axs[r, k]
        for lst, g in c[c.mix == mix].groupby('list'):
            fam, lk = lst.split('-')
            g = g.sort_values('threads')
            ax.plot(g.threads, g.mops, color=C[FAM[fam]], marker=MK[FAM[fam]], ls=LS[lk], ms=3.2, lw=1.2,
                    label=f'{fam} / {"TTAS+backoff" if lk == "backoff" else ("std::mutex" if lk == "mutex" else "TTAS")}')
        xaxis(ax, hw); ax.set_ylim(bottom=0)
        if r == 0: ax.set_title(title, fontsize=9)
        if k == 0: ax.set_ylabel(f'{name}\nthroughput (Mops/s)', fontsize=8)
        if r == 2: ax.set_xlabel('threads')
legend_below(fig, axs, ncol=4); fig.tight_layout(rect=(0, 0.08, 1, 1)); fig.savefig(f'{out}/fig5_clist.pdf'); plt.close(fig)

# ---- fig6: parallel A* speed-up (rows = platforms, cols = heuristics)
VER = {'list-coarse-backoff': 'list: coarse', 'list-fine-backoff': 'list: fine', 'list-optimistic-backoff': 'list: optimistic',
       'list-lazy-backoff': 'list: lazy', 'heap-backoff': 'shared heap', 'hda': 'HDA*', 'hda-window': 'HDA* + f-window'}
HS = [('zero', '$h = 0$ (parallel Dijkstra)'), ('euclid', 'Euclidean $h$'), ('euclid-costly', 'costly $h$ (400 flops / eval.)')]
fig, axs = plt.subplots(3, 3, figsize=(9, 7.6), sharex=True)
for r, (p, name, hw) in enumerate(PLAT):
    a = rd('astar', p)
    for k, (H, title) in enumerate(HS):
        ax = axs[r, k]; x = a[a.heuristic == H]
        seq = x[x.version == 'seq-binary-heap'].total_ms.iloc[0]
        for i, (v, lab) in enumerate(VER.items()):
            g = x[x.version == v].sort_values('threads')
            ax.plot(g.threads, seq / g.total_ms, color=C[i], marker=MK[i], ls='--' if v.startswith('hda') else '-', label=lab)
        ax.axhline(1, color='#52514e', lw=0.8); xaxis(ax, hw); ax.set_ylim(bottom=0)
        if r == 0: ax.set_title(title, fontsize=9)
        if k == 0: ax.set_ylabel(f'{name}\nspeed-up vs. sequential A*', fontsize=8)
        if r == 2: ax.set_xlabel('threads')
legend_below(fig, axs); fig.tight_layout(rect=(0, 0.045, 1, 1)); fig.savefig(f'{out}/fig6_astar.pdf'); plt.close(fig)

# ---- fig7: search overhead
fig, axs = plt.subplots(1, 3, figsize=(9, 2.9), sharey=True)
for ax, (p, name, hw) in zip(axs, PLAT):
    a = rd('astar', p); a = a[a.heuristic == 'euclid']
    seq = a[a.version == 'seq-binary-heap'].avg_expanded.iloc[0]
    for i, v in enumerate(['list-lazy-backoff', 'heap-backoff', 'hda', 'hda-window']):
        g = a[a.version == v].sort_values('threads')
        ax.plot(g.threads, g.avg_expanded / seq, color=C[[3, 4, 5, 6][i]], marker=MK[[3, 4, 5, 6][i]],
                ls='--' if v.startswith('hda') else '-', label=VER[v])
    xaxis(ax, hw); ax.set_yscale('log'); ax.set_title(name, fontsize=8.5); ax.set_xlabel('threads')
axs[0].set_ylabel('expansions / sequential (log)')
legend_below(fig, axs); fig.tight_layout(rect=(0, 0.09, 1, 1)); fig.savefig(f'{out}/fig7_overhead.pdf'); plt.close(fig)

# ---- fig8: cross-platform scaling
fig, axs = plt.subplots(1, 3, figsize=(9, 2.9))
for i, (p, name, hw) in enumerate(PLAT):
    lab = name.split(' (')[0]
    c = rd('clist', p); g = c[(c.list == 'lazy-backoff') & (c.mix == 'read')].sort_values('threads')
    axs[0].plot(g.threads, g.mops / g.mops.iloc[0], color=C[i], marker=MK[i], label=lab)
    g = c[(c.list == 'coarse-backoff') & (c.mix == 'pq')].sort_values('threads')
    axs[1].plot(g.threads, g.mops / g.mops.iloc[0], color=C[i], marker=MK[i], label=lab)
    a = rd('astar', p); a = a[a.heuristic == 'euclid-costly']; seq = a[a.version == 'seq-binary-heap'].total_ms.iloc[0]
    best = a[a.version != 'seq-binary-heap'].groupby('threads').total_ms.min()
    axs[2].plot(best.index, seq / best.values, color=C[i], marker=MK[i], label=lab)
axs[0].set_title('lazy list, 90% contains', fontsize=8.5); axs[0].set_ylabel('throughput vs. 1 thread')
axs[1].set_title('coarse list, add / removeMin', fontsize=8.5); axs[1].set_ylabel('throughput vs. 1 thread')
axs[2].set_title('parallel A*, costly $h$: best version', fontsize=8.5); axs[2].set_ylabel('speed-up vs. sequential')
for ax in axs: xaxis(ax); ax.axhline(1, color='#52514e', lw=0.8); ax.set_xlabel('threads'); ax.set_ylim(bottom=0)
legend_below(fig, axs); fig.tight_layout(rect=(0, 0.09, 1, 1)); fig.savefig(f'{out}/fig8_cross.pdf'); plt.close(fig)
# ---- fig9: time-dependent travel time vs. departure time (lecturer's speed profile)
sw = rd('seq_td_sweep', PLAT[2][0])
fig, ax = plt.subplots(figsize=(9, 2.7))
ax.plot(sw.depart_min, sw.optimal_min, color=C[0], marker='o', ms=3, label='optimal (TD-A*, $h = d / v_{\\max}$)')
ax.plot(sw.depart_min, sw.lecturer_h_min, color=C[1], marker='s', ms=3, ls='--', label="TD-A* with the lecturer's $h = d / \\mathit{speed}(t)$")
ax.plot(sw.depart_min, sw.departure_speed_model_min, color='#8a8984', ls=':', label='same route, speed at departure only (not FIFO)')
ax.set_xlabel('departure time (min)'); ax.set_ylabel('travel time (min)'); ax.legend(fontsize=7.5, ncol=1, loc='upper right')
fig.tight_layout(); fig.savefig(f'{out}/fig9_td.pdf'); plt.close(fig)
print('figures written to', out)
