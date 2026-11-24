"""Regenerate the LaTeX tables.
Usage: python3 make_tables.py <results_city_dir> <results_part2_dir> <tables_dir>"""
import os, sys
import pandas as pd
res, res2, out = sys.argv[1], sys.argv[2], sys.argv[3]; os.makedirs(out, exist_ok=True)
PLAT = [('macos-native-apple-m5-10c', 'Platform A (macOS, Apple M5)'),
        ('apple-silicon-arm64-4vcpu', 'Platform B (Linux VM, 4 vCPU)'),
        ('cloud-x86-2vcpu', 'Platform C (Linux, Xeon, 2 vCPU)')]
TS = [1, 2, 4, 8, 16]
def rd(kind, p):
    d = pd.read_csv(f'{res2 if kind == "clist" else res}/{kind}_{p}.csv')
    return d.drop_duplicates(subset=['version', 'heuristic', 'threads'], keep='first') if kind == 'astar' else d
def tt(s): return '\\texttt{' + s.replace('_', '\\_') + '}'

# ---------- Table: sequential algorithms
A, B, Cc = (rd('seq_heuristics', p) for p, _ in PLAT)
rows = [('length', 'dijkstra', 'Dijkstra'), ('length', 'astar-euclid', 'A*, Euclidean $h$'),
        ('length', 'astar-alt4', 'A*, ALT 4 landmarks'), ('length', 'astar-alt8', 'A*, ALT 8 landmarks'),
        ('length', 'astar-alt16', 'A*, ALT 16 landmarks'),
        ('time', 'dijkstra', 'Dijkstra'), ('time', 'astar-euclid', 'A*, Euclidean $h$')]
L = ['\\begin{tabular}{ll rr rrr}', '\\toprule',
     ' & & \\multicolumn{2}{c}{expanded nodes} & \\multicolumn{3}{c}{time per query (ms)} \\\\',
     '\\cmidrule(lr){3-4}\\cmidrule(lr){5-7}',
     'Metric & Algorithm & A & B, C & A & B & C \\\\', '\\midrule']
for i, (m, a, lab) in enumerate(rows):
    if i == 5: L.append('\\midrule')
    g = lambda d: d[(d.metric == m) & (d.algorithm == a) & (d.eps == 0)].iloc[0]
    L.append(f'{m if i in (0, 5) else ""} & {lab} & {g(A).avg_expanded:,.0f} & {g(B).avg_expanded:,.0f} & '
             f'{g(A).avg_ms:.3f} & {g(B).avg_ms:.3f} & {g(Cc).avg_ms:.3f} \\\\'.replace(',', '{,}'))
L += ['\\bottomrule', '\\end{tabular}']
open(f'{out}/tab_seq.tex', 'w').write('\n'.join(L) + '\n')

# ---------- Table: uncertain / dynamic heuristics (platform C)
d = Cc[(Cc.metric == 'length') & Cc.algorithm.isin(['astar-uncertain', 'astar-dynamic'])]
L = ['\\begin{tabular}{r rrrr rrrr}', '\\toprule',
     ' & \\multicolumn{4}{c}{uncertain $h$} & \\multicolumn{4}{c}{dynamic $h$} \\\\',
     '\\cmidrule(lr){2-5}\\cmidrule(lr){6-9}',
     '$\\varepsilon$ & mean & worst & expanded & re-opened & mean & worst & expanded & re-opened \\\\', '\\midrule']
for e in sorted(d.eps.unique()):
    u = d[(d.eps == e) & (d.algorithm == 'astar-uncertain')].iloc[0]; v = d[(d.eps == e) & (d.algorithm == 'astar-dynamic')].iloc[0]
    L.append(f'{e:.1f} & {u.avg_ratio:.3f} & {u.max_ratio:.3f} & {u.avg_expanded:,.0f} & {u.avg_reopened:,.0f} & '
             f'{v.avg_ratio:.3f} & {v.max_ratio:.3f} & {v.avg_expanded:,.0f} & {v.avg_reopened:,.0f} \\\\'.replace(',', '{,}'))
L += ['\\bottomrule', '\\end{tabular}']
open(f'{out}/tab_eps.tex', 'w').write('\n'.join(L) + '\n')

# ---------- Table: concurrent lists
LISTS = [('coarse', 'Coarse'), ('fine', 'Fine-grained'), ('optimistic', 'Optimistic'), ('lazy', 'Lazy')]
L = ['\\begin{tabular}{l' + 'r' * 10 + '}', '\\toprule',
     ' & \\multicolumn{5}{c}{90\\% \\texttt{contains} (threads)} & \\multicolumn{5}{c}{\\texttt{add}/\\texttt{removeMin} (threads)} \\\\',
     '\\cmidrule(lr){2-6}\\cmidrule(lr){7-11}',
     'List / lock & ' + ' & '.join(map(str, TS * 2)) + ' \\\\']
for p, name in PLAT:
    c = rd('clist', p)
    L += ['\\midrule', f'\\multicolumn{{11}}{{l}}{{\\emph{{{name}}}}} \\\\']
    for fam, flab in LISTS:
        for lk, llab in [('mutex', tt('std::mutex')), ('backoff', 'TTAS+backoff')]:
            vals = []
            for mix in ['read', 'pq']:
                for T in TS:
                    x = c[(c.list == f'{fam}-{lk}') & (c.mix == mix) & (c.threads == T)]
                    vals.append(f'{x.mops.iloc[0]:.1f}' if len(x) else '--')
            L.append(f'{flab}, {llab} & ' + ' & '.join(vals) + ' \\\\')
L += ['\\bottomrule', '\\end{tabular}']
open(f'{out}/tab_clist.tex', 'w').write('\n'.join(L) + '\n')

# ---------- Table: parallel A*
VER = [('seq-binary-heap', 'sequential A*'), ('list-coarse-backoff', 'list: coarse'), ('list-fine-backoff', 'list: fine'),
       ('list-optimistic-backoff', 'list: optimistic'), ('list-lazy-backoff', 'list: lazy'),
       ('list-lazy-mutex', 'list: lazy, ' + tt('std::mutex')), ('heap-backoff', 'shared heap'),
       ('heap-mutex', 'shared heap, ' + tt('std::mutex')), ('hda', 'HDA*'), ('hda-window', 'HDA* + $f$-window')]
L = ['\\begin{tabular}{l' + 'r' * 10 + '}', '\\toprule',
     ' & \\multicolumn{5}{c}{Euclidean $h$ (threads)} & \\multicolumn{5}{c}{costly $h$ (threads)} \\\\',
     '\\cmidrule(lr){2-6}\\cmidrule(lr){7-11}',
     'Version & ' + ' & '.join(map(str, TS * 2)) + ' \\\\']
for p, name in PLAT:
    a = rd('astar', p)
    L += ['\\midrule', f'\\multicolumn{{11}}{{l}}{{\\emph{{{name}}}}} \\\\']
    for v, lab in VER:
        vals = []
        for H in ['euclid', 'euclid-costly']:
            x = a[(a.heuristic == H) & (a.version == v)]
            seq = a[(a.heuristic == H) & (a.version == 'seq-binary-heap')].total_ms.iloc[0]
            for T in TS:
                y = x[x.threads == T]
                if v.startswith('seq'):
                    vals.append(f'{seq:.0f}' if T == 1 else '')
                elif len(y):
                    s = f'{y.total_ms.iloc[0]:.0f}'
                    vals.append('\\textbf{' + s + '}' if y.total_ms.iloc[0] < seq else s)
                else:
                    vals.append('--')
        L.append(f'{lab} & ' + ' & '.join(vals) + ' \\\\')
L += ['\\bottomrule', '\\end{tabular}']
open(f'{out}/tab_astar.tex', 'w').write('\n'.join(L) + '\n')
# ---------- Table: time-dependent A* (lecturer's model)
NAMES = [('zero (TD-Dijkstra)', '$h = 0$ (time-dependent Dijkstra)'), ('great-circle / vmax', '$h = d_{\\mathrm{gc}} / v_{\\max}$'),
         ('exact length / vmax', '$h = d_{\\mathrm{road}} / v_{\\max}$ (one reverse Dijkstra)'),
         ('lecturer: great-circle / speed(t)', "$h = d_{\\mathrm{gc}} / \\mathit{speed}(t)$ (lecturer)")]
L = ['\\begin{tabular}{l rrrrr r}', '\\toprule',
     'Heuristic & ms (A) & ms (C) & expanded & re-opened & mean & worst \\\\', '\\midrule']
TA, TC = rd('seq_td', PLAT[0][0]).set_index('heuristic'), rd('seq_td', PLAT[2][0]).set_index('heuristic')
for k, lab in NAMES:
    a, c = TA.loc[k], TC.loc[k]
    L.append(f'{lab} & {a.avg_ms:.1f} & {c.avg_ms:.1f} & {c.avg_expanded:,.0f} & {c.avg_reopened:,.0f} & {c.avg_ratio:.3f} & {c.max_ratio:.3f} \\\\'.replace(',', '{,}'))
L += ['\\bottomrule', '\\end{tabular}']
open(f'{out}/tab_td.tex', 'w').write('\n'.join(L) + '\n')
print('tables written to', out)
