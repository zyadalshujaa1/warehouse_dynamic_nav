#!/usr/bin/env python3
"""Summarise results/summary.csv per scenario and method.

Rules (so every number in the table can be traced):
- One row per (scenario, method, run): if a trial was repeated, the LAST row wins.
- Trials with outcome 'startup_failed' are not navigation results; they are
  counted separately and left out of every statistic.
- min clearance median is taken over all valid trials of that cell.

Usage: python3 summarize.py ~/logistic_ws/results/summary.csv
"""
import csv
import statistics
import sys
from collections import OrderedDict, defaultdict


def main(path):
    last = OrderedDict()
    with open(path) as f:
        for row in csv.DictReader(f):
            last[(row['scenario'], row['method'], row['run'])] = row

    cells = defaultdict(list)
    startup_failed = []
    for (scen, meth, run), row in last.items():
        if row['outcome'] == 'startup_failed':
            startup_failed.append(f'{scen}_{meth}_{run}')
            continue
        cells[(scen, meth)].append(row)

    head = f'{"scenario":11s} {"method":8s} {"n":>2s} {"collision":>9s} {"succeeded":>9s} ' \
           f'{"timeout":>7s} {"other":>5s} {"min_clear_med":>13s} {"time_ok_med":>11s}'
    print(head)
    print('-' * len(head))
    for (scen, meth), rows in sorted(cells.items()):
        out = [r['outcome'] for r in rows]
        clear = [float(r['min_clearance_m']) for r in rows if r['min_clearance_m'] != '']
        t_ok = [float(r['mission_time_s']) for r in rows if r['outcome'] == 'succeeded']
        other = len(out) - out.count('collision') - out.count('succeeded') - out.count('timeout')
        print(f'{scen:11s} {meth:8s} {len(rows):2d} {out.count("collision"):9d} '
              f'{out.count("succeeded"):9d} {out.count("timeout"):7d} {other:5d} '
              f'{statistics.median(clear) if clear else float("nan"):13.2f} '
              f'{statistics.median(t_ok) if t_ok else float("nan"):11.1f}')
    not_moved = [f'{k[0]}_{k[1]}_{k[2]}' for k, r in last.items()
                 if r['outcome'] != 'startup_failed' and r['worker_moved'] == 'False']
    if not_moved:
        print('\nworker never moved (robot did not reach the scenario):', ', '.join(not_moved))
    if startup_failed:
        print('start-up failed (excluded):', ', '.join(startup_failed))


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else 'summary.csv')
