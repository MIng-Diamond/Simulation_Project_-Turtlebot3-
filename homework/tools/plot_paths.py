#!/usr/bin/env python3
"""把 record_path.py 生成的 CSV 画成多条路径。"""

import argparse
import csv
from collections import defaultdict

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--title', default='Nav2 global paths')
    args = parser.parse_args()

    paths = defaultdict(list)
    with open(args.input, newline='') as f:
        for row in csv.DictReader(f):
            paths[int(row['path_id'])].append(
                (float(row['x']), float(row['y'])))

    plt.figure(figsize=(7, 7))
    for path_id in sorted(paths):
        pts = paths[path_id]
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        plt.plot(xs, ys, marker='.', linewidth=1.2,
                 label=f'path {path_id}')
        if pts:
            plt.plot(xs[0], ys[0], 'go', markersize=5)
            plt.plot(xs[-1], ys[-1], 'rx', markersize=7)
    plt.axis('equal')
    plt.grid(True, linestyle='--', alpha=0.4)
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.title(args.title)
    plt.legend(fontsize=8)
    plt.tight_layout()
    plt.savefig(args.output, dpi=150)
    print('saved', args.output)


if __name__ == '__main__':
    main()
