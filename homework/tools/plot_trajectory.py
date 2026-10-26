#!/usr/bin/env python3
"""把 record_trajectory.py 生成的 CSV 画成 x-y 轨迹图。

用法：
  python3 plot_trajectory.py --input a.csv [b.csv ...] \
      --output trajectory.png --title "Circle r=0.5"
"""

import argparse
import csv

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402


def read_csv(path):
    xs, ys = [], []
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            xs.append(float(row['x']))
            ys.append(float(row['y']))
    return xs, ys


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', nargs='+', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--title', default='Trajectory')
    parser.add_argument('--offset_x', type=float, default=0.0)
    parser.add_argument('--offset_y', type=float, default=0.0)
    args = parser.parse_args()

    plt.figure(figsize=(6, 6))
    for path in args.input:
        xs, ys = read_csv(path)
        if not xs:
            continue
        xs = [x + args.offset_x for x in xs]
        ys = [y + args.offset_y for y in ys]
        plt.plot(xs, ys, linewidth=1.5, label=path.split('/')[-1])
        plt.plot(xs[0], ys[0], 'go', markersize=5)
        plt.plot(xs[-1], ys[-1], 'rx', markersize=7)
    plt.axis('equal')
    plt.grid(True, linestyle='--', alpha=0.4)
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.title(args.title)
    if len(args.input) > 1:
        plt.legend(fontsize=8)
    plt.tight_layout()
    plt.savefig(args.output, dpi=150)
    print('saved', args.output)


if __name__ == '__main__':
    main()
