#!/usr/bin/env python3
"""读取 nav2_map_server 保存的 PGM，统计并渲染成 PNG。

用法：
  python3 plot_map.py ~/maps/tb3_world.pgm \
      --output ~/turtlebot3_ws/homework/maps/tb3_world.png
"""

import argparse
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


def read_pgm(path):
    with open(path, 'rb') as f:
        magic = f.readline().strip()
        if magic != b'P5':
            raise ValueError(f'unsupported PGM magic: {magic}')
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        width, height = map(int, line.split())
        maxval = int(f.readline())
        data = np.frombuffer(f.read(), dtype=np.uint8)
        if data.size != width * height:
            raise ValueError('unexpected PGM data size')
        return data.reshape((height, width)), maxval


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('pgm')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()

    img, maxval = read_pgm(args.pgm)
    values, counts = np.unique(img, return_counts=True)
    total = img.size
    print(f'file: {args.pgm}')
    print(f'size: {img.shape[1]} x {img.shape[0]}  maxval={maxval}')
    print('pixel histogram:')
    for v, c in zip(values, counts):
        print(f'  value {v:3d}: {c:8d} ({100.0 * c / total:5.1f}%)')

    # nav2 map_saver: 0=occupied, 254/255=free, 205=unknown
    free = int(np.count_nonzero(img >= 250))
    occupied = int(np.count_nonzero(img <= 10))
    unknown = total - free - occupied
    print(f'free={free} ({100.0 * free / total:.1f}%)  '
          f'occupied={occupied} ({100.0 * occupied / total:.1f}%)  '
          f'unknown={unknown} ({100.0 * unknown / total:.1f}%)')

    # 渲染：占据=黑，空闲=白，未知=灰
    rgb = np.zeros((img.shape[0], img.shape[1], 3), dtype=float)
    rgb[img >= 250] = (1.0, 1.0, 1.0)
    rgb[img <= 10] = (0.0, 0.0, 0.0)
    mask_unknown = (img > 10) & (img < 250)
    rgb[mask_unknown] = (0.6, 0.6, 0.6)

    dpi = 120
    figsize = (img.shape[1] / dpi, img.shape[0] / dpi)
    plt.figure(figsize=figsize, dpi=dpi)
    plt.imshow(rgb, origin='lower')
    plt.axis('off')
    plt.tight_layout(pad=0)
    out_dir = os.path.dirname(os.path.abspath(args.output))
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    plt.savefig(args.output, dpi=dpi, bbox_inches='tight', pad_inches=0)
    print('saved', args.output)


if __name__ == '__main__':
    main()
