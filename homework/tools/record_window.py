#!/usr/bin/env python3
"""录一个 X11/XWayland 窗口为 MP4。

原理：xwd 直接读取窗口内容，逐帧转 PNG，再用 ffmpeg 合成。
适合 GNOME Wayland + XWayland 下的 Gazebo Classic / RViz2 / turtlesim。

用法：
  python3 record_window.py --title "RViz" --duration 20 --fps 5 \
      --output ~/turtlebot3_ws/homework/videos/05_nav2.mp4

注意：xwd 读取的是窗口当前显示区域，录制前请确保目标窗口在最前面
（不要被 Gazebo/终端挡住）。脚本会自动选面积最大的匹配窗口。
"""

import argparse
import os
import re
import shutil
import subprocess
import tempfile
import time


def find_window(title_substr, display=':0'):
    env = dict(os.environ, DISPLAY=display)
    out = subprocess.check_output(
        ['xwininfo', '-root', '-tree'], text=True, env=env,
        stderr=subprocess.DEVNULL)
    candidates = []
    for line in out.splitlines():
        if title_substr.lower() not in line.lower():
            continue
        m = re.search(r'\s(\d+)x(\d+)\+', line)
        if not m:
            continue
        area = int(m.group(1)) * int(m.group(2))
        wid = line.split()[0]
        candidates.append((area, wid, line.strip()))
    if not candidates:
        raise RuntimeError(f'window not found: {title_substr!r}')
    candidates.sort(reverse=True)
    return candidates[0][1], candidates[0][2]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--title', required=True,
                        help='window title substring, e.g. "RViz"')
    parser.add_argument('--duration', type=float, default=15.0)
    parser.add_argument('--fps', type=float, default=5.0,
                        help='capture rate')
    parser.add_argument('--playback-fps', type=float, default=0.0,
                        help='0 = real time; >0 = timelapse playback fps')
    parser.add_argument('--output', required=True)
    parser.add_argument('--display', default=':0')
    args = parser.parse_args()

    wid, desc = find_window(args.title, args.display)
    print(f'window: {wid}  {desc}')

    out_dir = os.path.dirname(os.path.abspath(args.output))
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    env = dict(os.environ, DISPLAY=args.display)
    tmp = tempfile.mkdtemp(prefix='winrec_')
    frame = 0
    start = time.monotonic()
    period = 1.0 / max(0.5, args.fps)
    try:
        while True:
            now = time.monotonic()
            if now - start >= args.duration:
                break
            xwd = os.path.join(tmp, 'frame.xwd')
            png = os.path.join(tmp, f'frame_{frame:05d}.png')
            r = subprocess.run(['xwd', '-id', wid, '-out', xwd],
                               env=env, stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL)
            if r.returncode != 0:
                print('xwd failed, stop')
                break
            subprocess.run(
                ['ffmpeg', '-y', '-loglevel', 'error', '-i', xwd, png],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            os.unlink(xwd)
            frame += 1
            elapsed = time.monotonic() - start
            next_t = start + frame * period
            if next_t > time.monotonic():
                time.sleep(next_t - time.monotonic())
            if elapsed > args.duration + 5.0:
                break

        if frame == 0:
            raise RuntimeError('no frames captured')
        elapsed = time.monotonic() - start
        real_fps = frame / elapsed
        out_fps = args.playback_fps if args.playback_fps > 0 else real_fps
        print(f'captured {frame} frames in {elapsed:.1f}s '
              f'(actual {real_fps:.2f} fps, playback {out_fps:.2f} fps)')
        subprocess.run([
            'ffmpeg', '-y', '-loglevel', 'error',
            '-framerate', f'{out_fps:.4f}',
            '-i', os.path.join(tmp, 'frame_%05d.png'),
            '-vf', 'pad=ceil(iw/2)*2:ceil(ih/2)*2',
            '-c:v', 'libx264', '-pix_fmt', 'yuv420p',
            '-movflags', '+faststart', args.output,
        ], check=True)
        print('saved', args.output)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


if __name__ == '__main__':
    main()
