#!/bin/bash
# 截取当前聚焦窗口（GNOME Wayland 可用）。
# 用法：bash screenshot_window.sh /绝对路径/output.png
set -e
OUT="$1"
if [ -z "$OUT" ]; then
    echo "usage: $0 /absolute/path/output.png" >&2
    exit 1
fi
gdbus call --session \
    --dest org.gnome.Shell \
    --object-path /org/gnome/Shell/Screenshot \
    --method org.gnome.Shell.Screenshot.ScreenshotWindow \
    true false true "$OUT"
