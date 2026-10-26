#!/bin/bash
# 停止 TurtleBot3 / Gazebo Classic 相关进程，并清理 FastDDS 残留。
#
# 只处理 Gazebo Classic 和 turtlebot3/tb3_course 的进程，不会杀
# TurtleBot4 的 Ignition 仿真（ign gazebo / ros_gz / turtlebot4）。
#
# 用法：bash ~/turtlebot3_ws/stop_tb3.sh

python3 - <<'PY'
import glob
import os
import signal
import time

LAUNCH_MARKERS = (
    'turtlebot3_gazebo', 'turtlebot3_cartographer',
    'turtlebot3_navigation2', 'turtlebot3_teleop',
    'turtlebot3_fake_node', 'turtlebot3_bringup',
    'tb3_course', 'multi_robot_2.launch.py',
    'turtlesim', 'gzserver', 'gzclient',
    'tb3_navigation2.rviz', 'tb3_cartographer.rviz',
    'cartographer_map.rviz',
)

ORPHAN_MARKERS = (
    'gzserver', 'gzclient', 'spawn_entity.py',
    'turtlebot3', 'tb3_course', 'turtlesim',
    'tb3_navigation2.rviz', 'tb3_cartographer.rviz',
    'cartographer_map.rviz',
    'cartographer_node', 'cartographer_occupancy_grid_node',
    'nav2_', 'amcl', 'map_server', 'planner_server',
    'controller_server', 'bt_navigator', 'behavior_server',
    'waypoint_follower', 'velocity_smoother', 'lifecycle_manager',
    '__ns:=/TB3_',
)


def read_procs():
    out = {}
    for e in os.listdir('/proc'):
        if not e.isdigit():
            continue
        pid = int(e)
        try:
            with open(f'/proc/{pid}/stat', 'rb') as f:
                stat = f.read().decode('utf-8', 'replace')
            ppid = int(stat[stat.rindex(')') + 2:].split()[1])
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                cmd = f.read().decode('utf-8', 'replace').replace('\x00', ' ').strip()
        except (OSError, ValueError, IndexError):
            continue
        if cmd:
            out[pid] = {'ppid': ppid, 'cmd': cmd}
    return out


# 绝不杀自己和祖先
mine, p = set(), os.getpid()
while p > 1:
    mine.add(p)
    try:
        with open(f'/proc/{p}/stat', 'rb') as f:
            st = f.read().decode('utf-8', 'replace')
        p = int(st[st.rindex(')') + 2:].split()[1])
    except (OSError, ValueError, IndexError):
        break
mine.add(os.getpid())

procs = read_procs()
kids = {}
for pid, info in procs.items():
    kids.setdefault(info['ppid'], []).append(pid)

roots = [
    pid for pid, info in procs.items()
    if pid not in mine and any(m in info['cmd'] for m in LAUNCH_MARKERS)
]
victims = set()
for root in roots:
    stack = [root]
    while stack:
        cur = stack.pop()
        if cur in victims:
            continue
        victims.add(cur)
        stack.extend(kids.get(cur, []))

for pid, info in procs.items():
    if pid in mine or pid in victims:
        continue
    if info['ppid'] in (1, 1870) and any(
            m in info['cmd'] for m in ORPHAN_MARKERS):
        victims.add(pid)

# 不要误杀 Ignition/TurtleBot4
victims = {
    pid for pid in victims
    if not any(x in procs[pid]['cmd']
               for x in ('ign gazebo', 'ros_gz', 'turtlebot4'))
}

if not victims:
    print('没有正在运行的 TurtleBot3/Gazebo Classic 进程')
else:
    print(f'停止 {len(victims)} 个进程...')
    for sig in (signal.SIGTERM, signal.SIGKILL):
        for pid in victims:
            try:
                os.kill(pid, sig)
            except OSError:
                pass
        time.sleep(2)

# 只有确认没有 ROS/Gazebo 进程时才清理共享内存，避免影响 TB4
remaining = [
    pid for pid, info in read_procs().items()
    if ('opt/ros/humble' in info['cmd'] or 'gzserver' in info['cmd']
        or 'ign gazebo' in info['cmd'])
    and 'ros2-daemon' not in info['cmd']
]
if not remaining:
    n = 0
    for f in glob.glob('/dev/shm/fastrtps_*') + glob.glob('/dev/shm/sem.fastrtps_*'):
        try:
            os.unlink(f)
            n += 1
        except OSError:
            pass
    print(f'清理 {n} 个 FastDDS 共享内存文件')
else:
    print(f'仍有 {len(remaining)} 个 ROS/Gazebo 进程，跳过共享内存清理')
PY

echo "剩余 TB3/Gazebo Classic 进程: $(ps -eo cmd --no-headers | grep -cE 'turtlebot3|gzserver|gzclient|tb3_course' || true)"
echo "DDS 端口占用: $(ss -ulpn 2>/dev/null | grep -cE ':74[0-9][0-9]|:75[0-9][0-9]' || true)"
