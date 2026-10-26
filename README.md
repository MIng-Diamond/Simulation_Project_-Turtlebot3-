# Simulation Project - TurtleBot3

基于 **Ubuntu 22.04 + ROS 2 Humble + Gazebo Classic 11.10.2** 的 TurtleBot3
仿真课程作业仓库。

本仓库只包含 TurtleBot3 工作空间（`turtlebot3_ws`），不包含其他机器人
或其他工作空间。

## 项目概览

- 课程功能包：`src/tb3_course`（10 个节点 + 双车 launch）
- 覆盖教程第二~六节：ROS2 基础、Gazebo、Cartographer SLAM、Nav2、多机器人
- 作业产物：实验报告、地图、截图、轨迹数据、演示视频
- 真机环境实测，所有结果来自本机 Gazebo Classic 仿真

## 主要结果

| 实验 | 结果 |
|---|---|
| turtlesim 画圆 | 参数 `radius=0.5` 实测平均半径 0.499 m；`radius=2.0` 实测 1.997 m |
| 开环正方形 | 0.36 m 边长，闭合误差约 **0.344 m** |
| 闭环正方形（`/odom`） | 0.40 m 边长，闭合误差约 **0.005 m** |
| Cartographer 地图 | 124 × 115 @ 0.05 m，自由 55.3%，占据 5.6%，六边形墙和 9 个圆柱闭合 |
| Nav2 | 3 个目标全部 `SUCCEEDED`；两圈 8 目标巡逻完成 |
| 多机器人 | `/TB3_1`、`/TB3_2` 话题/TF 正常；镜像、距离、跟随均有实测 |
| 镜像/距离 | `scale=0.9` 镜像有效；静止距离 2.820 m；0.5 s 看门狗停车 |
| Leader-Follower | follower 追到 0.622 m；急转弯切内道，已加 0.45 m 安全停车 |

## 演示视频

视频在 `homework/videos/`，点击可直接播放/下载：

| 视频 | 内容 | 时长 |
|---|---|---|
| [画圆](homework/videos/02_circle.mp4) | turtlesim，r=0.5 | 8 s |
| [闭环正方形](homework/videos/03_square_odom.mp4) | Gazebo，0.4 m 边长 | 25 s |
| [建图片段](homework/videos/04_mapping.mp4) | Cartographer 建图过程 | 40 s |
| [完整建图加速版](homework/videos/04_mapping_full.mp4) | 完整一圈，8× 加速 | 29.4 s |
| [Nav2 导航](homework/videos/05_nav2.mp4) | RViz2 全局路径 + 到达目标 | 20 s |

每个视频都有对应的截帧 PNG，已插入 `homework/report/report.md` 的对应位置。

## 关键图片

**Cartographer 建图结果**（完整六边形 + 9 个圆柱）：

![Cartographer map](homework/maps/tb3_world.png)

**Nav2 运行截图**（全局路径、代价地图、目标）：

![Nav2](homework/screenshots/05_nav2_goals.png)

**两台 TurtleBot3**（`/TB3_1`、`/TB3_2`）：

![Multi robot](homework/screenshots/06_multi_robot.png)

## 目录结构

```text
turtlebot3_ws/
├── src/tb3_course/
│   ├── tb3_course/              # 10 个 Python 节点
│   ├── launch/
│   │   └── multi_robot_2.launch.py
│   ├── package.xml
│   └── setup.py
├── homework/
│   ├── report/
│   │   ├── report.md            # Markdown 报告（图片/视频链接）
│   │   └── report.docx          # Word 报告
│   ├── videos/                  # 演示视频 + 截帧
│   ├── screenshots/             # RViz2 / Gazebo / rqt_graph 截图
│   ├── maps/                    # PGM / YAML / PNG
│   ├── data/                    # 轨迹 CSV、路径 CSV、实验日志
│   ├── tools/                   # 录屏、绘图、轨迹记录、md->docx
│   └── rviz/                    # 自定义 RViz 配置
├── tb3_env.sh                   # 每个终端先 source
├── install_tb3.sh               # apt 安装 TB3
├── build_tb3.sh                 # colcon build
├── start_world.sh
├── start_cartographer.sh
├── start_nav2.sh
├── start_teleop.sh
├── start_multi_robot.sh
├── stop_tb3.sh                  # 清理 TB3/Gazebo Classic 进程
└── README.md
```

## 快速开始

### 1. 安装依赖

```bash
sudo apt install -y \
  ros-humble-turtlebot3 \
  ros-humble-turtlebot3-gazebo \
  ros-humble-turtlebot3-fake-node
```

### 2. 每个新终端 source 环境

```bash
source ~/turtlebot3_ws/tb3_env.sh
```

> `tb3_env.sh` 会补上 `GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models`。
> 不加时 `gazebo_ros` 找不到 `ground_plane` / `sun`，会尝试联网下载并卡住，
> `/spawn_entity` 服务不出现，机器人 spawn 失败。

### 3. 编译

```bash
cd ~/turtlebot3_ws
colcon build --symlink-install --packages-select tb3_course
source install/setup.bash
```

或：

```bash
bash build_tb3.sh
```

### 4. 启动

```bash
# 终端 1：世界
bash start_world.sh                 # 默认 turtlebot3_world
bash start_world.sh empty_world     # 空世界

# 终端 2：Cartographer
bash start_cartographer.sh

# 终端 3：遥控
bash start_teleop.sh

# Nav2（需要地图）
bash start_nav2.sh

# 两台机器人
bash start_multi_robot.sh
bash start_multi_robot.sh empty_world.world
```

### 5. 停机

```bash
bash stop_tb3.sh
```

只清理 TurtleBot3 / Gazebo Classic 进程；如反复启停后 DDS 端口异常，
先确认没有 ROS 进程，再执行本脚本。

## 节点与作业对应

| 节点 | 作用 | 对应作业 |
|---|---|---|
| `circle` | turtlesim 画圆，`radius` 参数可调 | 2.9 |
| `pose_listener` | 订阅 `/turtle1/pose` | 2.8 |
| `move_square` | 开环近似正方形 | 3.7 / 3.9 |
| `square_odom` | 基于 `/odom` 闭环正方形 | 3.9 拓展 |
| `nav_goal` | 单个 Nav2 `NavigateToPose` 目标 | 5.5 / 5.9 |
| `patrol` | 多点巡航，`loops` 控制圈数 | 5.6 / 5.9 |
| `velocity_mirror` | `/TB3_1/cmd_vel` 镜像到 `/TB3_2/cmd_vel` | 6.6 / 6.9 |
| `robot_distance` | 两车 odom 欧氏距离，发布 `/tb3_distance` | 6.9 |
| `leader_follower` | 闭环 P 控制器跟随 leader | 6.7 / 6.9 |
| `auto_mapper` | 程序化巡航建图（外圈 + 中心走廊） | 4.7 |

查看可执行文件：

```bash
ros2 pkg executables tb3_course
```

## 已知坑与处理

1. **Gazebo 卡在加载世界**：`GAZEBO_MODEL_PATH` 缺少系统模型路径。
   已由 `tb3_env.sh` 修复。
2. **最后一条 `/cmd_vel` 会保持**：运动结束后必须补零，否则机器人会一直
   前进；`velocity_mirror` 已加 `cmd_timeout=0.5 s` 看门狗。
3. **Nav2 目标时间戳过旧**：`patrol`/`nav_goal` 的 `header.stamp` 设为 0，
   让 Nav2 使用最新 TF，避免 `Extrapolation Error`。
4. **自动建图路线不能贴墙**：`turtlebot3_world` 墙在 x≈±2.2 m，
   路线内收到 ±1.65 m；雷达扇区按 0..2π 正确归一化。
5. **RViz 窗口高度为奇数时 H.264 编码失败**：`record_window.py` 已加
   `pad=ceil(iw/2)*2:ceil(ih/2)*2`。

## 实验报告

- [Markdown 报告](homework/report/report.md)（图片和视频链接可直接查看）
- [Word 报告](homework/report/report.docx)

报告包含第二~六节全部作业、原理说明、实测数据和截图。

## 说明

- 所有实验数据均来自本机实际运行，未使用编造数据。
- 本仓库未启用 Git LFS：仓库约 15 MB，最大单文件约 3.3 MB，远小于
  GitHub 100 MB 限制；如果后续加入 rosbag 或更大的视频，建议使用
  `git lfs track "*.bag"`。
- 代码遵循 `src/tb3_course/LICENSE` 中的 Apache-2.0；报告、视频和截图
  为课程作业产物。
