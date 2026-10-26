# 第 6 章：工程化、调试与 Capstone 项目

> 本章目标：把“能跑通”升级为“能复现、能调试、能独立设计项目”。
> 你会学会组织 ROS2 包、写 launch/参数、做实验记录、用 git 管理代码、
> 设计指标和消融，最后从 4 个 Capstone 方向里选一个独立完成。

---

## 6.1 一个 ROS2 项目的最小工程规范

### 6.1.1 目录

```text
turtlebot3_ws/
├── src/tb3_course/       # 源码（提交 git）
├── homework/             # 报告、数据、视频（提交 git）
├── build/ install/ log/  # 编译产物（.gitignore）
├── tb3_env.sh            # 环境入口
├── start_*.sh            # 启动脚本
├── stop_tb3.sh           # 停机/清理
└── README.md             # 项目说明
```

### 6.1.2 功能包

```text
src/tb3_course/
├── package.xml           # 包名/依赖/许可证
├── setup.py              # entry_points + data_files
├── setup.cfg
├── resource/tb3_course
├── tb3_course/           # Python 节点
├── launch/               # launch 文件
└── test/                 # 测试
```

`setup.py` 里两件事最重要：

```python
data_files=[
    ('share/ament_index/resource_index/packages', ['resource/tb3_course']),
    ('share/tb3_course', ['package.xml']),
    ('share/tb3_course/launch', glob('launch/*.launch.py')),
],
entry_points={
    'console_scripts': [
        'circle = tb3_course.turtle_circle:main',
        'trail_follower = tb3_course.leader_follower_path:main',
    ],
},
```

- 忘了 `data_files` → `ros2 launch` 找不到 launch；
- 忘了 entry point → `ros2 run` 找不到可执行文件；
- 改了 `setup.py` → 必须重新 `colcon build` + source。

### 6.1.3 构建

```bash
source ~/turtlebot3_ws/tb3_env.sh
cd ~/turtlebot3_ws
colcon build --symlink-install --packages-select tb3_course
source install/setup.bash
```

- `--symlink-install`：Python 文件用符号链接，改源码不用重新 build；
- 但 `setup.py` / `package.xml` 改了，仍然要重新 build；
- 只 build 一个包：`--packages-select`。

---

## 6.2 Git 与可复现

### 6.2.1 提交什么

提交：

- `src/`、`launch/`、`scripts/`；
- `homework/report/`、`homework/maps/`、`homework/videos/`、
  `homework/data/`、`homework/tools/`；
- `README.md`、`.gitignore`。

不提交：

- `build/`、`install/`、`log/`；
- `__pycache__`、`*.pyc`；
- 临时截图/录制帧。

`.gitignore`：

```gitignore
build/
install/
log/
__pycache__/
*.py[cod]
.DS_Store
*.swp
*~
*.xwd
```

### 6.2.2 一次实验一个记录

每次实验至少记录：

```text
commit/代码版本：
世界/地图：
命令：
参数：
关键输出（topic/数值/截图）：
指标：
异常/失败：
结论：
```

这样别人（包括未来的你）才能复现。

### 6.2.3 提交信息

好的提交信息：

```text
Add closed-loop square controller and measured 5 mm closure

- square_odom.py: odom feedback + heading P control
- results: open-loop 0.344 m vs closed-loop 0.005 m
- add square_odom.csv and trajectory plot
```

不要写“update”“fix bug”这种没有信息量的。

---

## 6.3 调试方法论

### 6.3.1 从外到内

```text
1. 进程/节点在不在？     ros2 node list
2. 话题在不在？           ros2 topic list
3. 数据有没有？           ros2 topic echo/hz
4. 类型/QoS 对不对？      ros2 topic info -v
5. 坐标系对不对？         tf2_echo/view_frames
6. 参数对不对？           ros2 param get/dump
7. 算法逻辑对不对？       加日志/画图/复现
```

### 6.3.2 二分法

- 一半命令能跑、一半不能 → 从中间切开；
- 用空世界排除环境干扰；
- 用 `ros2 topic pub` 直接注入数据，绕过上游；
- 用 `ros2 bag record` 录下来，离线回放。

### 6.3.3 最小复现

把问题压缩到：

```text
一个节点 + 一个话题 + 一个参数
```

能复现，就离解决不远了。

---

## 6.4 本项目的真实坑（也是你的排障清单）

| 现象 | 根因 | 处理 |
|---|---|---|
| Gazebo 卡住，`/spawn_entity` 不出现 | `GAZEBO_MODEL_PATH` 缺系统模型，联网下载超时 | `tb3_env.sh` 补 `/usr/share/gazebo-11/models` |
| `/scan` 全是 0.164 m | TB4/Ignition GPU 雷达 PRIME 问题 | TB3 用 CPU ray；TB4 需 PRIME |
| 反复启停后话题有但收不到 | DDS 端口/共享内存残留 | `stop_tb3.sh`，清 `/dev/shm/fastrtps_*` |
| 车一直走/侧翻 | `/cmd_vel` 最后一条保持 | 补零；`velocity_mirror` 看门狗 |
| Nav2 卡在 `Extrapolation Error` | goal 时间戳旧 | `stamp=0` |
| 自动建图原地摆动 | 路线贴墙 + 雷达扇区索引错 | 路线内收；按 0..2π 归一化角度 |
| RViz 窗口高度奇数，H.264 编码失败 | yuv420p 要求偶数宽高 | `pad=ceil(iw/2)*2:ceil(ih/2)*2` |
| 官方 multi_robot 写 `/opt` 失败 | apt 安装目录只读 | 临时 SDF 写到 `/tmp` |
| Nav2 目标被拒绝 | 目标在障碍/膨胀层 | 换自由空间；调 inflation |
| 多车 TF 冲突 | frame 没加前缀 | `frame_prefix` + SDF frame 改名 |

---

## 6.5 Capstone 项目（四选一）

> 目标：不依赖教程，独立完成一个可演示、可复现、可解释的小项目。
> 建议 2–3 周，最后交代码 + 报告 + 3 分钟视频。

### 方向 A：新世界的自主建图 + 导航

要求：

1. 选一个没跑过的世界（`turtlebot3_house` 或自定义 world）；
2. 自己设计覆盖路线，建图并保存；
3. 用 Nav2 导航到至少 5 个点；
4. 评估：地图覆盖率、路径成功率、用时；
5. 对比：不同 `inflation_radius`/planner 的影响。

考察：SLAM、Nav2、参数、排障。

### 方向 B：多机器人编队/跟随穿迷宫

要求：

1. 两车在 `turtlebot3_world` 里；
2. leader 走一条穿圆柱的路线；
3. follower 保持 0.5–0.8 m 距离；
4. 加一个安全策略（避障/停车/重跟）；
5. 评估：最小距离、距离误差、任务完成时间、是否碰撞。

进阶：

- 加 leader 速度前馈；
- 用 Pure Pursuit 跟踪轨迹；
- 三车编队。

考察：多机器人、TF、控制、路径。

### 方向 C：地图融合最小实验

要求：

1. 两车在不同起点/区域分别建图；
2. 用已知初始位姿把两张地图变换到同一坐标系；
3. 融合占据栅格；
4. 评估：覆盖率、冲突格比例、墙线一致性；
5. 讨论：如果初始位姿未知，怎么办？

考察：SLAM、坐标变换、栅格地图、研究能力。

### 方向 D：Nav2 插件对比与调参

要求：

1. 同一张地图/同一组目标；
2. 对比 NavFn vs SmacPlanner2D；
3. 对比 DWB vs RPP；
4. 记录路径长度、用时、重规划次数、恢复行为次数；
5. 给出“什么场景用哪个”的结论。

考察：Nav2、实验设计、数据分析。

---

## 6.6 Capstone 评分表（建议）

| 维度 | 权重 | 优秀标准 |
|---|---|---|
| 问题定义 | 10% | 目标清楚、边界明确、指标可量化 |
| 系统设计 | 20% | 框图正确、模块职责清楚、坐标/话题/QoS 合理 |
| 实现质量 | 20% | 代码可读、参数化、异常处理、可复现 |
| 实验设计 | 20% | 有基线、对照、消融、重复实验 |
| 数据分析 | 15% | 指标完整、图表清楚、解释合理 |
| 演示/报告 | 15% | 视频能看出效果，报告能讲清原理和限制 |

**一票否决**：

- 代码不能复现；
- 数据造假；
- 讲不清自己的代码/算法。

---

## 6.7 如何独立学习一个新模块

以“给 Nav2 换一个 planner”为例：

1. **看文档**：Nav2 官方文档的 planner 插件页；
2. **看接口**：`ros2 interface show nav2_msgs/action/NavigateToPose`；
3. **找最小例子**：官方 launch/params；
4. **跑基线**：先跑默认 NavFn，记录指标；
5. **替换**：改 YAML 的 `planner_plugins`；
6. **验证**：同样目标/地图，记录指标；
7. **消融**：只改一个参数，观察变化；
8. **解释**：为什么变好/变差；
9. **记录**：命令、参数、指标、截图；
10. **复盘**：下次遇到新插件，按同样流程。

---

## 6.8 8 周学习计划（可压缩）

| 周 | 任务 | 检查点 |
|---|---|---|
| 1 | 第 1 章 + 练习 | 能独立写 pub/sub，画 TF 树 |
| 2 | 第 2 章 + 闭环正方形 | 解释开环/闭环误差 |
| 3 | 第 3 章 + 建图 | 地图闭合，能评估 |
| 4 | 第 4 章 + Nav2 3 目标 | 能排障 goal rejected |
| 5 | 第 5 章 + 镜像/距离 | 双车话题/TF 正常 |
| 6 | 第 5 章 + P/轨迹跟随 | 穿圆柱不碰撞 |
| 7 | 第 6 章 + Capstone 开题 | 指标/基线/实验设计 |
| 8 | Capstone 实现 + 报告/视频 | 可复现、能讲清 |

---

## 6.9 命令速查

```bash
# 环境
source ~/turtlebot3_ws/tb3_env.sh
colcon build --symlink-install --packages-select tb3_course
source install/setup.bash

# 仿真
bash start_world.sh
bash start_world.sh empty_world
bash start_cartographer.sh
bash start_nav2.sh
bash start_teleop.sh
bash start_multi_robot.sh
bash stop_tb3.sh

# 观察
ros2 node list
ros2 topic list -t
ros2 topic info /scan -v
ros2 topic echo /odom --once
ros2 topic hz /scan
ros2 topic bw /scan
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_tools view_frames
ros2 param list
ros2 lifecycle get /amcl
ros2 action list
rqt_graph

# 节点
ros2 run tb3_course circle --ros-args -p radius:=0.5
ros2 run tb3_course pose_listener
ros2 run tb3_course move_square --ros-args -p use_sim_time:=true
ros2 run tb3_course square_odom --ros-args -p use_sim_time:=true
ros2 run tb3_course auto_mapper --ros-args -p use_sim_time:=true
ros2 run tb3_course nav_goal --ros-args -p use_sim_time:=true -p goal_x:=1.45 -p goal_y:=0.0
ros2 run tb3_course patrol --ros-args -p use_sim_time:=true -p loops:=2
ros2 run tb3_course velocity_mirror
ros2 run tb3_course robot_distance --ros-args -p use_sim_time:=true
ros2 run tb3_course leader_follower --ros-args -p use_sim_time:=true
ros2 run tb3_course trail_follower --ros-args -p use_sim_time:=true

# 地图
ros2 run nav2_map_server map_saver_cli -f ~/maps/tb3_world
python3 homework/tools/plot_map.py ~/maps/tb3_world.pgm --output out.png

# 数据
python3 homework/tools/record_trajectory.py --type odom --topic /odom \
  --duration 30 --output data/odom.csv
python3 homework/tools/plot_trajectory.py --input data/odom.csv --output plot.png
python3 homework/tools/record_path.py --output data/paths.csv
python3 homework/tools/plot_paths.py --input data/paths.csv --output paths.png

# 录屏
python3 homework/tools/record_window.py --title RViz \
  --duration 20 --fps 4 --output homework/videos/demo.mp4
```

---

## 6.10 推荐阅读顺序

1. ROS2 官方教程：Nodes/Topics/Services/Actions/Parameters/Launch/TF2；
2. REP 105（坐标框架）；
3. Gazebo Classic 教程：Model/Plugin/Sensor；
4. Cartographer 论文 + 配置；
5. Nav2 文档：Concepts → Configuration → Tutorials；
6. *Probabilistic Robotics*：Kalman/粒子滤波/SLAM（选读）；
7. 多机器人 SLAM 综述（选读）。

**读论文的方法**：

- 先读 abstract/introduction/figures；
- 画系统框图；
- 找输入/输出/优化目标；
- 想“如果我要复现，最小实验是什么”；
- 不要从头到尾逐字读。

---

## 6.11 最后：什么叫“独立实现”

能做到下面这些，才算真正学会：

- 给一个新世界，你能自己决定用 Cartographer 还是 slam_toolbox；
- 给一个新目标，你能判断 Nav2 参数要不要调、调哪个；
- 给一个新的多机器人任务，你能设计话题/TF/控制/安全策略；
- 代码跑不通，你能按“节点→话题→数据→TF→参数→算法”定位；
- 你能用指标证明“变好了”，也能解释“为什么”；
- 你能把实验写成别人可以复现的报告。

> 教程到这里结束，但学习才刚开始。下一步选一个 Capstone 方向，
> 从“最小可运行”开始，迭代、记录、解释。遇到问题，回到本教程的
> 章节和自测题，通常能找到排查思路。
