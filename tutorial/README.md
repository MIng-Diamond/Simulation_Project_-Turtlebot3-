# 从零到一：TurtleBot3 + ROS 2 Humble 机器人仿真完整教程

> 本教程基于老师发的《TurtleBot3 ROS 2 Humble 仿真机器人实验教程》，
> 但不会只让你“照着命令敲一遍”。它结合本仓库的真实实现，讲清楚：
>
> **原理 → 最小实验 → 代码/算法 → 改错与调参 → 独立实现**
>
> 所有命令都在 Ubuntu 22.04 + ROS 2 Humble + Gazebo Classic 11.10.2 +
> TurtleBot3 Burger 上实际跑过。老师教程里踩过的坑，这里会解释为什么，
> 并给出本机的修正。

---

## 0. 这套教程适合谁

- 第一次接触 ROS/移动机器人仿真的本科生；
- 已经能照着教程跑通，但一改参数/一换场景就不知道从哪下手的人；
- 想真正理解 SLAM、Nav2、多机器人，而不是只会复制 launch 文件的人。

**先修要求**：

- 会用 Linux 终端：`cd`、`ls`、`source`、管道、`grep`；
- 会一点 Python：类、函数、`import`、`if/for`；
- 不要求学过 ROS、Gazebo、SLAM、控制理论。

---

## 1. 学完你能做到什么

| 能力 | 对应章节 |
|---|---|
| 解释 ROS2 的节点/话题/服务/动作/参数/QoS，能自己写 publisher/subscriber | 第 1 章 |
| 看懂 URDF/SDF、Gazebo 插件、差速底盘和激光雷达模型 | 第 2 章 |
| 用 `/odom` 写闭环控制器，解释开环/闭环误差 | 第 2 章 |
| 解释 Cartographer 的子图/扫描匹配/回环，能建图、保存、评估地图 | 第 3 章 |
| 解释 Nav2 的代价地图、AMCL、规划器/控制器/行为树，能发目标、调参、排障 | 第 4 章 |
| 搭双机器人 launch，解释 namespace/TF/DDS，能写镜像、距离、Leader-Follower | 第 5 章 |
| 独立设计一个小项目：问题、指标、基线、实验、分析、演示 | 第 6 章 |

---

## 2. 怎么用这套教程（很重要）

### 2.1 三步法：不要复制粘贴

每个知识点都按三步走：

1. **先读原理**：这一节要解决什么问题？输入输出是什么？为什么需要它？
2. **再跑最小实验**：用仓库里最小的命令/节点验证一个现象，记录输出。
3. **最后改代码**：先预测“改这个参数会发生什么”，再运行验证，再解释差异。

> 复制粘贴只能让你“见过”；预测—验证—解释才会让你“会”。
> 每章最后的练习就是逼你做第三步。

### 2.2 每章的结构

```text
你将学会
先修
原理（为什么）
最小实验（手把手命令 + 期望输出）
代码解读（本仓库实际文件）
常见坑与排障
练习（从模仿到独立）
自测题（能讲清楚才算会）
```

### 2.3 准备一个实验记录

建议每做一次实验记录：

```text
日期/章节：
目标：
命令：
关键输出（话题/数值/截图路径）：
我的预测：
实际结果：
差异与解释：
下次要改什么：
```

这份记录最后可以直接变成你的实验报告和答辩材料。

### 2.4 遇到问题时先做三件事

1. `ros2 node list` / `ros2 topic list` 看系统里到底有什么；
2. `ros2 topic echo/info/hz` 看数据有没有、类型对不对、频率对不对；
3. `ros2 run tf2_ros tf2_echo <parent> <child>` 看坐标变换链断在哪。

---

## 3. 本仓库地图

```text
turtlebot3_ws/
├── src/tb3_course/                 # 课程功能包（本教程的核心代码）
│   ├── tb3_course/
│   │   ├── turtle_circle.py        # 第 1 章：publisher + 参数
│   │   ├── pose_listener.py        # 第 1 章：subscriber
│   │   ├── move_square.py          # 第 2 章：开环控制
│   │   ├── square_odom.py          # 第 2 章：闭环控制
│   │   ├── auto_mapper.py          # 第 3 章：程序化建图
│   │   ├── nav_goal.py             # 第 4 章：Nav2 Action
│   │   ├── patrol.py               # 第 4 章：多点巡航
│   │   ├── velocity_mirror.py      # 第 5 章：开环镜像
│   │   ├── robot_distance.py       # 第 5 章：两车距离
│   │   ├── leader_follower.py      # 第 5 章：P 跟随
│   │   └── leader_follower_path.py # 第 5 章：轨迹跟随（穿迷宫）
│   └── launch/multi_robot_2.launch.py
├── homework/                       # 作业产物（报告/地图/视频/数据）
├── tb3_env.sh                      # 每个终端先 source
├── start_world.sh / start_nav2.sh / start_multi_robot.sh
├── stop_tb3.sh
└── tutorial/                       # 本教程
```

### 先跑通一次环境

```bash
source ~/turtlebot3_ws/tb3_env.sh
cd ~/turtlebot3_ws
colcon build --symlink-install --packages-select tb3_course
source install/setup.bash
ros2 pkg executables tb3_course
```

期望看到 `tb3_course` 下 11 个可执行文件。

启动世界：

```bash
bash start_world.sh
```

另开终端：

```bash
source ~/turtlebot3_ws/tb3_env.sh
ros2 topic list
ros2 topic echo /scan --once
ros2 topic hz /odom
```

只要你看到 `/odom` 和 `/scan` 有数据，就说明环境正常。

---

## 4. 学习路线（建议 8 周）

| 周 | 内容 | 产出 |
|---|---|---|
| 1 | 第 1 章：ROS2 基础、节点/话题、TF、launch | 能独立写 publisher/subscriber，画出 TF 树 |
| 2 | 第 2 章：Gazebo/URDF/SDF、差速底盘、开环/闭环 | 自己改正方形参数，解释误差来源 |
| 3 | 第 3 章：SLAM 原理、Cartographer、建图 | 自己建一张图并评估 |
| 4 | 第 4 章：Nav2 架构、AMCL、代价地图 | 3 个目标 + 两圈巡逻，调一次参数 |
| 5 | 第 5 章：多机器人 namespace/TF、镜像、距离 | 双车镜像 + 距离输出 |
| 6 | 第 5 章：Leader-Follower、轨迹跟随 | 两车穿过圆柱阵列 |
| 7 | 第 6 章：工程化、调试、测试 | 代码/报告可复现 |
| 8 | Capstone | 独立项目 + 演示视频 + 报告 |

时间紧可以两周压缩，但**每章的练习必须自己写一遍**。

---

## 5. 章节链接

1. [第 1 章：ROS2 基础与第一个节点](01_ros2_basics.md)
2. [第 2 章：Gazebo、机器人模型与运动控制](02_gazebo_robot.md)
3. [第 3 章：Cartographer SLAM 与建图](03_slam.md)
4. [第 4 章：Navigation2 自主导航](04_nav2.md)
5. [第 5 章：多机器人通信与协作](05_multi_robot.md)
6. [第 6 章：工程化、调试与 Capstone 项目](06_engineering_capstone.md)

### 和老师教程的对应关系

| 老师教程 | 本教程 |
|---|---|
| 第一节 基础准备、ROS2 安装 | 第 1 章 + 第 2 章 2.1/2.2 |
| 第二节 ROS2 与 TurtleBot3 基础 | 第 1 章 + 第 2 章 2.3–2.6 |
| 第三节 Gazebo 环境与测试 | 第 2 章 |
| 第四节 Cartographer SLAM | 第 3 章 |
| 第五节 Navigation2 | 第 4 章 |
| 第六节 多机器人 | 第 5 章 |
| 课程作业/拓展 | 每章练习 + 第 6 章 Capstone |

老师教程给的是“做什么”；本教程补上“为什么”“代码怎么写的”“出错怎么查”
“怎么自己设计下一个实验”。

---

## 6. 自测与项目

每章末尾有自测题。建议用“**合上书讲给别人听**”的标准要求自己：

- 能画出系统框图/TF 树；
- 能说出每个输入输出、频率、坐标系；
- 能解释为什么不用另一种方案；
- 能在一个新场景里改代码并验证。

第 6 章给出 4 个 Capstone 方向和一个评分表。真正独立实现的标志是：

> 给你一个新的世界/新的目标，你不需要教程，能自己拆问题、选工具、
> 写代码、做实验、解释结果。

---

## 7. 参考资料

- ROS 2 Humble 官方文档：<https://docs.ros.org/en/humble/>
- Nav2 官方文档：<https://docs.nav2.org/>
- Gazebo Classic 教程：<https://classic.gazebosim.org/tutorials>
- Cartographer 论文：Hess et al., *Real-Time Loop Closure in 2D LIDAR SLAM*, ICRA 2016
- REP 105（坐标框架）：<https://www.ros.org/reps/rep-0105.html>
- *Probabilistic Robotics*（Thrun, Burgard, Fox）——SLAM/定位的理论基础
- TurtleBot3 官方 e-Manual：<https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/>

---

## 8. 说明

- 本教程以“能独立实现”为目标，不追求把每个 API 都列全；
  遇到新 API，按“看消息类型 → 看 QoS → 看坐标系 → 写最小节点”的顺序查文档。
- 老师教程的章节编号和本教程不完全对应，但知识点一一覆盖；
  本教程补充了原理、代码实现、调试方法和多机器人进阶。
- 所有实验都可以在本仓库的代码上复现；报告和视频在
  `homework/report/` 与 `homework/videos/`。
