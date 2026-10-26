# 第 5 章：多机器人通信与协作

> 本章目标：理解 ROS2 多机器人的 namespace、TF frame、DDS domain；
> 能自己启动两台 TB3，写镜像、距离、Leader-Follower；理解为什么
> “追当前点”会切内角，以及如何用“轨迹跟随”穿过圆柱阵列；
> 最后建立多机器人地图融合/协同避障的研究框架。

---

## 5.1 多机器人到底难在哪

单机器人时，话题名是 `/cmd_vel`、`/odom`、`/scan`，TF 是
`odom→base_footprint`。两台机器人如果都这样：

```text
两台车都发 /cmd_vel   → 谁收到谁的？
两台车都发 /odom     → 话题冲突
两台车都叫 odom frame → TF 树冲突
```

ROS2 的解决办法：

1. **namespace**：`/TB3_1/cmd_vel`、`/TB3_2/cmd_vel`；
2. **TF frame 前缀**：`TB3_1/odom`、`TB3_2/odom`；
3. **DDS domain**：不同组/不同网络用 `ROS_DOMAIN_ID` 隔离。

---

## 5.2 官方 launch 的结构

官方：

```bash
/opt/ros/humble/share/turtlebot3_gazebo/launch/multi_robot.launch.py
```

它默认 **4 台**：

```python
number_of_robots = 4
namespace = 'TB3'
pose = [[-2, -0.5], [0.5, -2], [2, 0.5], [-0.5, 2]]
```

每台车：

1. 复制一份 `model.sdf`；
2. 把 `odometry_frame` 改成 `TB3_1/odom`；
3. 把 `robot_base_frame` 改成 `TB3_1/base_footprint`；
4. 把 `frame_name` 改成 `TB3_1/base_scan`；
5. 写到临时 SDF；
6. 用 `spawn_entity.py` 生成，带 `-robot_namespace TB3_1`；
7. `robot_state_publisher` 用 `frame_prefix=TB3_1`；
8. 用 `GroupAction + PushRosNamespace('TB3_1')` 组织。

### 我们的 `multi_robot_2.launch.py` 改了什么

文件：`src/tb3_course/launch/multi_robot_2.launch.py`

1. `number_of_robots = 2`；
2. `pose = [[-2.0,-0.5], [0.5,-2.0]]`；
3. 临时 SDF 写到 `/tmp/tb3_multi/`，而不是 `/opt/ros/...`（普通用户没权限）；
4. 支持 `world:=...` 参数，默认 `turtlebot3_world.world`；
5. 其他 namespace/frame 逻辑与官方一致。

> 为什么要改临时路径？
> 官方 launch 把 `tmp1.sdf` 写到包安装目录。apt 安装的 `/opt/ros` 是
> root 所有，普通用户写入会 `PermissionError`。这是一个很典型的
> “官方示例在源码空间能跑、apt 安装后不能跑”的问题。

### 启动

```bash
source ~/turtlebot3_ws/tb3_env.sh
ros2 launch tb3_course multi_robot_2.launch.py
# 或空世界
ros2 launch tb3_course multi_robot_2.launch.py world:=empty_world.world
```

### 检查

```bash
ros2 topic list | grep TB3_
ros2 topic echo /TB3_1/odom --once
ros2 topic echo /TB3_2/odom --once
ros2 run tf2_ros tf2_echo TB3_1/odom TB3_1/base_footprint
```

期望：

```text
/TB3_1/cmd_vel  /TB3_1/odom  /TB3_1/scan  /TB3_1/imu ...
/TB3_2/cmd_vel  /TB3_2/odom  /TB3_2/scan  /TB3_2/imu ...
```

本机初始 odom：

```text
TB3_1 = (-2.00, -0.50)
TB3_2 = ( 0.50, -2.00)
```

说明 Gazebo 的 `libgazebo_ros_diff_drive` 默认 `odometry_source=world`，
两车的 odom 都是世界坐标。如果换版本后发现 odom 从 0 开始，那是编码器
里程计，需要给距离/跟随节点加 spawn 偏移。

---

## 5.3 速度镜像：`velocity_mirror.py`

### 原理

```text
/TB3_1/cmd_vel ──订阅──→ 乘以 scale ──发布──→ /TB3_2/cmd_vel
```

这是**开环镜像**，不是协作：

- leader 动，follower 跟着动；
- 两车动力学差异会漂移；
- 没有反馈，不能保持队形。

### 代码

```python
class VelocityMirror(Node):
    def __init__(self):
        super().__init__('velocity_mirror')
        self.declare_parameter('scale', 0.9)
        self.declare_parameter('cmd_timeout', 0.5)
        self.sub = self.create_subscription(
            Twist, '/TB3_1/cmd_vel', self.cmd_cb, 10)
        self.pub = self.create_publisher(Twist, '/TB3_2/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.watchdog)
        self.last_cmd_time = None

    def cmd_cb(self, msg):
        scale = float(self.get_parameter('scale').value)
        out = Twist()
        out.linear.x = msg.linear.x * scale
        out.angular.z = msg.angular.z * scale
        self.pub.publish(out)
        self.last_cmd_time = time.monotonic()

    def watchdog(self):
        if self.last_cmd_time is None:
            return
        if time.monotonic() - self.last_cmd_time > 0.5:
            self.pub.publish(Twist())
            self.last_cmd_time = None
```

### 为什么需要看门狗

Gazebo 的 `/cmd_vel` 会保持最后一条命令。如果 leader 发完非零速度后
不补零，follower 会一直走。本课程早期两台车就是这样撞墙侧翻的。

看门狗：leader 0.5 s 没发命令 → 给 follower 发零速度。

> **教训**：任何“转发/镜像”节点都要考虑输入中断时的安全行为。

### 实测

```text
leader  (-2.00,-0.50) → (-1.74,-0.50)
follower ( 0.50,-2.00) → ( 0.65,-2.00)
scale=0.9；随后 follower 在 0.5 s 后停车
两车 z≈0.0085，姿态水平
```

---

## 5.4 两车距离：`robot_distance.py`

### 原理

订阅两车 odom，计算欧氏距离：

```python
distance = math.hypot(x2 - x1, y2 - y1)
```

### 代码

```python
self.sub1 = self.create_subscription(
    Odometry, '/TB3_1/odom', self.odom1_cb, 10)
self.sub2 = self.create_subscription(
    Odometry, '/TB3_2/odom', self.odom2_cb, 10)
self.pub = self.create_publisher(Float32, '/tb3_distance', 10)
self.timer = self.create_timer(1.0, self.report)
```

### 坐标系问题

- Gazebo 默认 odom 是世界坐标 → 直接相减；
- 如果 odom 是编码器坐标 → 加 spawn 偏移：

```python
x1 = pose1.x + offset_x1
y1 = pose1.y + offset_y1
```

本仓库提供 `tb3_1_offset_x/y`、`tb3_2_offset_x/y` 参数。

### 实测

```text
初始 distance = 2.915 m
镜像运动后 = 2.820 m
静止时稳定
```

---

## 5.5 闭环 Leader-Follower：`leader_follower.py`

### 目标

让 follower 保持在 leader 后方 `d=0.6 m`：

```text
target = leader_xy - d * [cos(leader_yaw), sin(leader_yaw)]
```

### 控制律

```text
dx = target_x - follower_x
dy = target_y - follower_y
rho = hypot(dx, dy)
alpha = normalize(atan2(dy, dx) - follower_yaw)

if |alpha| > 0.5:
    ω = clamp(kp_w * alpha, ±max_w)      # 原地转
else:
    v = clamp(kp_v * rho, 0, max_v) * max(0, cos(alpha))
    ω = clamp(kp_w * alpha, ±max_w)
```

### 代码要点

```python
cmd = Twist()
if abs(alpha) > rotate_threshold:
    cmd.angular.z = clamp(kp_w * alpha, -max_w, max_w)
else:
    if rho > 0.03:
        cmd.linear.x = clamp(kp_v * rho, 0.0, max_v) * max(
            0.0, math.cos(alpha))
    cmd.angular.z = clamp(kp_w * alpha, -max_w, max_w)

if separation < min_distance:
    self.stop()
    return
```

### 实测与限制

开阔区：

```text
follower 追到 leader 后方 0.618 m
```

急转弯：

```text
leader 画半径 0.25 m 小圈，follower 切内道，距离降到 0.36 m
```

因此加了 `min_distance=0.45 m` 安全停车。更平滑的做法：

- 加 leader 速度前馈；
- 用纯跟踪/Pure Pursuit；
- 直接用 Nav2 跟随；
- 记录轨迹跟随（下一节）。

---

## 5.6 轨迹跟随：`leader_follower_path.py`（穿迷宫）

### 为什么 P 跟随会切内角

P 跟随的目标是“leader 当前位姿后方 0.6 m 的点”。leader 转弯时，
这个点会跳到新方向；follower 直接追这个点，就会走捷径、切进圆柱。

### 思路：记录 leader 的轨迹

```text
leader 走过 → 记下每 0.05 m 一个 breadcrumb
follower 目标 = 沿轨迹落后 0.6 m 的点
```

因为 leader 的轨迹是**已经避障过的安全路径**，follower 沿轨迹走就不会
撞圆柱。

### 核心代码

记录：

```python
if not self.trail:
    self.trail.append((p.x, p.y))
else:
    lx, ly = self.trail[-1]
    if math.hypot(p.x - lx, p.y - ly) >= resolution:
        self.trail.append((p.x, p.y))
```

取目标（从轨迹末端往回走 `distance` 的弧长）：

```python
remaining = distance
for i in range(len(self.trail) - 1, 0, -1):
    p1 = self.trail[i]
    p0 = self.trail[i - 1]
    seg = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    if seg > 1e-9 and seg >= remaining:
        ratio = remaining / seg
        return (p1[0] + (p0[0] - p1[0]) * ratio,
                p1[1] + (p0[1] - p1[1]) * ratio)
    remaining -= seg
return self.trail[0]
```

然后用和 P 跟随一样的角度/速度控制。额外加：

- `min_distance` 安全停车；
- 前方 `/scan` 安全停车；
- leader odom 超时停车。

### 运行

终端 1：双车世界

```bash
ros2 launch tb3_course multi_robot_2.launch.py
```

终端 2：把 leader 开到 follower 附近（示例，实际可用 `auto_mapper` remap）

```bash
ros2 run tb3_course auto_mapper --ros-args -p use_sim_time:=true \
  -p waypoints:="-1.65,-0.5;-1.65,-1.5;0.5,-1.5" \
  -p spin_at_end:=false \
  -r /cmd_vel:=/TB3_1/cmd_vel \
  -r /odom:=/TB3_1/odom \
  -r /scan:=/TB3_1/scan
```

终端 3：轨迹跟随

```bash
ros2 run tb3_course trail_follower --ros-args \
  -p use_sim_time:=true -p follow_distance:=0.6
```

终端 4：让 leader 穿圆柱阵列（可用 `auto_mapper` 的 waypoints）

```bash
ros2 run tb3_course auto_mapper --ros-args -p use_sim_time:=true \
  -p waypoints:="1.65,-1.65;1.65,0.55;0.55,0.55;0.55,-0.55;-0.55,-0.55;-0.55,0.55;-1.65,0.55;-1.65,-1.5" \
  -p linear_speed:=0.15 -p spin_at_end:=false \
  -r /cmd_vel:=/TB3_1/cmd_vel \
  -r /odom:=/TB3_1/odom \
  -r /scan:=/TB3_1/scan
```

### 实测

```text
leader 路线：(0.37,-1.49) → (1.65,-1.65) → (1.65,0.55) → (0.55,0.55) → ...
follower 距离：0.55–0.79 m
成功绕过 9 个圆柱
```

视频：`homework/videos/06_leader_follower_maze.mp4`。

### 局限与改进

- leader 的轨迹是“过去”的，leader 突然倒退/急转时 follower 仍可能跟不上；
- 没有显式避障，只有 `min_distance` 和前方 scan 停车；
- 可以做：
  - 用 leader 速度前馈；
  - 用 Pure Pursuit 跟踪轨迹；
  - 在 follower 局部加 DWA/TEB；
  - 多机器人共享 costmap。

---

## 5.7 研究型：多机器人地图融合与协同避障

> 这一节是“研究型”调研与方案设计。本仓库**没有实现**代码；
> 报告里已明确标注完成范围。下面给你一个能自学/做课题的框架。

### 5.7.1 为什么难

单机器人 SLAM 已经要解决扫描匹配、回环、位姿图；多机器人还要加：

1. **坐标对齐**：每台车的 map 原点不同，要统一到全局坐标系；
2. **数据关联**：不同车看到的同一障碍/回环如何识别；
3. **跨机器人回环**：A 车看到的场景，B 车也看到，如何建立约束；
4. **地图融合**：占据栅格怎么合并？冲突怎么处理？
5. **通信**：带宽、延迟、断连；DDS domain/namespace；
6. **动态环境**：障碍移动、地图更新、一致性。

### 5.7.2 常见路线

| 路线 | 思路 | 优点 | 缺点 |
|---|---|---|---|
| 集中式 | 一个中心节点收集各车子图/位姿，统一优化 | 全局一致性好 | 单点故障、通信压力 |
| 分布式 | 各车维护局部地图，周期交换位姿/特征 | 鲁棒、可扩展 | 一致性难 |
| 基于 pose graph | 跨车回环作为约束，联合优化 | 理论清晰 | 需要数据关联 |
| 地图合并 | 已知初始变换，直接栅格/点云配准 | 简单 | 依赖初始对齐 |

### 5.7.3 一个可做的最小实验

如果你要把“研究型”做成实现级，建议从最小闭环开始：

**实验 A：离线地图融合**

1. 同一世界，两台车分别建两张局部地图（不同起点/不同区域）；
2. 记录两车初始位姿，得到已知变换；
3. 把一张地图按变换投影到另一张地图的坐标系；
4. 对占据栅格做融合（例如取 max/加权）；
5. 评估：融合后覆盖率、墙线一致性、冲突格比例。

**实验 B：协同避障**

1. 两车共享 odom 和前方最小距离；
2. 设计规则：距离 < d_safe 或对方在前方 → 减速/停车；
3. 在 `turtlebot3_world` 里让两车相向而行；
4. 评估：最小距离、任务完成时间、停车次数。

**实验 C：跨机器人回环（研究型）**

1. 两车分别建图；
2. 用扫描描述子/特征匹配找跨车回环；
3. 把回环约束加入 pose graph，联合优化；
4. 比较优化前后的地图一致性。

### 5.7.4 怎么读论文/做调研

1. 先看综述（multi-robot SLAM survey），建立术语表；
2. 选 2–3 篇代表工作，画系统框图：输入、输出、通信、优化；
3. 复现最小实验，不要一上来就追求完整系统；
4. 定义指标：覆盖率、ATE/RPE、通信量、运行时间、冲突格比例；
5. 做消融：去掉回环/去掉通信，看指标变化；
6. 记录失败案例，比成功案例更有价值。

### 5.7.5 本课程的要求边界

- 如果老师只要“调研报告”：本仓库的 5.7 内容和报告已足够；
- 如果老师要“代码实现”：需要明确范围（A/B/C 哪一个），
  并接受它不是完整的多机器人 SLAM 系统。

---

## 5.8 练习

### 练习 A（基础）

启动双车，列出所有 `/TB3_*` 话题，画出 TF 树（`view_frames`）。

### 练习 B（镜像）

运行 `velocity_mirror`，用 `ros2 topic pub` 让 leader 前进 3 s，
然后补零；观察 follower 的运动和停车。故意不补零，观察差异。

### 练习 C（距离）

运行 `robot_distance`，记录两车从远到近的距离曲线；用 `rqt_plot` 画。

### 练习 D（P 跟随）

运行 `leader_follower`，让 leader 直行/转弯，记录距离；调
`follow_distance`、`kp_linear`、`kp_angular`，观察超调/振荡。

### 练习 E（轨迹跟随）

运行 `trail_follower`，让 leader 穿圆柱阵列；记录 follower 距离；
尝试把 `follow_distance` 改成 0.4/0.8，观察效果。

### 练习 F（独立）

给 `trail_follower` 加一个“leader 速度前馈”：

```python
cmd.linear.x += leader_speed * cos(alpha)
```

先预测它会不会减小跟踪误差，再实验验证。

### 练习 G（研究型）

选 5.7.3 的实验 A 或 B，做一个最小实现，写出指标和结果。

---

## 5.9 自测题

1. 多机器人为什么需要 namespace？只改话题名够吗？
2. 为什么 TF frame 也要加前缀？不加会怎样？
3. `ROS_DOMAIN_ID` 和 namespace 分别解决什么问题？
4. 官方 multi_robot launch 为什么写临时 SDF 会失败？怎么改？
5. Gazebo 的 odom 是世界坐标还是编码器坐标？怎么验证？
6. `velocity_mirror` 为什么需要看门狗？
7. P 跟随为什么在急转弯会切内角？怎么改？
8. 轨迹跟随的目标点怎么取？弧长怎么算？
9. 轨迹跟随还有什么局限？至少说 3 个改进方向。
10. 多机器人地图融合的 4 个核心难点是什么？
11. 如果老师要求实现级研究型作业，你会先做哪个最小实验？为什么？

> 下一章：工程化、调试、测试和 Capstone。你会把“会写代码”升级成
> “能独立做项目、能复现、能讲清楚”。
