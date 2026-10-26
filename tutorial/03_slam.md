# 第 3 章：Cartographer SLAM 与建图

> 本章目标：理解 SLAM 在解决什么问题、Cartographer 的子图/扫描匹配/回环
> 是怎么工作的；能独立启动建图、保存地图、评估地图；看懂 `auto_mapper`
> 的每个状态和参数；知道什么时候该换 slam_toolbox 或考虑 3D SLAM。

---

## 3.1 SLAM 到底在解决什么

机器人一边走一边建图，同时还要知道自己在哪里：

```text
输入：/scan（激光）+ /odom（里程计）+ /tf（初始关系）
输出：/map（占据栅格地图）+ map→odom（修正后的定位）
```

如果只用里程计：

- 轮子打滑、地面不平、模型误差 → 位姿逐渐漂移；
- 走一圈回到原点，里程计可能认为“差了 20 cm”。

SLAM 的核心思想：

1. **扫描匹配**：把当前激光和已有地图/子图对齐，修正位姿；
2. **回环检测**：发现“这个地方我来过”，建立约束；
3. **位姿图优化**：把所有约束放在一起优化，消除累积误差；
4. **占据栅格**：把激光命中点标记为占据，穿过的地方标记为空闲。

### 占据栅格地图

每个格子有三种状态：

```text
0   空闲（free）
100 占据（occupied）
-1  未知（unknown）
```

PGM 文件里通常：

- 0 = 黑 = 占据；
- 254 = 白 = 空闲；
- 205 = 灰 = 未知。

地图 YAML：

```yaml
image: tb3_world.pgm
resolution: 0.05          # 每像素 5 cm
origin: [-1.21, -2.37, 0] # 左下角在地图坐标系的位置
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

### 子图（submap）

Cartographer 不会把整张地图一次性优化，而是：

- 每走一小段，把扫描拼成一个 **submap**；
- 当前扫描先和最近的 submap 匹配（局部 SLAM）；
- 所有 submap 之间再做回环检测和全局优化（全局 SLAM）。

这样计算量可控，又能处理大场景。

---

## 3.2 Cartographer 的配置

配置文件：

```bash
/opt/ros/humble/share/turtlebot3_cartographer/config/turtlebot3_lds_2d.lua
```

关键字段：

```lua
map_frame = "map"
tracking_frame = "imu_link"
published_frame = "odom"
odom_frame = "odom"
provide_odom_frame = false
use_odometry = true
num_laser_scans = 1
TRAJECTORY_BUILDER_2D.min_range = 0.12
TRAJECTORY_BUILDER_2D.max_range = 3.5
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)
```

逐个理解：

- `map_frame`：全局地图坐标系；
- `tracking_frame`：用哪个 frame 做跟踪（TB3 用 `imu_link`）；
- `published_frame`：Cartographer 发布 `map→published_frame`，这里是
  `map→odom`；
- `use_odometry=true`：用轮式里程计做初值，减少扫描匹配搜索范围；
- `min_range/max_range`：过滤太近/太远的激光；
- `use_imu_data=false`：2D 低速度场景不需要 IMU；
- `use_online_correlative_scan_matching=true`：在线相关性扫描匹配，
  比纯梯度优化更稳，但更耗 CPU；
- `motion_filter`：运动小于阈值就不插入新扫描，节省计算。

> 为什么 `published_frame=odom`？
> 因为 Nav2/AMCL 后续会接管 `map→odom`，而 Cartographer 建图时也需要
> 发布这个变换。`provide_odom_frame=false` 表示不额外造一个 odom frame，
> 直接用机器人已有的 `odom`。

---

## 3.3 动手建图

### 启动

终端 1：

```bash
source ~/turtlebot3_ws/tb3_env.sh
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

终端 2：

```bash
source ~/turtlebot3_ws/tb3_env.sh
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

终端 3（本仓库的自动巡航）：

```bash
source ~/turtlebot3_ws/tb3_env.sh
ros2 run tb3_course auto_mapper --ros-args -p use_sim_time:=true
```

### 检查

```bash
ros2 topic echo /map --once --field info
ros2 topic hz /scan
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo map base_footprint
```

### 保存

```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/tb3_world
```

得到：

```text
~/maps/tb3_world.pgm
~/maps/tb3_world.yaml
```

### 本机实测

```text
124 × 115 栅格 @ 0.05 m
origin = [-1.21, -2.37, 0]
free 55.3%，occupied 5.6%，unknown 39.1%
```

未知区域主要是六边形墙外；墙内边界和 9 个圆柱闭合。

---

## 3.4 `auto_mapper`：程序化建图的算法

文件：`src/tb3_course/tb3_course/auto_mapper.py`

### 为什么需要它

手动遥控建图：

- 需要人盯着 RViz 走；
- 容易漏掉区域；
- 重复实验时不可复现。

`auto_mapper` 用 waypoint + odom 反馈 + 激光避障，自动跑一条覆盖路线。

### 输入输出

```text
订阅：/odom、/scan
发布：/cmd_vel
参数：waypoints、linear_speed、angular_speed、waypoint_tolerance、
      obstacle_distance、front_half_angle_deg、emergency_half_angle_deg、
      emergency_distance、spin_at_end、max_duration
```

### 状态机

```text
WAIT_ODOM
  └── 收到 odom → GO

GO
  ├── 到达当前 waypoint：index++，切换下一个
  ├── 前方障碍：AVOID
  └── 否则：朝向目标，直行 + 角度修正

AVOID
  ├── 前方/紧急扇区恢复安全：GO
  └── 超时：GO（带警告）

SPIN_END
  └── 转 360° 后结束（帮助回环）
```

### 代码骨架

```python
if self.state == 'GO':
    tx, ty = self.waypoints[self.index]
    dx, dy = tx - x, ty - y
    dist = math.hypot(dx, dy)
    if dist < tol:
        self.stop()
        self.index += 1
        return

    heading = math.atan2(dy, dx)
    error = normalize_angle(heading - yaw)
    front = self.front_min()
    emergency_front = self.emergency_min()

    if front < obstacle or emergency_front < emergency:
        self.stop()
        self.avoid_dir = 1.0 if self.left_min() > self.right_min() else -1.0
        self.switch('AVOID')
        return

    cmd = Twist()
    if abs(error) > 0.5:
        cmd.angular.z = clamp(1.5 * error, -angular, angular)
    else:
        cmd.linear.x = speed
        cmd.angular.z = clamp(1.5 * error, -angular, angular)
    self.pub.publish(cmd)
```

### 雷达扇区：为什么不能直接取下标

本机 `LaserScan.angle_min=0.0`，`angle_max≈2π`。
如果按 `(center - angle_min)/inc` 取下标：

- `center=0`：负半轴被忽略，只检查一侧；
- `center=-π/2`：索引全负，返回 `inf`。

正确写法：

```python
for i, r in enumerate(self.scan.ranges):
    if not math.isfinite(r) or r <= self.scan.range_min:
        continue
    angle = self.scan.angle_min + i * self.scan.angle_increment
    if abs(normalize_angle(angle - center_rad)) <= half_rad:
        best = min(best, r)
```

### 自车回波与扇区选择

TB3 雷达会扫到自己的结构，侧面 50° 外有 0.12–0.19 m 的回波。
所以：

- 前方 `front_half_angle_deg=20°`，`obstacle_distance=0.30 m`；
- 近距紧急 `emergency_half_angle_deg=45°`，`emergency_distance=0.18 m`；
- 50° 外的自车回波不参与。

### 路线不能贴墙

`turtlebot3_world` 的墙大约在 x≈±2.2 m。最初路线沿 x=-2.0 走，
离墙只有 0.2 m；一转向就正对墙，机器人原地左右摆动。
后来路线内收到 ±1.65 m，离墙和圆柱约 0.55 m，才稳定。

**教训**：规划路线时，把机器人半径、雷达盲区、控制误差都算进去；
不要用“贴边”的 waypoint。

### 为什么最后要转 360°

Cartographer 的回环检测需要看到之前的环境。最后原地转一圈，
可以让当前扫描和起点附近的 submap 建立更多约束，帮助闭环。

---

## 3.5 地图评估：怎么判断地图好不好

不要只看“图看起来像”。量化指标：

1. **覆盖率**：free + occupied 占比；未知太多说明没走完；
2. **墙线闭合**：边界是否连续；
3. **圆柱/障碍轮廓**：9 个圆柱是否都在、位置是否合理；
4. **重影**：同一面墙出现两条平行线，说明扫描匹配/位姿有问题；
5. **回环误差**：走回起点时，地图是否错位。

本仓库的 `homework/tools/plot_map.py` 可以统计 free/occupied/unknown 并
渲染 PNG：

```bash
python3 homework/tools/plot_map.py ~/maps/tb3_world.pgm \
  --output homework/maps/tb3_world.png
```

### 地图不好时怎么调

| 现象 | 可能原因 | 调整 |
|---|---|---|
| 墙重影 | 走太快/转向太快 | 降速，减小 `motion_filter` 阈值 |
| 地图不更新 | `/scan`、`/odom`、`/clock` 有问题 | 检查 topic 和 `use_sim_time` |
| 地图错位 | TF/初始位姿问题 | `tf2_echo map base_footprint` |
| 回环失败 | 没回到已见区域 | 多走回访，最后转一圈 |
| 空白太多 | 路线没覆盖 | 改 waypoints，增加走廊/中心环 |

---

## 3.6 Cartographer vs slam_toolbox

| 维度 | Cartographer | slam_toolbox |
|---|---|---|
| 方法 | 子图 + 扫描匹配 + 回环 + 位姿图 | Karto/图优化 |
| 配置 | Lua，参数多 | YAML/ROS 参数，ROS2 原生 |
| 优点 | 回环稳，大场景精度好，支持 3D | 轻量、易集成、支持 lifelong mapping |
| 缺点 | 配置复杂，CPU 占用高 | 大场景/3D 不如 Cartographer |
| 适用 | 课程/科研建图 | 快速集成、长期建图 |

**怎么选**：

- 课程要求 Cartographer → 用 Cartographer；
- 想要 ROS2 原生、快速接 Nav2 → slam_toolbox；
- 大场景/回环复杂 → Cartographer；
- 需要长期更新地图 → slam_toolbox。

### 自己验证的方法

用同一个 bag/同一个世界，分别跑两种 SLAM，保存地图，比较：

- 覆盖率、墙线闭合、重影；
- CPU/内存；
- 调参难度；
- 回环后的地图一致性。

---

## 3.7 3D SLAM 为什么不在本课程

3D SLAM 估计 6 自由度位姿（x,y,z,roll,pitch,yaw），输出点云/体素地图。
常见方案：

- LiDAR：LOAM、LeGO-LOAM、LIO-SAM、FAST-LIO；
- 视觉：ORB-SLAM3、RTAB-Map；
- 多传感器：LiDAR + IMU + 相机。

本课程不用 3D SLAM 的原因：

1. TB3 Burger 只有单线 2D 激光，没有 3D LiDAR/深度相机；
2. 课程目标是 2D 栅格 + Nav2，2D 地图直接可用；
3. 3D SLAM 计算量大，普通笔记本跑 Gazebo + 3D SLAM 很吃力；
4. 3D 点云要投影/降维才能用于 2D 导航。

**学习建议**：先把 2D SLAM 的扫描匹配、回环、位姿图吃透，再看 3D
SLAM 论文，会容易很多。

---

## 3.8 练习

### 练习 A（基础）

用 `auto_mapper` 建图，保存 PGM/YAML，运行 `plot_map.py`，记录
free/occupied/unknown 比例。

### 练习 B（调参）

改 `Cartographer` 的 `motion_filter.max_angle_radians` 和
`min_range/max_range`，观察地图差异。先预测，再实验。

### 练习 C（路线）

自己设计一条覆盖路线（waypoints 字符串），要求：

- 离墙/圆柱至少 0.5 m；
- 覆盖外圈 + 中心走廊；
- 最后转一圈。

用 `-p waypoints:="..."` 运行，比较地图。

### 练习 D（对比）

安装/运行 slam_toolbox，用同一个世界建图，和 Cartographer 比较。
提示：`ros2 launch slam_toolbox online_async_launch.py`。

### 练习 E（独立）

写一个节点：订阅 `/map`，统计 free/occupied/unknown 比例，并发布
`std_msgs/msg/Float32` 覆盖率。要求不复制 `plot_map.py`，自己用
`OccupancyGrid` 消息字段实现。

---

## 3.9 自测题

1. SLAM 的输出为什么既有 `/map` 又有 `map→odom`？
2. 扫描匹配和回环检测分别解决什么问题？
3. Cartographer 的子图是什么？为什么要分子图？
4. `tracking_frame`、`published_frame`、`odom_frame` 分别是什么？
5. `use_odometry=true` 的作用？关掉会怎样？
6. `auto_mapper` 的状态机有哪些状态？AVOID 怎么选方向？
7. 为什么雷达扇区不能直接用 `(center-angle_min)/inc` 取下标？
8. 地图重影/不更新/空白多，分别怎么排查？
9. Cartographer 和 slam_toolbox 的取舍？
10. 为什么本课程不直接上 3D SLAM？

> 下一章：拿建好的地图做 Nav2 自主导航。你会理解代价地图、AMCL、
> 规划器/控制器，并自己发目标、调参、排障。
