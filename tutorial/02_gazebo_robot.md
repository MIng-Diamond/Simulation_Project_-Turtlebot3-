# 第 2 章：Gazebo、机器人模型与运动控制

> 本章目标：理解 Gazebo Classic 的 server/client/插件结构，看懂 TurtleBot3
> 的 SDF/URDF、差速底盘和激光雷达；能解释开环/闭环控制的误差，独立写出
> 一个基于 `/odom` 的闭环控制器，并知道怎么排查“车不动/雷达不对/车翻车”。

---

## 2.1 Gazebo Classic 的三个角色

```text
gzserver   ← 物理引擎、传感器、插件（没有它就没有仿真）
gzclient   ← 图形界面（关了不影响仿真）
master     ← 进程间通信（gzserver/gzclient/工具通过它连接）
```

本课程用 Gazebo Classic 11（`gazebo --version` 应显示 11.10.x）。
TB4 用的是 Ignition，两者命令/插件体系不同，不要混用。

### 启动一个世界时发生了什么

`ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py` 大致做：

1. 启动 `gzserver`，加载 `.world` 文件；
2. 启动 `gzclient`（GUI）；
3. 启动 `robot_state_publisher`，读取 URDF，发布 `/robot_description` 和静态 TF；
4. 调用 `spawn_entity.py`，通过 `/spawn_entity` 服务把机器人 SDF 放进 Gazebo。

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### 一个本机真实踩过的坑：Gazebo 卡在加载世界

`gazebo_ros` 的 launch 默认把 `GAZEBO_MODEL_PATH` 设成
`turtlebot3_gazebo/models`，不包含系统自带的 `ground_plane`、`sun`。
如果本机访问 `models.gazebosim.org` 超时，Gazebo 会一直尝试联网下载模型，
世界加载不完，`/spawn_entity` 服务不出现，机器人 spawn 失败。

本仓库的 `tb3_env.sh` 修复了它：

```bash
export GAZEBO_MODEL_PATH="/usr/share/gazebo-11/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
export GAZEBO_RESOURCE_PATH="/usr/share/gazebo-11${GAZEBO_RESOURCE_PATH:+:$GAZEBO_RESOURCE_PATH}"
```

**排查方法**：看 `~/.gazebo/server-*/default.log` 里是否卡在
“Getting models from http://models.gazebosim.org/”。

### `gzserver` / `gzclient` 进程

```bash
ps -eo pid,cmd | grep -E 'gzserver|gzclient'
```

- 只关 GUI：`pkill -x gzclient`，`gzserver` 继续跑，传感器/物理不受影响；
- 反复 `Ctrl+C` 可能留下孤儿进程，见 2.8。

---

## 2.2 TurtleBot3 的模型：URDF 和 SDF

### URDF 和 SDF 分别是什么

- **URDF**：ROS 的机器人描述格式，描述 link/joint/外观/惯性，主要给
  `robot_state_publisher`、RViz 用；
- **SDF**：Gazebo 的仿真描述格式，除了 link/joint，还能带 `<plugin>`
  （差速驱动、激光、IMU 等），是 Gazebo 真正仿真的对象。

TurtleBot3 的 SDF：

```bash
/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf
```

### 差速驱动插件

```xml
<plugin name="turtlebot3_diff_drive" filename="libgazebo_ros_diff_drive.so">
  <update_rate>30</update_rate>
  <left_joint>wheel_left_joint</left_joint>
  <right_joint>wheel_right_joint</right_joint>
  <wheel_separation>0.160</wheel_separation>
  <wheel_diameter>0.066</wheel_diameter>
  <command_topic>cmd_vel</command_topic>
  <publish_odom>true</publish_odom>
  <publish_odom_tf>true</publish_odom_tf>
  <odometry_topic>odom</odometry_topic>
  <odometry_frame>odom</odometry_frame>
  <robot_base_frame>base_footprint</robot_base_frame>
</plugin>
```

它做的事：

- 订阅 `cmd_vel`；
- 把线速度/角速度换算成左右轮速；
- 用轮子编码器/世界位姿算出 `odom`；
- 发布 `odom→base_footprint` 的 TF。

### 差速底盘运动学

```text
v_left  = v - ω * L/2
v_right = v + ω * L/2
```

其中 `L` 是轮距（0.160 m），`v` 是线速度，`ω` 是角速度。

反过来：

```text
v = (v_left + v_right) / 2
ω = (v_right - v_left) / L
```

**重要**：`/cmd_vel` 的 `linear.x` 和 `angular.z` 不是直接给轮子的，
是插件帮你换算的。

### 激光雷达插件

```xml
<plugin name="turtlebot3_laserscan" filename="libgazebo_ros_ray_sensor.so">
  <ros>
    <remapping>~/out:=scan</remapping>
  </ros>
  <output_type>sensor_msgs/LaserScan</output_type>
  <frame_name>base_scan</frame_name>
  <update_rate>5</update_rate>
  <ray>
    <scan>
      <horizontal>
        <samples>360</samples>
        <min_angle>0.000000</min_angle>
        <max_angle>6.280000</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.120000</min>
      <max>3.5</max>
      <resolution>0.015000</resolution>
    </range>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev>
    </noise>
  </ray>
</plugin>
```

关键点：

- Gazebo Classic 的 ray sensor 是 **CPU 射线**，不依赖 GPU 离屏渲染；
- TB4 的 Ignition `gpu_lidar` 才受 NVIDIA PRIME/EGL 影响；
- 所以本机 TB3 的 `/scan` 不会出现“全是 0.164 m”的 TB4 问题。

### 在 Gazebo 里看模型

启动世界后，在 GUI 左侧 `World → Models` 展开，可以看到 `burger`、
`ground_plane`、`sun`、`turtlebot3_world` 等。选中模型可以看到
`pose`、`link`、`plugin` 等属性。

---

## 2.3 启动并验证

### 启动

```bash
source ~/turtlebot3_ws/tb3_env.sh
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

等待 Gazebo 加载完成。判据：

```bash
ros2 topic list
```

至少应有：

```text
/clock  /cmd_vel  /imu  /odom  /scan  /tf  /tf_static
```

### 检查频率和数据

```bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic echo /odom --once
ros2 topic echo /scan --once
```

### 分析 `/scan`

`LaserScan` 的关键字段：

| 字段 | 含义 |
|---|---|
| `angle_min` / `angle_max` | 扫描起止角（本机 0..2π） |
| `angle_increment` | 每束角度间隔（2π/360） |
| `range_min` / `range_max` | 0.12 / 3.5 m |
| `ranges` | 每束距离，`inf` 表示没打到 |

本机实测（`turtlebot3_world`）：

- 360 束，约 324 个有限值；
- min ≈ 0.505 m，median ≈ 1.053 m，max ≈ 3.328 m；
- 没有 0.164 m 的假值。

### 一个重要细节：扫描角范围是 0..2π

很多教程假设 `angle_min=-π`。本机是 `0.0`。如果你用

```python
center_index = int((center_rad - angle_min) / angle_increment)
```

来取扇区，当 `center_rad=0` 时，负索引会被忽略，导致**只检查一半**；
当 `center= -π/2` 时，索引全为负，直接返回 `inf`。

正确做法：遍历所有束，把角度归一化到 `[-π, π]` 再比较：

```python
for i, r in enumerate(scan.ranges):
    angle = scan.angle_min + i * scan.angle_increment
    if abs(normalize_angle(angle - center_rad)) <= half_rad:
        ...
```

`auto_mapper` 和 `trail_follower` 都用了这个写法。第 3 章会细讲。

---

## 2.4 让机器人动起来：`/cmd_vel`

```bash
# 前进 2 秒
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
```

观察：

```bash
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_footprint
```

### 必须知道的坑：最后一条 `/cmd_vel` 会保持

Gazebo 的差速插件不会因为发布者退出就自动停车。你发完非零速度后，
如果不再发零速度，机器人会一直走。

```bash
# 测试完一定补零
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

本课程的多机器人实验早期就因此让两台车撞墙侧翻。
`velocity_mirror` 后来加了 0.5 s 看门狗，见第 5 章。

---

## 2.5 开环控制：按时间走正方形

### 代码：`move_square.py`

```python
class MoveSquare(Node):
    def __init__(self):
        super().__init__('move_square')
        self.declare_parameter('side_time', 3.0)
        self.declare_parameter('turn_time', 2.0)
        self.declare_parameter('linear_speed', 0.18)
        self.declare_parameter('angular_speed', math.pi / 4.0)

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.side_index = 0
        self.state = 'FORWARD'
        self.state_start = self.get_clock().now()
        self.timer = self.create_timer(0.05, self.loop)

    def loop(self):
        cmd = Twist()
        if self.side_index >= 4:
            self.pub.publish(cmd)
            self.timer.cancel()
            return

        if self.state == 'FORWARD':
            cmd.linear.x = speed
            if self.elapsed() >= side_time:
                self.switch_state('TURN')
        elif self.state == 'TURN':
            cmd.angular.z = angular
            if self.elapsed() >= turn_time:
                self.side_index += 1
                self.switch_state('FORWARD')
        self.pub.publish(cmd)
```

### 为什么它是开环

控制量只由“时间”决定，不看机器人实际走到哪：

- 轮子打滑、加速过程、计时误差都会累积；
- `side_time=3 s, v=0.12` 本应走 0.36 m，实际可能多/少；
- 转 90° 的 `turn_time` 也不准。

本机实测：

```text
起点 (-2.000, -0.500)
终点 (-2.344, -0.508)
闭合误差 ≈ 0.344 m
```

### 练习

1. 改 `linear_speed`/`side_time`，预测边长和闭合误差。
2. 把 `turn_time` 从 2.0 改成 1.5，会发生什么？为什么？
3. 在 `TURN` 状态加一个 `cmd.linear.x = 0`，确保原地转。

---

## 2.6 闭环控制：用 `/odom` 走正方形

### 原理

每一步都用里程计反馈：

1. 记录本边起点；
2. 向前走到“离起点 `side_length` 米”；
3. 用当前 yaw 计算目标 yaw = 名义航向 + 90°，原地转到目标；
4. 直行时用航向 P 控制抑制左右轮差；
5. 重复四次，输出闭合误差。

### 关键数学

四元数转 yaw：

```python
def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))
```

角度归一化：

```python
def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle
```

距离：

```python
dist = math.hypot(x - start_x, y - start_y)
```

航向误差：

```python
heading_error = normalize_angle(side_yaw - yaw)
cmd.angular.z = clamp(kp_heading * heading_error, -max_correction, max_correction)
```

### 代码：`square_odom.py` 的状态机

```text
WAIT_ODOM
  └── 收到第一帧 odom → 记录 initial_xy / segment_start_xy / side_yaw
       → FORWARD

FORWARD
  ├── dist < side - tol：直行 + 航向修正
  └── 到达：停车，side_index++
        ├── 4 边完成 → 输出 closure error，结束
        └── 否则 → target_yaw = side_yaw + 90° → TURN

TURN
  ├── |error| > tol：原地转，角速度 = clamp(1.5*error, ±max)
  └── 到位：segment_start_xy = 当前位姿，side_yaw = target_yaw → FORWARD
```

### 实测

```text
起点 (-2.000, -0.500, yaw=0)
终点 (-1.992, -0.496)
闭合误差 ≈ 0.005 m
```

对比：

| 控制 | 边长 | 闭合误差 |
|---|---|---|
| 开环 `move_square` | 0.36 m | 0.344 m |
| 闭环 `square_odom` | 0.40 m | 0.005 m |

### 为什么闭环好这么多

- 每边都按实际位移停，不受打滑/加速影响；
- 转向按实际 yaw 闭环，不靠时间；
- 直行时航向修正，抑制左右轮差；
- 容差 `tolerance=0.005` 和 20 Hz 控制周期决定了最终精度。

### 练习

1. 把 `side_length` 改成 1.0，观察闭合误差；为什么误差没有线性变大？
2. 把 `tolerance` 改成 0.05，闭合误差会怎样？
3. 把 `kp_heading` 改成 0 和 5，观察直行漂移/抖动。
4. 把正方形改成矩形（两条边不同长度），需要改哪些状态？
5. 加一个 `/scan` 安全层：前方 < 0.3 m 时停车。提示：订阅 `LaserScan`，
   用 `sector_min`。

---

## 2.7 传感器数据怎么用于控制

### 前方最近距离

```python
def sector_min(scan, center_rad, half_rad):
    best = float('inf')
    for i, r in enumerate(scan.ranges):
        if not math.isfinite(r) or r <= scan.range_min:
            continue
        angle = scan.angle_min + i * scan.angle_increment
        if abs(normalize_angle(angle - center_rad)) <= half_rad:
            best = min(best, r)
    return best
```

- `center=0` 前方；
- `center=+π/2` 左侧；
- `center=-π/2` 右侧。

### 自车回波

TB3 的雷达装在车体上方，侧面 50° 以外可能扫到自己的结构（0.12–0.19 m）。
做避障时：

- 前方用窄扇区（±20°）；
- 近距用 ±45° 紧急扇区；
- 不要把 50°+ 的自车回波当成障碍。

`auto_mapper` 的参数就是按这个思路设计的。

---

## 2.8 进程、DDS 和“看起来在跑其实没数据”

### 现象

反复 `Ctrl+C` 启动/停止后：

- `ros2 topic list` 刷 `Failed init_port fastrtps_portXXXX`；
- 话题存在但 `echo` 不到；
- 控制器报 `Could not contact service /controller_manager/...`；
- `ros2 topic list` 直接崩。

### 原因

`ros2 launch` 会拉起几十个节点；`Ctrl+C` 只杀 launch 本身，子节点可能
变成孤儿进程，继续占用 DDS 的 UDP 端口（7400+）。端口耗尽后新节点
无法建立通信。

### 处理

```bash
bash ~/turtlebot3_ws/stop_tb3.sh
```

它会：

- 杀掉 `turtlebot3_gazebo/cartographer/navigation2/teleop` 等 launch 及其后代；
- 清理孤儿 `parameter_bridge`、`cartographer_node`、`gzserver` 等；
- 在没有 ROS 进程时清理 `/dev/shm/fastrtps_*`。

**习惯**：每次实验结束都跑一次停机脚本，再开始下一轮。

---

## 2.9 本章常见坑

| 现象 | 原因 | 处理 |
|---|---|---|
| Gazebo 卡住，`/spawn_entity` 不出现 | `GAZEBO_MODEL_PATH` 缺系统模型 | `source tb3_env.sh` |
| `/scan` 全是 `inf` | 空世界没有障碍 | 正常；换 `turtlebot3_world` |
| `/scan` 全是 0.164 m | TB4/Ignition GPU 雷达问题，不是 TB3 | 本章用 TB3 CPU 雷达，不会出现 |
| 车不听使唤/一直走 | 最后一条 `cmd_vel` 保持 | 发零速度；加看门狗 |
| 车翻车 | 撞墙/圆柱；速度太快；没有避障 | 降速、加安全距离、补零 |
| TF 报 extrapolation | `use_sim_time` 不一致 | 统一 true |
| 改了代码没效果 | 没 build/source | `colcon build --symlink-install` + source |

---

## 2.10 自测题

1. `gzserver` 和 `gzclient` 各做什么？只关 GUI 会影响 `/scan` 吗？
2. URDF 和 SDF 的区别？`libgazebo_ros_diff_drive` 负责什么？
3. 差速底盘已知 `v, ω, L`，怎么算左右轮速？
4. 为什么 TB3 的 `/scan` 不会出现 TB4 的 0.164 m 问题？
5. `LaserScan` 的 `angle_min/angle_increment/ranges` 怎么对应到角度？
6. 开环正方形误差 0.344 m，闭环 0.005 m，差异来自哪些环节？
7. 为什么发完 `/cmd_vel` 必须补零？看门狗怎么设计？
8. 反复启停后通信异常，怎么排查和清理？

> 下一章：用 Cartographer 建图。你会看到“里程计漂移”怎么被扫描匹配
> 和回环修正，以及为什么自动巡航路线不能贴墙。
