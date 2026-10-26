# 第 1 章：ROS2 基础与第一个节点

> 本章目标：你能解释 ROS2 的通信模型，独立写出 publisher/subscriber，
> 会用 `ros2` 命令行和 rqt 工具观察系统，理解 launch/参数/命名空间/TF/
> 仿真时间。学完这一章，再看后面的 Gazebo、SLAM、Nav2，才不会“只会敲命令”。

---

## 1.1 先搞清楚：ROS2 到底解决了什么问题

一个机器人程序通常由很多部分组成：

```text
遥控程序  ─┐
SLAM 程序 ─┼─→  谁把数据传给谁？谁先启动？消息格式是什么？
导航程序  ─┘      不同语言（C++/Python）怎么通信？
```

如果没有中间件，你会写一堆 socket、序列化、线程、发现协议。
ROS2 就是一套**机器人软件中间件**：

- 规定节点怎么发现彼此；
- 规定消息怎么序列化；
- 提供话题/服务/动作/参数等通信模式；
- 提供 launch、TF、bag、rviz、rqt 等工具。

### ROS2 和 ROS1 的关键区别

| 维度 | ROS1 | ROS2 |
|---|---|---|
| 中心节点 | 必须有 `roscore` | 无中心，DDS 自动发现 |
| 通信 | 自定义 TCPROS | DDS / RMW，可换实现 |
| 多机器人 | `ROS_MASTER_URI` + `ROS_IP` | `ROS_DOMAIN_ID` + namespace |
| QoS | 简单 | 可靠/尽力、历史、深度、持久性 |
| 实时/嵌入式 | 较弱 | 更好 |

> 本课程用 ROS2 Humble。你会在多机器人章节看到：
> 不同机器/不同组用 `ROS_DOMAIN_ID` 隔离，同一台车内部用 namespace 区分。

### DDS 是什么（只需要理解到这个程度）

DDS 是一个**分布式发布/订阅协议**。每个节点启动后：

1. 在网络上广播“我存在”；
2. 发布者声明“我在哪个 topic 上发什么类型”；
3. 订阅者声明“我订阅哪个 topic、什么 QoS”；
4. 匹配成功后，数据直接点对点传输。

所以 ROS2 不需要 master，但**网络、端口、共享内存**会直接影响通信。
本课程遇到的“反复启停后话题有但收不到数据”，就是 DDS 端口/共享内存残留。

---

## 1.2 ROS2 的五个核心概念

| 概念 | 一句话 | 典型例子 |
|---|---|---|
| Node 节点 | 一个独立功能进程 | 遥控节点、SLAM 节点 |
| Topic 话题 | 单向、持续的数据流 | `/scan`、`/odom`、`/cmd_vel` |
| Service 服务 | 一问一答 | `/spawn_entity`、参数服务 |
| Action 动作 | 长任务，可取消、有反馈 | Nav2 的 `/navigate_to_pose` |
| Parameter 参数 | 节点运行时的配置 | `use_sim_time`、`radius` |

### 话题 vs 服务 vs 动作：怎么选

```text
数据一直流、多个订阅者 → Topic
立刻要一个结果、短 → Service
任务很久、要进度、可取消 → Action
```

例如：

- 激光雷达 10 Hz 持续发 → `/scan` 是 Topic；
- 让 Gazebo 生成一个机器人，调用一次就返回 → Service；
- 让机器人从 A 导航到 B，需要几十秒、要反馈剩余距离、可取消 → Action。

### QoS：为什么 `ros2 topic echo` 有时收不到

QoS 是发布者和订阅者之间的“契约”：

- **Reliability**：`RELIABLE`（保证送达）或 `BEST_EFFORT`（尽力而为）；
- **History**：保留最近 N 条或全部；
- **Durability**：`VOLATILE` 或 `TRANSIENT_LOCAL`（后加入也能拿到最后一条）；
- **Depth**：队列深度。

传感器数据常用 `BEST_EFFORT`（丢一两帧没关系，不能堵）。
如果订阅端用 `RELIABLE`，可能匹配不上，表现就是“话题存在但收不到”。

调试命令：

```bash
ros2 topic info /scan -v
ros2 topic echo /scan --once --qos-reliability best_effort
```

### 消息类型

ROS2 的消息类型长这样：

```text
geometry_msgs/msg/Twist
sensor_msgs/msg/LaserScan
nav_msgs/msg/Odometry
```

看一个消息有哪些字段：

```bash
ros2 interface show geometry_msgs/msg/Twist
ros2 interface show sensor_msgs/msg/LaserScan
```

> 记住这个习惯：**写代码前先看消息类型和字段**，不要凭记忆猜。

---

## 1.3 工作空间、功能包、编译系统

### 三个关键词

- **workspace 工作空间**：一个目录，里面有 `src/`、`build/`、`install/`、`log/`；
- **package 功能包**：ROS2 的最小组织单位，有 `package.xml`；
- **ament/colcon**：ROS2 的构建系统和构建工具。

本仓库：

```text
~/turtlebot3_ws/          ← 工作空间根目录
├── src/tb3_course/       ← 功能包
├── build/                ← 编译中间文件（不要提交 git）
├── install/              ← 编译产物（source 这里）
└── log/                  ← 编译日志
```

### 为什么必须 `source`

`source install/setup.bash` 会把 `install/` 里的路径加到环境变量
（`AMENT_PREFIX_PATH`、`LD_LIBRARY_PATH` 等）。不 source，`ros2 run`
就找不到你的包。

每个新终端都要：

```bash
source /opt/ros/humble/setup.bash        # ROS2 本体
source ~/turtlebot3_ws/install/setup.bash # 你的包
```

本仓库的 `tb3_env.sh` 已经帮你做了这两件事，还补了 Gazebo 模型路径：

```bash
source ~/turtlebot3_ws/tb3_env.sh
```

### 功能包的目录结构（ament_python）

```text
src/tb3_course/
├── package.xml          # 包名、依赖、许可证
├── setup.py             # Python 安装脚本 + console_scripts
├── setup.cfg            # 安装路径配置
├── resource/tb3_course  # ament 资源索引
├── tb3_course/          # Python 模块
│   ├── __init__.py
│   └── *.py
├── launch/              # launch 文件
└── test/                # 测试
```

`setup.py` 里最重要的是 `entry_points`：

```python
entry_points={
    'console_scripts': [
        'circle = tb3_course.turtle_circle:main',
        'pose_listener = tb3_course.pose_listener:main',
        # ...
    ],
},
```

含义：`ros2 run tb3_course circle` 会执行
`tb3_course/turtle_circle.py` 里的 `main()`。

---

## 1.4 第一个 publisher：让 turtlesim 画圆

### 原理

Publisher 做三件事：

1. 创建节点；
2. 创建 publisher，声明 topic 和消息类型；
3. 定时/事件触发时 `publish(msg)`。

### 代码：`turtle_circle.py`（本仓库实际文件）

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CirclePublisher(Node):
    def __init__(self):
        super().__init__('circle_publisher')
        self.declare_parameter('radius', 1.0)
        self.declare_parameter('linear_speed', 1.0)
        self.declare_parameter('rate_hz', 10.0)

        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        rate = max(0.1, float(self.get_parameter('rate_hz').value))
        self.timer = self.create_timer(1.0 / rate, self.timer_cb)

    def timer_cb(self):
        radius = max(0.01, float(self.get_parameter('radius').value))
        speed = float(self.get_parameter('linear_speed').value)

        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = speed / radius
        self.pub.publish(msg)
```

### 逐行理解

- `super().__init__('circle_publisher')`：节点名，`ros2 node list` 会看到 `/circle_publisher`；
- `declare_parameter`：声明参数，运行时可改；
- `create_publisher(Twist, topic, 10)`：消息类型、话题名、队列深度；
- `create_timer(period, callback)`：周期调用；
- `msg.linear.x = speed`：前进速度；
- `msg.angular.z = speed / radius`：角速度 = 线速度 / 半径。

### 数学：为什么 `ω = v / r`

匀速圆周运动：

```text
v = ω r   →   ω = v / r
```

所以：

- `v=1.0, r=0.5` → `ω=2.0`；
- `v=1.0, r=2.0` → `ω=0.5`。

### 动手

终端 1：

```bash
source ~/turtlebot3_ws/tb3_env.sh
ros2 run turtlesim turtlesim_node
```

终端 2：

```bash
source ~/turtlebot3_ws/tb3_env.sh
ros2 run tb3_course circle --ros-args -p radius:=0.5
```

终端 3 观察：

```bash
ros2 node list
ros2 topic list
ros2 topic info /turtle1/cmd_vel -v
ros2 topic echo /turtle1/cmd_vel --once
ros2 topic hz /turtle1/cmd_vel
```

期望：`linear.x = 1.0`，`angular.z = 2.0`，频率约 10 Hz。

### 关键练习（不要跳过）

1. 把 `radius` 改成 2.0，先预测 `angular.z` 是多少，再运行验证。
2. 把 `linear_speed` 改成 0.5，半径不变，圆的线速度/周期怎么变？
3. 写一个 subscriber 记录 `/turtle1/pose`，每 0.5 s 打印一次。
4. 让 `radius` 每隔 2 秒在 0.5 和 2.0 之间切换（提示：在 timer 里改参数或维护状态）。

---

## 1.5 第一个 subscriber：读取位姿

### 代码：`pose_listener.py`

```python
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose


class PoseListener(Node):
    def __init__(self):
        super().__init__('pose_listener')
        self.sub = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_cb, 10)

    def pose_cb(self, msg):
        self.get_logger().info(
            f'x={msg.x:.2f}, y={msg.y:.2f}, theta={msg.theta:.2f}',
            throttle_duration_sec=0.5)
```

### 理解 callback

- `create_subscription(类型, topic, 回调, 队列深度)`；
- 每来一条消息，ROS2 调用 `pose_cb`；
- `throttle_duration_sec=0.5` 限制日志频率，避免刷屏；
- 回调里**不要做耗时操作**（比如长时间循环），否则会阻塞执行器。

### 动手

```bash
ros2 run tb3_course pose_listener
```

同时运行 `circle`，观察位姿变化。

### 思考题

- 为什么 subscriber 的回调频率由 publisher 决定？
- 如果回调里 `sleep(1)`，消息会丢吗？为什么？（提示：队列深度、执行器）

---

## 1.6 节点、执行器与 `rclpy.spin`

一个节点可以：

- 创建多个 publisher/subscriber/timer/service；
- 这些回调由 **executor** 调度；
- `rclpy.spin(node)` 让节点一直运行，等待回调。

```python
rclpy.init(args=args)
node = CirclePublisher()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()
```

> 常见错误：忘记 `rclpy.init()`、忘记 `spin()`、在回调里调用
> `rclpy.shutdown()` 导致后续回调失效。第 5 章的 Nav2 目标时间戳问题
> 就和“节点时钟/生命周期”有关。

---

## 1.7 用命令行观察系统

### 节点

```bash
ros2 node list
ros2 node info /circle_publisher
```

`node info` 会列出这个节点的 publisher/subscriber/service/parameter。

### 话题

```bash
ros2 topic list
ros2 topic list -t          # 带类型
ros2 topic info /scan -v    # 发布/订阅者、QoS
ros2 topic echo /odom --once
ros2 topic hz /scan
ros2 topic bw /scan
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
```

### 服务

```bash
ros2 service list
ros2 service type /spawn_entity
ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity "{...}"
```

### 动作

```bash
ros2 action list
ros2 action info /navigate_to_pose
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{...}"
```

### 参数

```bash
ros2 param list
ros2 param get /circle_publisher radius
ros2 param set /circle_publisher radius 2.0
ros2 param dump /circle_publisher
```

### rqt

```bash
rqt_graph     # 节点/话题连接图
rqt_plot      # 曲线
rqt_console   # 日志
```

`rqt_graph` 是理解系统拓扑最快的工具。每次启动新系统，先看一眼图。

---

## 1.8 Launch：把多个节点组织起来

### 为什么需要 launch

手动开 5 个终端很麻烦，而且容易漏掉参数。Launch 文件可以：

- 一次启动多个节点；
- 传参数；
- 条件启动；
- 组织 namespace；
- 处理启动顺序。

### 一个最小 launch

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tb3_course',
            executable='circle',
            name='circle_publisher',
            parameters=[{'radius': 0.5}],
            output='screen',
        ),
    ])
```

运行：

```bash
ros2 launch tb3_course circle.launch.py
ros2 launch tb3_course circle.launch.py --show-args
```

### 参数文件（YAML）

```yaml
circle_publisher:
  ros__parameters:
    radius: 0.5
    linear_speed: 1.0
    rate_hz: 10.0
```

```python
parameters=[os.path.join(pkg_share, 'config', 'circle.yaml')]
```

### Remap 和 namespace

```python
Node(
    package='tb3_course',
    executable='circle',
    remappings=[('/turtle1/cmd_vel', '/turtle2/cmd_vel')],
    namespace='tb3_1',
)
```

效果：

- 话题变成 `/tb3_1/turtle2/cmd_vel`；
- 节点名变成 `/tb3_1/circle_publisher`。

多机器人章节会大量用到 namespace 和 remap。

---

## 1.9 TF2：机器人坐标变换

### 为什么需要 TF

机器人有很多坐标系：

```text
map           全局地图坐标系
 └── odom     里程计坐标系（连续但会漂移）
      └── base_footprint  机器人在地面的投影
           └── base_link  机器人本体
                ├── base_scan  激光雷达
                ├── imu_link   IMU
                └── wheel_left_link / wheel_right_link
```

每个传感器数据都带 `frame_id`。要把它和地图对齐，必须知道这些坐标系
之间的变换。TF2 就是维护这个变换树的库。

### 看 TF

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo map odom
```

`view_frames` 会生成 `frames.pdf`，画出整棵树。

### REP 105 的关键约定

- `map`：全局固定，SLAM/定位发布 `map→odom`；
- `odom`：连续、短期准确，但会漂移；
- `base_footprint`：机器人在地面的投影；
- 传感器 frame 通过静态 TF 挂在 `base_link`。

### 常见错误

- 只看 `odom` 不看 `map`，会以为机器人“没漂”；
- 时间戳不对：`tf2_echo` 报 extrapolation；
- 两个机器人用了相同 frame 名，TF 树冲突（第 5 章解决）。

---

## 1.10 仿真时间与 `/clock`

Gazebo 发布 `/clock`，所有节点用 `use_sim_time:=true` 时按仿真时间运行。

```bash
ros2 topic echo /clock --once
ros2 param get /robot_state_publisher use_sim_time
```

如果节点没开 `use_sim_time`：

- 它用系统时间，和 Gazebo 的 `/clock` 不一致；
- TF 查询会报“时间戳对不上”；
- timer 可能按真实时间走，控制周期和仿真不同步。

### 规则

- Gazebo 里的节点：`use_sim_time:=true`；
- 纯 turtlesim/离线节点：一般保持 false；
- 不确定时：`ros2 param get <node> use_sim_time`。

---

## 1.11 本章常见坑

| 现象 | 原因 | 处理 |
|---|---|---|
| `Package 'tb3_course' not found` | 没 source install | `source install/setup.bash` |
| `ros2 run` 找不到可执行文件 | `setup.py` 没加 entry point / 没重新 build | 加 entry point，`colcon build` |
| 话题存在但 echo 不到 | QoS 不匹配 | `--qos-reliability best_effort` |
| 改了代码没效果 | 用 `--symlink-install` 但没重新 build/source | 重新 build + source |
| TF 报 extrapolation | 时间戳/use_sim_time 不一致 | 统一 `use_sim_time` |
| 节点名冲突 | 同名节点 | 用 namespace 或 `name:=` |

---

## 1.12 练习：从模仿到独立

### 练习 A（基础）

写一个节点 `speed_watch`：

- 订阅 `/turtle1/pose`；
- 计算相邻两帧的线速度 `v = Δs/Δt` 和角速度 `ω = Δθ/Δt`；
- 每 0.5 s 打印一次。

要求：先画出数据流图，再写代码，最后用 `rqt_plot` 验证。

### 练习 B（参数/launch）

把 `circle` 的参数写进 YAML，用 launch 启动；再用 `ros2 param set`
运行时改半径，观察轨迹变化。

### 练习 C（调试）

故意把 `use_sim_time` 设为 false 启动一个 Gazebo 节点，观察 TF/时间
会发生什么；解释原因并恢复。

### 练习 D（独立）

不看 `pose_listener.py`，自己写一个 subscriber，订阅 `/scan`，
打印前方 ±30° 的最小距离。提示：

- 先 `ros2 interface show sensor_msgs/msg/LaserScan`；
- 注意 `angle_min`、`angle_increment`、`ranges` 的索引；
- 过滤 `inf` 和小于 `range_min` 的值。

---

## 1.13 自测题（能讲清楚才算会）

1. ROS2 为什么不需要 `roscore`？DDS 在其中做什么？
2. Topic、Service、Action 的区别？各举一个本课程的例子。
3. QoS 的 Reliability 和 Durability 分别解决什么问题？
4. `source install/setup.bash` 到底改了什么环境变量？
5. 为什么改了 `src/` 里的代码，不重新 build 可能不生效？
6. `map`、`odom`、`base_footprint` 的区别？谁发布 `map→odom`？
7. `use_sim_time` 不设置会发生什么？怎么排查？
8. `ros2 topic echo` 收不到数据，你会按什么顺序排查？

> 下一章：把节点放到 Gazebo 里，理解机器人模型、差速底盘、激光雷达，
> 并写开环/闭环控制器。
