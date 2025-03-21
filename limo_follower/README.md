# Limo激光雷达跟随功能包

这个功能包实现了基于激光雷达的目标跟随功能，使Limo机器人能够实时扫描周围环境，寻找最近的物体并进行跟随。

## 功能特点

- 实时扫描周围环境，识别最近的物体
- 自动跟随检测到的最近物体
- 若扫描到新的距离更近的物体，会丢弃当前跟随目标，选择跟随新出现的更近目标
- 保持与被跟随物体的安全距离，车头朝向被跟随物体
- 支持动态参数调整，可以在运行时调整跟随距离、速度等参数

## 安装与使用

### 前提条件

- ROS Noetic
- Gazebo仿真环境
- Limo机器人仿真包

### 安装步骤

1. 将功能包放入ROS工作空间的src目录下
   ```bash
   cp -r limo_follower ~/catkin_ws/src/
   ```

2. 编译功能包
   ```bash
   cd ~/catkin_ws
   catkin_make
   ```

3. 刷新环境
   ```bash
   source ~/catkin_ws/devel/setup.bash
   ```

### 使用方法

1. 首先启动Limo机器人的Gazebo仿真环境
   ```bash
   roslaunch limo_gazebo_sim limo_four_diff.launch
   ```

2. 然后启动激光雷达跟随功能
   ```bash
   roslaunch limo_follower limo_follower.launch
   ```

## 参数配置

功能包支持通过动态参数配置调整以下参数：

- `maxSpeed`: 最大速度限制
- `targetDist`: 与目标物体保持的目标距离
- `P_v`, `I_v`, `D_v`: 线速度PID控制参数
- `P_w`, `I_w`, `D_w`: 角速度PID控制参数
- `winSize`: 激光扫描窗口大小
- `deltaDist`: 激光扫描过滤阈值

可以通过rqt_reconfigure工具在运行时调整这些参数：
```bash
rosrun rqt_reconfigure rqt_reconfigure
```

## 节点说明

### laser_tracker

激光雷达数据处理节点，负责处理激光扫描数据并识别最近的物体。

#### 订阅话题
- `limo1/scan` (sensor_msgs/LaserScan): 激光雷达扫描数据

#### 发布话题
- `object_tracker/current_position` (limo_follower/position): 目标物体的位置信息
- `object_tracker/info` (std_msgs/String): 跟踪状态信息

### laser_follower

目标跟随控制节点，负责根据目标位置控制机器人运动。

#### 订阅话题
- `object_tracker/current_position` (limo_follower/position): 目标物体的位置信息
- `object_tracker/info` (std_msgs/String): 跟踪状态信息

#### 发布话题
- `limo2/cmd_vel` (geometry_msgs/Twist): 机器人速度控制命令

## 工作原理

1. `laser_tracker`节点处理来自limo1机器人的激光扫描数据，识别最近的物体并发布其位置信息
2. `laser_follower`节点接收目标位置信息，使用PID控制器计算适当的线速度和角速度
3. 控制命令发送到limo2机器人，使其跟随目标物体
4. 如果检测到更近的物体，系统会自动切换跟随目标

## 故障排除

如果遇到问题，请检查：

1. 确保limo_gazebo_sim仿真环境正常运行
2. 检查激光雷达数据是否正常发布
3. 查看节点日志以获取详细错误信息
   ```bash
   rosrun rqt_console rqt_console
   ```

## 许可证

BSD
