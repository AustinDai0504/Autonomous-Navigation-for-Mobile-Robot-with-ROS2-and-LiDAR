# ROS2 Autonomous Logistics AMR

An autonomous mobile robot (AMR) for warehouse logistics applications built with ROS2 Jazzy, Gazebo Harmonic, and Nav2. The robot autonomously navigates a warehouse environment using SLAM-generated maps and AMCL localization.

---

## Architecture Overview

<p align="center">
  <img src="docs/system_architecture.png" alt="ROS2 Logistics AMR System Architecture" width="100%">
</p>

<p align="center">
  <img src="docs/navigation_data_flow.png" alt="Nav2 Data Flow" width="100%">
</p>

<p align="center">
  <img src="docs/topic_tf_flow.png" alt="Topic and TF Flow" width="100%">
</p>

---



## Features

- **Autonomous Navigation** — Nav2 stack with AMCL localization and NavFn global planner
- **SLAM Mapping** — Real-time warehouse map generation using SLAM Toolbox
- **Sim-to-Real Ready** — Full ROS2 Jazzy + Gazebo Harmonic pipeline with ros2_control hardware interfaces
- **Containerised** — Complete Docker stack for reproducible deployment
- **CI/CD** — GitHub Actions pipeline that builds and tests on every push
- **Warehouse World** — Custom Gazebo SDF environment with walls and shelf obstacles

---

## Stack

| Component | Technology |
|-----------|-----------|
| Robot OS | ROS2 Jazzy |
| Simulation | Gazebo Harmonic 8.x |
| Navigation | Nav2 (AMCL + NavFn + RegPP Controller) |
| SLAM | SLAM Toolbox |
| Hardware Interface | ros2_control + diff_drive |
| Containerisation | Docker + docker-compose |
| CI/CD | GitHub Actions |
| Language | Python, C++ |

---

## Quick Start

### Prerequisites
- Ubuntu 24.04
- ROS2 Jazzy
- Gazebo Harmonic
- Docker (optional)

### Install dependencies

```bash
sudo apt install -y \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-ros-gz \
  ros-jazzy-rmw-cyclonedds-cpp \
  python3-colcon-common-extensions
```

### Build

```bash
git clone https://github.com/DAI-Qian/ros2_logistics_amr.git
cd ros2_logistics_amr
colcon build --symlink-install
source install/setup.bash
```

### Run

**Terminal 1 — Gazebo simulation:**
```bash
ros2 launch amr_bringup amr_gazebo.launch.py
```

**Terminal 2 — Nav2 autonomous navigation:**
```bash
ros2 launch amr_bringup nav2.launch.py
```

**Terminal 3 — RViz visualisation:**
```bash
ros2 run rviz2 rviz2
```

Set Fixed Frame to `map`, add `/map` and `/scan` topics, then use **2D Goal Pose** to send navigation goals.

### Docker

```bash
cd docker
docker-compose up
```

---

## How It Works

1. **Robot spawns** in a Gazebo warehouse world with walls and shelves
2. **SLAM Toolbox** builds a 2D occupancy map as the robot explores
3. **AMCL** localises the robot on the saved map using lidar scan matching
4. **Nav2** plans a collision-free path and executes it using the Regulated Pure Pursuit controller
5. **ros2_control** sends velocity commands to the differential drive plugin

---

## 系统架构图

下面的图基于当前仓库中的 launch 文件、URDF/Xacro、Nav2 参数、Gazebo world、地图文件和 `amr_controller` 节点整理。
### 1. 软件包与文件组织

```mermaid
flowchart TB
    Repo["ros2_logistics_amr workspace"]

    Repo --> Bringup["src/amr_bringup<br/>系统启动入口"]
    Repo --> Description["src/amr_description<br/>机器人模型"]
    Repo --> Navigation["src/amr_navigation<br/>导航与建图配置"]
    Repo --> Controller["src/amr_controller<br/>自定义 Python 节点"]
    Repo --> Maps["maps<br/>地图与充电桩数据库"]
    Repo --> Worlds["worlds<br/>Gazebo 仓库环境"]
    Repo --> Docker["docker<br/>容器化运行环境"]

    Bringup --> GazeboLaunch["launch/amr_gazebo.launch.py<br/>启动 Gazebo、spawn robot、ROS-GZ bridge、TF"]
    Bringup --> Nav2Launch["launch/nav2.launch.py<br/>启动 Nav2 bringup 与初始位姿发布"]

    Description --> Xacro["urdf/amr_robot.urdf.xacro<br/>底盘、双轮、脚轮、激光雷达、Gazebo 插件"]

    Navigation --> Nav2Params["config/nav2_params.yaml<br/>AMCL、Planner、Controller、Costmaps、Docking"]
    Navigation --> SlamParams["config/slam_toolbox.yaml<br/>SLAM Toolbox 建图参数"]

    Controller --> OdomTF["odom_tf_broadcaster.py<br/>/odom -> odom->base_footprint TF"]
    Controller --> InitialPose["initial_pose_publisher.py<br/>发布 /initialpose"]

    Maps --> WarehouseMap["warehouse_map.yaml / .pgm<br/>Nav2 静态地图"]
    Maps --> DockDB["dock_database.json<br/>Docking server 数据"]

    Worlds --> WarehouseWorld["warehouse.sdf<br/>墙体、货架、地面、灯光、物理与传感插件"]
```

### 2. 运行时总体架构

```mermaid
flowchart LR
    User["用户 / RViz2<br/>2D Goal Pose / Waypoints"] --> Nav2Action["Nav2 Action 接口<br/>NavigateToPose / NavigateThroughPoses"]

    subgraph Sim["Gazebo Harmonic 仿真层"]
        World["warehouse.sdf<br/>仓库、墙体、货架"]
        RobotModel["amr_robot.urdf.xacro<br/>差速底盘 + 2D LiDAR"]
        DiffDrive["gz-sim-diff-drive-system<br/>订阅 cmd_vel，发布 odom"]
        Lidar["gpu_lidar sensor<br/>发布 scan"]
        Clock["Gazebo clock"]
        World --> RobotModel
        RobotModel --> DiffDrive
        RobotModel --> Lidar
        World --> Clock
    end

    subgraph Bridge["ros_gz_bridge"]
        CmdBridge["/cmd_vel<br/>Twist"]
        OdomBridge["/odom<br/>Odometry"]
        ScanBridge["/scan<br/>LaserScan"]
        ClockBridge["/clock<br/>Clock"]
        TFBridge["/tf<br/>TFMessage"]
    end

    subgraph ROS["ROS2 Jazzy 运行层"]
        RSP["robot_state_publisher<br/>robot_description -> robot TF"]
        OdomTF["odom_tf_broadcaster<br/>/odom -> odom->base_footprint"]
        InitialPose["initial_pose_publisher<br/>/initialpose"]
        MapServer["map_server<br/>warehouse_map.yaml"]
        AMCL["amcl<br/>map + scan + odom -> map->odom"]
        Nav2["Nav2 bringup<br/>BT Navigator / Planner / Controller / Costmaps"]
        Slam["slam_toolbox<br/>可选建图模式"]
    end

    User --> Nav2
    Nav2Action --> Nav2
    Nav2 --> CmdBridge --> DiffDrive
    DiffDrive --> OdomBridge --> OdomTF
    DiffDrive --> OdomBridge --> Nav2
    Lidar --> ScanBridge --> AMCL
    Lidar --> ScanBridge --> Nav2
    Lidar --> ScanBridge --> Slam
    Clock --> ClockBridge
    ClockBridge --> RSP
    ClockBridge --> OdomTF
    ClockBridge --> InitialPose
    ClockBridge --> AMCL
    ClockBridge --> Nav2
    ClockBridge --> Slam
    RSP --> Nav2
    OdomTF --> Nav2
    InitialPose --> AMCL
    MapServer --> AMCL
    MapServer --> Nav2
    AMCL --> Nav2
```

### 3. Nav2 导航数据流

```mermaid
flowchart TB
    Goal["RViz2 / 上层任务<br/>导航目标"] --> BTN["bt_navigator<br/>行为树调度"]

    subgraph Localization["定位"]
        Map["map_server<br/>warehouse_map.yaml"]
        Scan["/scan<br/>LaserScan"]
        Odom["/odom<br/>Odometry"]
        Initial["/initialpose<br/>初始位姿"]
        AMCL["amcl<br/>粒子滤波定位<br/>发布 map->odom"]
        Map --> AMCL
        Scan --> AMCL
        Odom --> AMCL
        Initial --> AMCL
    end

    subgraph Planning["全局规划"]
        GCostmap["global_costmap<br/>static + obstacle + inflation"]
        Planner["planner_server<br/>NavfnPlanner"]
        Map --> GCostmap
        Scan --> GCostmap
        GCostmap --> Planner
    end

    subgraph Control["局部控制"]
        LCostmap["local_costmap<br/>voxel + inflation<br/>rolling window 3m x 3m"]
        Controller["controller_server<br/>Regulated Pure Pursuit"]
        Smoother["velocity_smoother<br/>cmd_vel_smoothed"]
        Collision["collision_monitor<br/>安全停止区"]
        Scan --> LCostmap
        Odom --> Controller
        LCostmap --> Controller
        Controller --> Smoother
        Smoother --> Collision
    end

    subgraph Recovery["恢复与任务插件"]
        Behavior["behavior_server<br/>spin / backup / wait / assisted_teleop"]
        Waypoint["waypoint_follower<br/>WaitAtWaypoint"]
        Docking["docking_server<br/>SimpleChargingDock"]
    end

    AMCL --> BTN
    BTN --> Planner
    Planner --> Controller
    BTN --> Behavior
    BTN --> Waypoint
    BTN --> Docking
    Collision --> Cmd["/cmd_vel<br/>发送到 Gazebo 差速驱动"]
```

### 4. Topic 与 TF 关系

```mermaid
flowchart LR
    subgraph Topics["主要 ROS Topics"]
        ScanTopic["/scan<br/>sensor_msgs/LaserScan"]
        OdomTopic["/odom<br/>nav_msgs/Odometry"]
        CmdTopic["/cmd_vel<br/>geometry_msgs/Twist"]
        ClockTopic["/clock<br/>rosgraph_msgs/Clock"]
        InitTopic["/initialpose<br/>PoseWithCovarianceStamped"]
        MapTopic["/map<br/>nav_msgs/OccupancyGrid"]
    end

    subgraph Producers["生产者"]
        GzLidar["Gazebo LiDAR"]
        GzDiff["Gazebo DiffDrive"]
        GzClock["Gazebo Clock"]
        InitNode["initial_pose_publisher"]
        MapServer2["map_server"]
        Nav2Controller["Nav2 controller / collision_monitor"]
    end

    subgraph Consumers["消费者"]
        AMCL2["AMCL"]
        Costmaps["Global / Local Costmaps"]
        Nav2Stack["Nav2 Stack"]
        DiffDrive2["Gazebo DiffDrive"]
        AllSimNodes["use_sim_time nodes"]
    end

    GzLidar --> ScanTopic --> AMCL2
    ScanTopic --> Costmaps
    GzDiff --> OdomTopic --> AMCL2
    OdomTopic --> Nav2Stack
    Nav2Controller --> CmdTopic --> DiffDrive2
    GzClock --> ClockTopic --> AllSimNodes
    InitNode --> InitTopic --> AMCL2
    MapServer2 --> MapTopic --> AMCL2
    MapTopic --> Costmaps
```

```mermaid
flowchart TB
    MapFrame["map"]
    OdomFrame["odom"]
    BaseFootprint["base_footprint"]
    BaseLink["base_link"]
    LeftWheel["left_wheel"]
    RightWheel["right_wheel"]
    Caster["caster_wheel"]
    LidarFrame["lidar_link"]

    MapFrame -- "AMCL: map -> odom" --> OdomFrame
    OdomFrame -- "odom_tf_broadcaster: odom -> base_footprint" --> BaseFootprint
    BaseFootprint -- "robot_state_publisher: fixed base_joint" --> BaseLink
    BaseLink -- "continuous left_wheel_joint" --> LeftWheel
    BaseLink -- "continuous right_wheel_joint" --> RightWheel
    BaseLink -- "fixed caster_joint" --> Caster
    BaseLink -- "fixed lidar_joint" --> LidarFrame
```

### 5. 启动时序

```mermaid
sequenceDiagram
    participant U as User
    participant GL as amr_gazebo.launch.py
    participant GZ as Gazebo Harmonic
    participant RSP as robot_state_publisher
    participant BR as ros_gz_bridge
    participant TF as odom_tf_broadcaster
    participant NL as nav2.launch.py
    participant N2 as Nav2 bringup
    participant IP as initial_pose_publisher
    participant AMCL as AMCL

    U->>GL: ros2 launch amr_bringup amr_gazebo.launch.py
    GL->>GZ: load worlds/warehouse.sdf
    GL->>RSP: publish robot_description from Xacro
    GL->>GZ: spawn amr_robot from /robot_description
    GL->>BR: bridge /cmd_vel, /odom, /scan, /clock, /tf
    GL->>TF: start /odom -> TF broadcaster
    GZ-->>BR: publish scan / odom / clock
    BR-->>TF: /odom
    TF-->>N2: odom->base_footprint TF

    U->>NL: ros2 launch amr_bringup nav2.launch.py
    NL->>N2: start nav2_bringup with map and params
    N2->>AMCL: load AMCL, map_server, costmaps, planner, controller
    NL->>IP: wait 5s, then start initial pose node
    IP-->>AMCL: publish /initialpose once
    AMCL-->>N2: publish map->odom TF
    U->>N2: send 2D Goal Pose in RViz
    N2-->>BR: publish /cmd_vel
    BR-->>GZ: drive differential robot
```

### 6. 仿真机器人与物理环境

```mermaid
flowchart TB
    subgraph Robot["AMR Robot"]
        Footprint["base_footprint"]
        Base["base_link<br/>0.6m x 0.4m x 0.2m"]
        Left["left_wheel<br/>radius 0.1m"]
        Right["right_wheel<br/>radius 0.1m"]
        Caster2["caster_wheel"]
        Lidar2["lidar_link<br/>360 samples, 10m max range"]
        Footprint --> Base
        Base --> Left
        Base --> Right
        Base --> Caster2
        Base --> Lidar2
    end

    subgraph Plugins["Gazebo Plugins"]
        DD["DiffDrive<br/>cmd_vel -> wheel motion -> odom"]
        LidarPlugin["gpu_lidar<br/>scan topic, 10 Hz"]
    end

    subgraph Warehouse["Warehouse World"]
        Ground["50m x 50m ground plane"]
        Walls["20m x 20m boundary walls"]
        Shelves["4 shelf obstacles<br/>shelf_1..shelf_4"]
        Physics["1ms physics step<br/>real_time_factor 1.0"]
    end

    Left --> DD
    Right --> DD
    Lidar2 --> LidarPlugin
    Ground --> Footprint
    Walls --> Lidar2
    Shelves --> Lidar2
    Physics --> DD
```

### 7. 在线建图与离线导航流程

```mermaid
flowchart LR
    subgraph Mapping["建图阶段"]
        Explore["遥控 / 探索仓库"]
        Scan3["/scan"]
        Odom3["/odom + TF"]
        Slam2["slam_toolbox<br/>mode: mapping<br/>scan matching + loop closing"]
        SaveMap["map_saver<br/>保存 warehouse_map.yaml / .pgm"]
        Explore --> Scan3
        Explore --> Odom3
        Scan3 --> Slam2
        Odom3 --> Slam2
        Slam2 --> SaveMap
    end

    subgraph NavigationMode["导航阶段"]
        StaticMap["warehouse_map.yaml"]
        AMCL3["AMCL 定位"]
        Nav2Mode["Nav2 规划与控制"]
        Cmd2["/cmd_vel"]
        StaticMap --> AMCL3
        Scan3 --> AMCL3
        AMCL3 --> Nav2Mode
        Nav2Mode --> Cmd2
    end

    SaveMap --> StaticMap
```
