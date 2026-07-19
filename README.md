<img src="img/header.jpg" alt="Header" width="100%"/>

# Outdoor Navigation

基于 [CMU autonomous_exploration_development_environment](https://www.cmu-exploration.com) 二次开发的地面机器人自主导航系统，
面向四足机器人 + 双 Livox MID360 + bynav RTK 的室外场景，新增了**先验地图定位（map_localization）**模块，
用一次建图换取此后可重复、可漂移修正的全局定位，同时保留 CMU 原生的局部规划 / 地形分析 / 探索栈。

## 系统总览

```mermaid
flowchart LR
    subgraph 感知
        L1[MID360 · 身体水平雷达]
        L2[MID360 · 云台/狗头雷达]
        R[bynav RTK\nINSPVAX / NavSatFix]
    end

    subgraph 里程计
        F[fast_lio_msf\n双雷达+IMU 紧耦合]
    end

    subgraph 全局定位
        M[mapLocalization\nGICP 匹配先验 PCD 地图]
        W[waypointTransform\n航点坐标转换]
    end

    subgraph CMU 局部栈（odom 系）
        LI[loam_interface]
        TA[terrain_analysis /\nterrain_analysis_ext]
        LP[local_planner]
        TP[tare_planner\n自主探索]
    end

    L1 & L2 --> F
    R -.初始位姿/退化兜底.-> M
    F -- /Odometry, /cloud_registered --> M
    F -- /Odometry, /cloud_registered --> LI
    M -- TF: prior_map→map, ~1Hz --> W
    W -- /way_point (odom系) --> LP
    LI --> TA --> LP
    TA --> TP --> LP
```

核心思路：**里程计（fast_lio_msf）与局部规划全程在 `map`（即 odom）坐标系内工作，不会感知到全局修正带来的跳变**；
`map_localization` 只在 `prior_map → map` 这条 TF 上以 ~1Hz 的频率做漂移修正，全局目标点在 `prior_map` 系下发布，
经 `waypointTransform` 持续转换为 `map` 系后再交给局部规划。

## 目录结构

| 目录 | 作用 |
|---|---|
| `map_localization` | **先验地图定位**：GICP 扫描匹配 + RTK 全局初始化/退化兜底 + 航点坐标转换（详见下文） |
| `FAST-LIO-Multi-Sensor-Fusion` (`fast_lio_msf`) | 里程计：双 MID360 + IMU 紧耦合 LIO，可选融合 GNSS / 轮速 |
| `loam_interface` | 把里程计输出适配为 CMU 局部栈使用的 `/state_estimation`、`/registered_scan` 及 TF |
| `livox_ros_driver2` / `Livox-SDK` / `Livox-SDK2` * | Livox 雷达驱动 SDK |
| `bynav_ros_driver(CJ)(1)` * | bynav RTK/GNSS 接收机驱动（含 `novatel_oem7_msgs`），建议改名去掉括号避免 catkin 路径问题 |
| `sensor_scan_generation` | 生成局部规划所需的点云扫描格式 |
| `terrain_analysis` / `terrain_analysis_ext` | 地形可通行性分析（含扩展版连通性检查） |
| `local_planner` | 局部避障与路径跟随，输出 `/cmd_vel` |
| `tare_planner` | 全局自主探索规划（TARE） |
| `vehicle_simulator` / `velodyne_simulator` | 仿真环境与传感器仿真 |
| `visualization_tools` / `waypoint_rviz_plugin` / `waypoint_example` | RViz 可视化与航点交互工具 |
| `joystick_drivers` | 手柄遥控驱动 |

`*` 标记的目录是第三方雷达 SDK / 驱动，本仓库 `.gitignore` 已将其排除，不随本仓库提交。
需要按对应厂商文档单独获取并放到 `src/` 下相同目录名（`Livox-SDK`、`Livox-SDK2`、`livox_ros_driver2`、`bynav_ros_driver(CJ)(1)`）后再编译。

## 定位框架详解

### 坐标系与职责划分

- `prior_map`：建图当天保存的先验点云地图坐标系（ENU 对齐，原点为建图时 RTK 记录的经纬度）。全局目标点在此系下发布。
- `map`（即 odom 系）：`fast_lio_msf` 里程计原点，全程连续、无跳变；CMU 局部栈（`loam_interface`、`terrain_analysis`、`local_planner`）只认这个系。
- `prior_map → map` 的 TF 由 `mapLocalization` 节点以 ~1Hz 广播，是唯一会"跳变"的修正量，局部栈不订阅它。

### 节点组成（`map_localization` 包）

| 节点 | 输入 | 输出 | 作用 |
|---|---|---|---|
| `mapLocalization` | `/Odometry`、`/cloud_registered`（来自 fast_lio_msf）、`/bynav/inspvax` 或 `/gps/fix` | TF `prior_map→map`、`/localization/pose`、`/localization/status`、`/localization/fitness` | 累积近几秒配准点云，用 GICP 与体素滤波后的先验地图匹配；RTK 提供全局初始位姿，扫描匹配退化时降级为仅平移的 RTK 修正 |
| `waypointTransform` | `/way_point_prior`（`prior_map` 系） | `/way_point`（`map` 系） | 把上层下发的全局目标持续转换到 odom 系供 `local_planner` 使用 |

`mapLocalization` 的状态机（`/localization/status`）：

| 值 | 状态 | 含义 |
|---|---|---|
| 0 | INIT | 等待 RTK + 足够扫描点完成全局初始化 |
| 1 | TRACK_MAP | 正常匹配先验地图 |
| 2 | TRACK_RTK | 匹配退化（如开阔地），降级为 RTK 平移修正 |
| 3 | LOST | 无 RTK 兜底，保持上一次修正不变 |

`/localization/fitness` 为 GICP 平均平方残差，越小说明匹配越好。

### 初始化行为

- bynav 输出 `INSPVAX`（含航向）：机器人静止即可完成全局初始化。
- 仅有 `NavSatFix`（无航向）：需要直线行走约 `min_travel`（默认 3 m），用运动轨迹与 RTK 位移对齐估出航向。
- 无 RTK 信号或先验地图原点文件缺失：无法自动初始化，节点会持续告警。

### 使用流程

**1）一次性标定**（填入 `fast_lio_msf/config/mid360*.yaml` 与 `map_localization.launch`）：

| 外参 | 位置 | 说明 |
|---|---|---|
| 云台雷达 → 身体雷达 IMU | `mid360*.yaml` 的 `right_lidar_to_imu` | 两台 MID360 之间的刚体变换 |
| RTK 天线杆臂 | `mid360*.yaml` 的 `extrinT_Gnss2IMU`、`map_localization.launch` 的 `antenna_offset` | 天线相对身体雷达 IMU 的位置偏移 |
| 雷达 IP / 话题 | `livox_ros_driver2/config/MID360_dual_quadruped.json`、`mid360*.yaml` | `left`=身体水平雷达（IMU 来源），`right`=云台雷达 |

两台 MID360 需通过 gPTP 与主机时间同步；bynav 建议配置输出 `INSPVAX`。

**2）建图（一次性）：**

```bash
roslaunch map_localization system_quadruped_mapping.launch
# 遥控机器人走完场地（RTK 尽量保持固定解），Ctrl-C 结束
# 产出：fast_lio_msf/PCD/scans.pcd（先验地图）+ PCD/map_origin.yaml（ENU 原点）
```

**3）日常作业：**

```bash
roslaunch map_localization system_quadruped_localization.launch

# 下发全局目标（prior_map 系，单位 m）
rostopic pub /way_point_prior geometry_msgs/PointStamped \
  '{header: {frame_id: prior_map}, point: {x: 120.5, y: 88.3, z: 0}}' -1
```

## 构建

```bash
cd <workspace>
catkin_make -DROS_EDITION=ROS1    # livox_ros_driver2 需要该宏
```

依赖：PCL、Eigen、GeographicLib（`fast_lio_msf`），bynav 驱动依赖的 `novatel_oem7_msgs`（仓库内已含）。

## 致谢

本仓库基于 [CMU autonomous_exploration_development_environment](https://www.cmu-exploration.com) 二次开发，
里程计部分基于 [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) 与 [FAST-LIO-Multi-Sensor-Fusion](https://github.com/kahowang/FAST_LIO_SAM)，感谢原作者的开源工作。
