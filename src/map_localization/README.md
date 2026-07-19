# map_localization — 四足双 MID360 + bynav RTK 先验地图定位

架构：

```
bynav RTK (INSPVAX / NavSatFix)          双 MID360 (livox_ros_driver2, multi_topic=1)
        │  初始位姿 / 退化兜底                         │
        ▼                                            ▼
  mapLocalization  ◄── /Odometry, /cloud_registered ── fast_lio_msf（双雷达合并，odom 系）
        │ GICP 匹配先验 PCD 地图                        │
        ▼                                            ▼
  TF: prior_map → map（~1Hz 修正）        loam_interface → CMU 局部栈（terrain_analysis /
        │                                             local_planner，全程 odom 系，无跳变）
        ▼
  waypointTransform: /way_point_prior (prior_map 系) → /way_point (odom 系)
```

## 使用流程

**第一步（一次性）：实测三组外参并填入配置**

| 量 | 位置 | 说明 |
|---|---|---|
| 狗头雷达 → 身体雷达 IMU | `fast_lio_msf/config/mid360*.yaml` 的 `right_lidar_to_imu` | 旋转已按"z 朝前、x 朝地"填为绕 y 轴 +90°，平移 `[0.30, 0, 0.10]` 是占位值 |
| RTK 天线杆臂 | 同上 `extrinT_Gnss2IMU` 及 `map_localization.launch` 的 `antenna_offset` | 天线相对身体雷达 IMU 的位置 |
| 雷达 IP / 话题名 | `livox_ros_driver2/config/MID360_dual_quadruped.json` 与 `mid360*.yaml` 的 `lid_topic_left/right`、`imu_topic` | left = 身体水平雷达（IMU 来源），right = 狗头雷达 |

两台 MID360 需通过 gPTP 与主机时间同步；bynav 需配置输出 `INSPVAX`（含航向，最优）或至少 `NavSatFix`。

**第二步：建图日（跑一次）**

```bash
roslaunch map_localization system_quadruped_mapping.launch
# 遥控狗走完场地（RTK 尽量保持固定解），Ctrl-C 结束
# 产出: fast_lio_msf/PCD/scans.pcd（ENU 对齐地图）+ PCD/map_origin.yaml（ENU 原点）
```

**第三步：日常作业**

```bash
roslaunch map_localization system_quadruped_localization.launch
# 下发全局目标（prior_map 系 = 建图时的 ENU 系，单位 m）:
rostopic pub /way_point_prior geometry_msgs/PointStamped \
  '{header: {frame_id: prior_map}, point: {x: 120.5, y: 88.3, z: 0}}' -1
```

监控话题：`/localization/status`（0=初始化中 1=地图匹配 2=RTK 兜底 3=保持上次修正）、`/localization/fitness`（GICP 平均平方残差，越小越好）。

## 初始化行为

- 有 INSPVAX（含方位角）：静止即可完成全局初始化。
- 只有 NavSatFix：需要直线走约 `min_travel`（默认 3 m）用轨迹对齐出航向。
- 无 RTK / 原点文件缺失：无法自动初始化（会持续告警）。

## 构建

```bash
cd <workspace>
catkin_make -DROS_EDITION=ROS1    # livox_ros_driver2 需要该宏
```

依赖：PCL、Eigen、GeographicLib（fast_lio_msf），bynav 驱动的 `novatel_oem7_msgs`（仓库内已含）。
建议把 `src/bynav_ros_driver(CJ)(1)/...` 重命名为不含括号的路径（如 `src/bynav_ros_driver`），
并删除其内层遗留的 `build/`、`devel/` 目录，避免干扰 catkin。
