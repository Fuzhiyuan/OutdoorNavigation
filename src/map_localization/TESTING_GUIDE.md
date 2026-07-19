# 四足机器人定位系统测试手册（小白版）

> 本手册面向第一次接触本系统的测试人员。按顺序做，**每个阶段全部打勾后才能进入下一阶段**。
> 出现"不通过"时，按每节末尾的《常见问题》排查；解决不了就把录好的 bag 包和现象发给负责人。

---

## 0. 你需要先知道的事

**这套系统是什么：**
狗身上装了 2 个激光雷达（头部 1 个、背部 1 个）和 1 个 RTK 高精度定位天线。
软件分四层，一层出错会连累上面所有层，所以必须一层一层测：

```
第1层  传感器驱动     （雷达出点云、RTK 出位置）
第2层  激光里程计     （FAST-LIO：狗知道"自己刚才怎么动的"）
第3层  地图定位       （拿点云和事先建好的地图比对，知道"自己在场地哪里"）
第4层  自主导航       （给一个目标点，狗自己走过去）
```

**基本操作约定：**
- 所有命令都在 Ubuntu 的终端里输入。打开终端：`Ctrl + Alt + T`。
- 每开一个新终端，先执行一次（否则会提示"找不到命令"）：
  ```bash
  cd ~/你的工作空间目录        # 例如 cd ~/dog_ws
  source devel/setup.bash
  ```
- 看到 `roslaunch ...` 的命令 = 启动一个程序，会一直运行，**别关这个终端**；要停止按 `Ctrl + C`。
- "录 bag" = 把传感器数据录下来存成文件，之后可以像放录像一样回放。**每个阶段都要录**，这是出问题后排查的唯一依据：
  ```bash
  rosbag record -a -o 阶段名_日期    # 例: rosbag record -a -o stage2_0719
  ```

---

## 阶段 0：编译检查（在电脑上，不用狗）

**目的：确认代码能编译、启动文件没有写错。**

```bash
cd ~/你的工作空间目录
catkin_make -DROS_EDITION=ROS1
```

- [ ] 编译最后显示 `[100%]`，没有红色 `Error`（黄色 warning 可以忽略）

```bash
roslaunch --check map_localization/launch/system_quadruped_mapping.launch
roslaunch --check map_localization/launch/system_quadruped_localization.launch
```

- [ ] 两条命令都没有报错

---

## 阶段 1：传感器检查（狗上电，放在原地不动）

**目的：确认两个雷达和 RTK 的数据都是好的。这是最重要的阶段，别跳步。**

### 1.1 雷达出数检查

新开终端，分别执行（IP 后缀按实际配置，见 `mid360.yaml`）：

```bash
rostopic hz /livox/lidar_192_168_1_12    # 背部雷达点云
rostopic hz /livox/lidar_192_168_1_3     # 头部雷达点云
rostopic hz /livox/imu_192_168_1_12      # 背部雷达 IMU
```

- [ ] 两个 lidar 话题都显示 `average rate: 10.0` 左右
- [ ] imu 话题显示 `average rate: 200` 左右
- 没有输出？→ 见本节《常见问题》

### 1.2 两个雷达的时间是否同步

```bash
rostopic echo /livox/lidar_192_168_1_12/header/stamp &
rostopic echo /livox/lidar_192_168_1_3/header/stamp
```

对比两边打印的时间戳（secs.nsecs）：

- [ ] 两台雷达时间戳差 **小于 0.01 秒**，且不会越差越大

### 1.3 双雷达外参目测（关键！）

**目的：确认"头部雷达装歪 90 度"这件事在配置里写对了。**

启动里程计和 rviz：

```bash
roslaunch fast_lio_msf mapping_mid360.launch
```

在 rviz 里看拼出来的点云（话题 `/cloud_registered`），拿狗对着一面平整的墙：

- [ ] 地面是**一个平面**，不是两层错开的面
- [ ] 墙面是**一片点**，不是两片错开/成角度的点
- [ ] 狗前方的地面能被看到（这是头部雷达贡献的）

**如果墙出现两层：** 头部雷达外参错了。打开
`src/FAST-LIO-Multi-Sensor-Fusion/config/mid360.yaml`，
找 `right_lidar_to_imu`，通知负责人调整（多半是 90 度的方向反了或平移量没实测）。

### 1.4 RTK 检查（需要到室外空旷处）

```bash
rostopic echo /gps/fix
```

- [ ] `status: 2`（或至少不是 -1），`latitude/longitude` 是本地的经纬度
- [ ] `position_covariance` 前几个数字**不是全 0**（RTK 固定解时通常 < 0.001）

```bash
rostopic echo /bynav/inspvax
```

- [ ] 有持续输出
- [ ] `azimuth`（方位角，正北=0、正东=90，顺时针）和狗实际朝向一致（拿手机指南针对比，误差 < 5 度）

### 阶段 1 常见问题

| 现象 | 原因和处理 |
|---|---|
| 雷达话题没数据 | 检查网线；`ping 192.168.1.12` 是否通；主机 IP 是否设成了 json 里的 host_ip（默认 192.168.1.5） |
| 只有一台雷达有数据 | json 里 `lidar_configs` 的 IP 和实际不符（MID360 出厂 IP = 192.168.1.1XX，XX 是序列号后两位） |
| 时间戳差持续变大 | gPTP 时间同步没生效，找负责人配置交换机/主机的 PTP |
| covariance 全是 0 | RTK 驱动没输出精度信息，此时**定位模块会拒绝使用 RTK**，必须解决 |
| azimuth 差 180 度 | 双天线接反了，或安装方向配置错误 |

---

## 阶段 2：激光里程计测试（遥控狗走，不用 RTK）

**目的：确认狗"知道自己怎么动的"，且长距离不飘。**

启动：终端 1 录 bag，终端 2 跑 `roslaunch fast_lio_msf mapping_mid360.launch`。

按顺序做四个动作，全程在 rviz 里观察：

| # | 动作 | 通过标准 |
|---|---|---|
| 1 | 狗站着不动 5 分钟 | rviz 里位置漂移 < 5cm（几乎不动） |
| 2 | 遥控走一个大圈（50~100 米）回到出发点 | rviz 轨迹终点和起点差 < 0.5 米；墙面点云干净不重影 |
| 3 | 原地快速转身 2~3 圈 | 转完后场景没有"错位"、点云没糊 |
| 4 | 小跑一段 + 上下台阶（如有） | 位置不跳变、不发散 |

- [ ] 4 项全部通过

**不通过怎么办：** 动作 2 墙面重影 → 回阶段 1.3 查外参；动作 3/4 发散 →
把 bag 交给负责人（可能需要调 IMU 噪声参数 `acc_cov/gyr_cov`）。

---

## 阶段 3：建图（一次性工作，选天气好、RTK 信号好的时候）

**目的：给场地建一张带真实经纬度坐标的点云地图。以后每天定位都靠它。**

1. 把狗带到场地里**天空开阔**的位置（建图起点要 RTK 固定解）。
2. 启动（同时开一个终端录 bag）：
   ```bash
   roslaunch map_localization system_quadruped_mapping.launch
   ```
3. 确认终端里出现了 `Map ENU origin saved to ...` 字样（表示 RTK 原点记录成功）。
4. 遥控狗**慢速（<0.5m/s）**走遍整个工作区域。要点：
   - 沿建筑物/固定物体走，多绕圈，重复走过的地方拼图更牢；
   - 尽量避开大量行人/车辆（动态物体会留“鬼影”）。
5. 走完回到起点附近，在启动终端按 `Ctrl + C` 结束。**耐心等它保存完**。
6. 检查产物（都在 `src/FAST-LIO-Multi-Sensor-Fusion/PCD/` 目录）：
   - [ ] `scans.pcd` 存在且不是空文件（通常几百 MB）
   - [ ] `map_origin.yaml` 存在，里面有 origin_latitude / longitude / altitude 三行
7. 验证地图质量：`pcl_viewer scans.pcd`（或用 CloudCompare 打开）：
   - [ ] 墙是直的、竖的，没有一面墙变两面的重影
8. **验证地图和 RTK 对得上（关键）**：选场地里 2~3 个空旷位置，狗分别走过去站定，
   记录此刻终端里 `/Odometry` 的 x/y 值和 `/gps/fix` 的经纬度，交给负责人换算比对：
   - [ ] 每个点误差 < 0.2 米

---

## 阶段 4：地图定位测试（先回放，再实测）

**目的：确认狗开机后能自动知道"我在地图的哪里"，并且走动中不丢。**

### 4.1 用回放测（不用狗）

```bash
# 终端1
roslaunch map_localization map_localization.launch
# 终端2：回放阶段3录的 bag
rosbag play 你的建图bag.bag
```

观察话题（新开终端 `rostopic echo /localization/status` 和 `/localization/fitness`）：

- [ ] status 从 0（初始化中）变成 1（地图匹配正常）
- [ ] fitness 稳定后 **< 0.1**（把这个数记下来，它是以后判断健康的基准）

**status 数字含义：** 0=正在初始化 1=地图匹配正常 2=靠RTK兜底 3=暂时失去修正（保持上次结果）

### 4.2 实测初始化

把狗带到场地里**任意位置**（不要是建图起点），启动：

```bash
roslaunch map_localization system_quadruped_localization.launch
```

- [ ] 狗静止状态下，1 分钟内 status 变为 1（RTK 有航向时静止即可初始化）
- [ ] 遥控走动，status 长期保持 1；经过空旷处允许短暂变 2，回到建筑附近能回 1

---

## 阶段 5：自主导航测试（人必须全程跟着，随时准备接管遥控）

**目的：最终验收——发目标点，狗自己走过去。**

**安全要求：场地开阔、无人，测试员拿着遥控器跟随，随时可急停。**

1. 先把速度降下来：
   ```bash
   roslaunch map_localization system_quadruped_localization.launch maxSpeed:=0.3 autonomySpeed:=0.3
   ```
2. **小步测试**：发一个 2 米外的目标点（x/y 是地图坐标，单位米）：
   ```bash
   rostopic pub /way_point_prior geometry_msgs/PointStamped \
     '{header: {frame_id: prior_map}, point: {x: 2.0, y: 0.0, z: 0}}' -1
   ```
   - [ ] 狗朝**正确方向**走并停在目标附近（方向不对立刻遥控接管，报告负责人）
3. - [ ] 发一个 20 米以上、需要绕障的目标点，能自主绕开障碍到达
4. - [ ] **重复精度**：同一个目标点，从不同方向让狗走过去 10 次，10 次停点互相差 < 0.3 米
5. - [ ] 恢复正常速度（不带 maxSpeed 参数，默认 0.8m/s）重复第 3 步
6. - [ ] **长时测试**：连续多点巡检 30 分钟以上，全程 status 无长时间 3、fitness 无持续恶化

全部打勾 = 验收通过。🎉

### 阶段 5 常见问题

| 现象 | 排查方向 |
|---|---|
| 狗走的方向完全不对 | 初始化时航向错了：查阶段 1.4 的 azimuth；或地图 ENU 对齐差（阶段 3 第 8 步） |
| 走着走着突然"抽一下" | 不应该发生（设计上修正不影响局部规划），录 bag 上报 |
| 到不了目标点附近就停 | 目标点是否在地图范围外/障碍物里；看 rviz 里 /way_point 的位置对不对 |
| 开阔地走偏 | fitness 恶化 + RTK 也不好，属于双盲区，绕开或加装辅助 |

---

## 附录：每次测试要留档的东西

1. 全程 rosbag（命名：`阶段_日期_序号.bag`）；
2. 阶段 4/5 的 `/localization/fitness`、`/localization/status` 数值范围；
3. 不通过项的现象描述 + 当时的终端截图。
