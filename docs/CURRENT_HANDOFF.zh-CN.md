# 当前阶段交接（先读此文件）

更新时间：2026-08-31

## 一句话状态

场地、1.2m OFFBOARD 飞行、固定地图 A*、双目视觉绕桩、双通道投放均已实现。2026-08-31 复审修掉两个会直接判负的避障缺陷，新增不依赖 VM 的闭环避障回归测试（24 种抽签布局零碰撞），并在 VM 实跑中定位修复三个环境级根因：Gazebo 物理输出 NaN、偏航控制器增益被调到默认值 1/9、`EKF2_EVA_NOISE` 从未设置导致转向时航向估计跟不上进而位置发散。完整任务链「危险品区 → 人员救助区 → 自主返航」已跑通，最快 `115.5s`（180s 预算内），最近圆柱 `0.436m`（碰撞阈值 0.30m）。此前基于 EKF 参数的诊断结论已失效。当前不是验收完成版：**10 轮批量复现率仅 20%**，瓶颈是估计器抖动，避障逻辑从未独立失败。

## 2026-08-31 避障复审与修复

本轮以竞赛规则（附件 4-1）为基准全面复审避障链路，按 TDD 修复。全部本地测试 `234/234` 通过（VM 同样 `234/234`）。

### 修复的缺陷

1. **合法抽签下航线规划直接失败（会判负）**
   人员救助区 2 距随机圆柱 2 摆位 2 仅 `0.602m`，小于巡航膨胀 `0.45m` + 动态圆半径 `0.20m`，`plan_route` 抛 `goal_blocked`，任务当场终止。该组合概率 1/6。
   修法：`plan_route` 新增目标松弛，所需间隙在最后 `0.60m` 内从 `0.45` 线性收敛到 `0.35`（`GOAL_INFLATION`/`GOAL_RELIEF_RADIUS`）。`0.35` 仍把整个机架挡在障碍外，只让出巡航余量，且只在规则强制悬停处让出。
   证据：24 种抽签组合逐一验证可规划（`test_every_drawn_field_layout_plans_the_full_mission`）。

2. **偏航速率使赛程在算术上不可能完成**
   `maximum_yaw_rate` 为 `0.08rad/s`，单次 90° 转向约 `19.6s`。按真实场地图统计，整条任务航线累计转向 `863°–1013°`，仅偏航即需 `188–221s`，而平移只需 `65–84s`，总计 `258–305s`，超出 3 分钟赛程。
   修法：`maximum_yaw_rate` `0.08 → 0.35`，`maximum_turning_speed` `0.08 → 0.12`。
   **平移速度保持 `0.18m/s` 不变**——`0.25m/s` 曾导致撞机，且提高偏航速率后已无需提速。

3. **已完成动态规划的圆柱被近距避障重复识别（约 10s 停车）**
   `FOLLOW_ROUTE` 的近距触发不查记忆，同一圆柱重跑 `BRAKE/OBSERVE/SELECT_SIDE` 与转向。
   修法：新增 `_local_brake_required`。已进入当前航线的圆柱不再触发局部停车；`emergency_range=0.35m` 以内无条件制动，保留定位误差下的兜底。

4. **换任务段时丢失圆柱记忆（本轮新发现，且会被第 3 条放大）**
   `set_goal` 用不带动态圆的固定地图重新规划，第二、三段航线对已识别圆柱失忆；而 `_matches_remembered_obstacle` 又会抑制远距重规划，近距制动也被第 3 条抑制 → 直接撞桩。
   修法：`set_goal` 改走 `_plan_with_memory`，带 `temporary_obstacles` 规划，规划失败再退回固定地图。这条同时是第 3 条成立的前提。

5. **圆柱圆模型不一致**
   远距重规划用「测量面沿视线外推一个圆柱半径」，近距选边却把测量面当圆心、用可见弦宽当半径，两者可差约一个半径，导致记忆匹配不可靠。已统一到 `_dynamic_obstacle_circle`，删除 `_obstacle_circle`。

6. **动态障碍记忆无上限**
   规则只放 2 个随机圆柱，误检会不断堆积圆并压缩自由空间直至 `route_unreachable`。新增 `maximum_dynamic_obstacles=4`，超出丢弃最旧。

### 新增测试

`test/test_avoidance_closed_loop.py`：真实规划器 + 合成双目 + 一阶滞后机体模型的闭环仿真，**不需要 ROS/Gazebo/VM**，约 11 秒跑完。

- 24 种抽签布局全部到达三个目标，零碰撞。
- 最近圆柱圆心 `0.611m`（碰撞阈值 `0.30m`），最近挡板面 `0.491m`（阈值 `0.20m`）。
- 最差任务用时 `97.9s`，3 分钟赛程剩约 `82s` 给悬停、识别、投放、降落。
- 双目测距偏差 `±0.10m` 内仍全过。

### 已量化的边界（给感知同学）

双目测距偏差必须控制在 **±0.10m** 以内：

- 偏差 `+0.20m`（测远了）：24 布局中 10 例撞桩，最近圆心 `0.044m`。
- 偏差 `−0.20m`（测近了）：20 例过度保守卡死，不撞但跑不完。

已验证放大 `dynamic_localization_margin` **不能**换来鲁棒性：调到 `0.15` 时即便测距完美也有 6 例卡死，因为阻塞圆超过 1.3m 保证通道的半宽。该值保持 `0.10`。

### 本轮未改动（保留原判断）

- `known_static_tolerance` 保持 `0.18`。圆柱摆位到挡板面最近仅 `0.400m`，测量面更近到约 `0.30m`，调大到 `0.25` 只剩 `0.05m` 余量，有把真圆柱误判为已知静态物的风险。挡板误检更应该用「测量宽度」区分，需要真实深度数据，本轮不做。
- `maximum_setpoint_lead` 由 `0.08` 提到 `0.25`。原值小于文档记录的 `0.22–0.43m` 定位误差，必然长期触发冻结，造成设定点走走停停。冻结机制本身（超限直接锁死 `last`）仍未改，因为改动会推翻 `test_setpoint_lead_limit_never_drags_route_toward_pose_drift` 所固定的契约，且无法在本地验证。建议后续改成只限制沿航向的超前量。

### 下一步（必须在 VM 里做）

1. `catkin_make` + 全量测试，确认 234 项在 VM 同样通过。
2. 种子 1 单次 SITL 跑通，重点看新的偏航速率 `0.35rad/s` 是否稳定。已在 VM 验证：见下一节 `2026-08-31 SITL 环境两个根因修复`，瓶颈是 `MC_YAW_P` 而非 EKF，不要退回 `0.08`（退回等于放弃赛程）。
3. 种子 1 连续三次通过后，跑 `1、4、10、2` 随机圆柱矩阵。
4. 矩阵通过后再接危险品识别、投放、人员救助、返航完整链。

## 2026-08-31 SITL 环境两个根因修复

在 VM 里实跑时发现两个环境级缺陷。它们同时使此前所有 EKF 调参结论失效，**下面 `2026-08-30 外部视觉跟踪诊断` 一节的结论请勿再作为依据**（保留原文仅供追溯）。

### 1. `fire_iris.sdf` 让 Gazebo 物理输出 NaN

未提交改动把 4 个旋翼关节的 `use_parent_model_frame` 从 `1` 改成 `0`。结果：

- `iris_0` 全部 link 位置恒为 `(0,0,0)`、速度为 `nan`（同一世界里的静态模型 twist 正常为 `0`）
- `/mavros/imu/data` **无任何消息**，PX4 收不到传感器数据
- EKF2 每 40ms 复位一次（`reset position to ev position`），`/mavros/local_position` 的 `z` 恒为 `nan`
- 规划器正确判为 `invalid_pose` → `HOLD_UNSAFE`，飞机从不解锁

改回 `1` 后立即正常起飞并悬停 `1.2m`，MAVROS 与 Gazebo 水平误差约 `0.005m`。

推论：此前记录的「EKF 输出跟踪误差 `0.22–0.43m`」「起飞转向撞北侧安全网」极可能都源于此，而非 `EKF2_TAU_POS`/`EKF2_TAU_VEL`。

### 2. `MC_YAW_P` 被调到默认值的 1/9

`config/px4/10016_iris.post` 中 `MC_YAW_P 0.3`（PX4 v1.11 默认 `2.8`）、`MC_YAWRATE_I 0.01`（默认 `0.1`）。

P 增益 `0.3` 时，偏航速率设定点 `0.35rad/s` 对应稳态航向误差约 `0.35/0.3 ≈ 1.17rad`，姿态控制器无法跟踪，推力矢量指向错误，最终 `Critical navigation failure! ` → `Failsafe enabled: no local position` → `AUTO.LAND`。

VM 实测 A/B（同场地同代码，运行中改 `/path_planner/maximum_yaw_rate`）：

| 偏航速率 | `MC_YAW_P 0.3` 下的结果 |
|---|---|
| `0.08rad/s` | 完整飞完第一段到 `REACHED`，水平误差 `0.001–0.010m` |
| `0.20rad/s` | 约 9 秒后 `AUTO.LAND` |
| `0.35rad/s` | 约 24 秒后 `Critical navigation failure` |

恢复 `MC_YAW_P 2.8`、`MC_YAWRATE_I 0.1` 后，在 `0.35rad/s` 下稳定悬停 `150` 秒无失效保护，误差 `0.005–0.006m`。

结论：偏航速率上限不是 EKF 决定的，是姿态控制器增益决定的。`maximum_yaw_rate 0.35` 可用，前提是本次增益修复同时生效。`MC_YAWRATE_MAX` 保留 `45`（安全上限，非瓶颈）。

### 3. 顺带修复与已知坑

- `world_generator.generate_world` 不创建父目录，任何未生成过的 seed 都会以 `IOError` 打死 `firefighting_sitl`，进而拖垮整个 launch。已修。
- `start_sitl.sh` 在仓库 `.post` 与已安装文件不一致时 `exit 2`。改 `.post` 后必须先删除 `PX4_Firmware/ROMFS/px4fmu_common/init.d-posix/10016_iris.post` 再启动。本次改动前的旧文件备份在 VM 上 `/home/ss/10016_iris.post.bak-before-yawgain`。
- ~~非正常降落后规划器可能永久卡死：落点落在固定挡板 `0.45m` 膨胀区内会抛 `start_blocked`。~~ **已修复**，见下文起点松弛（`START_INFLATION`）。危险品任务区距挡板 1 恰好 `0.45m`，规则要求在其上方悬停，悬停漂移几厘米即会触发，实测已在 VM 咬人一次。
- Python 2 与 Python 3 的 `random.choice` 实现不同，同一 seed 在 VM（Py2）与开发机（Py3）得到不同布局。避障测试按摆位穷举，不依赖 seed，因此不受影响；但用 seed 复现场景时必须以 VM 的 Py2 结果为准。VM 上 `seed 1` = `cyl(1,2) hazard2 person2`。

### 4. 定位发散根因：`EKF2_EVA_NOISE` 从未设置

用 ULog 逐帧比对后定位。`EKF2_EV_NOISE_MD 1` 表示观测噪声**取自参数**，视觉桥消息里的协方差完全不生效。而 `.post` 设了 `EKF2_EVP_NOISE`、`EKF2_EVV_NOISE`，**唯独没设 `EKF2_EVA_NOISE`**，于是取 PX4 默认 `0.05rad`。

收敛后航向方差约 `1e-4`，`R = 0.05² = 2.5e-3`，卡尔曼增益 ≈ `P/(P+R) ≈ 4%`，时间常数约 `0.5s`。`0.35rad/s` 的航线转向直接跑赢了它。

ULog 实测（seed 1，`13_08_44.ulg`）：

| 时刻 | 真值航向 | 估计航向 | 航向误差 | 航向测试比 | EV 位置测试比 |
|---|---|---|---|---|---|
| t+60.91 | 1.646 | 1.636 | −0.011 | 0.001 | 0.008 |
| t+61.26 | 1.527 | 1.588 | **0.061** | 0.003 | 0.017 |
| t+62.65 | 1.475 | 1.690 | 0.215 | 0.065 | 0.408 |
| t+63.34 | 1.501 | 1.760 | 0.259 | 0.107 | **3.387** |
| t+64.04 | 1.582 | 1.897 | 0.315 | 0.162 | **22.0** |
| t+65.78 | 1.578 | 1.698 | 0.121 | 0.017 | **50.9** |

航向测试比全程 `< 1`，**从未被拒**，纯粹是纠正不过来。错误航向再把位置预测一转，EV 位置新息冲破 `EKF2_EVP_GATE` 被拒，位置从此靠 IMU 纯积分，估计值最终跑到离真值 `44m`，飞控按幻觉修正把真机推向北侧安全网。

另外确认：融合状态位全程 `ev_pos=1 ev_yaw=1 yaw_align=1`，且 PX4 收到的视觉航向与真值差 `0.000–0.005rad`，**输入数据没问题，管线没污染**。

改为 `EKF2_EVA_NOISE 0.01`（参数文档下限，对 Gazebo 真值是诚实取值），增益升到约 50%。同场地实测：

| | 估计误差超 `0.25m` 的时刻 | 峰值误差 | 真值最北 |
|---|---|---|---|
| 修复前 | `27s` | `55m` | `y=0.59`（网在 0.65）|
| 修复后 | `132s` | — | `y=0.01` |

### 5. 已验证的两段避障

用 `competition_takeoff.launch`（无 `mission_manager`，目标由脚本直接发到 `/fire_mission/point_goal`，绕开尚未完成的识别环节）：

```
hazard zone  REACHED in 40.3 s  closest cylinder 0.635 m  lowest 1.25 m
rescue zone  REACHED in 78.2 s  closest cylinder 0.588 m  lowest 1.10 m
```

碰撞阈值 `0.30m`。救助区正是 seed 1 抽签下距圆柱仅 `0.602m` 的组合，即修复前 `plan_route` 直接 `goal_blocked` 的那个。

### 6. 失败的实验（勿重复）

- `EKF2_AID_MASK 280 → 24`（关掉视觉速度融合）：仍发散，峰值 `55m`。已回滚。交接文档早前记录的「改 24 更差」实验是在物理输出 NaN 的坏模型下做的，结论本身也不成立。
- `EKF2_EVP_GATE 10 → 30`（放宽位置门限，避免被拒）：**更差**。发散中的巨大新息被照单全收会搞垮滤波器。两段航线在 `10` 下都能飞，在 `30` 下一段都飞不完。已回滚。

### 7. 完整任务链已跑通，但只有 20% 复现率

首次完整跑通「危险品区 → 人员救助区 → 自主返航」：

```
run=3  hazard=41.7  rescue=28.7  home=45.1  total=115.5  closest=0.436  within_180=yes
run=6  hazard=41.5  rescue=22.1  home=90.4  total=154.0  closest=0.590  within_180=yes
```

10 轮批量统计（`tools/run_avoidance_campaign.sh`，原始记录 `docs/campaign-2026-09-01.txt`）：

| 结果 | 次数 |
|---|---|
| 完整通过 | **2** |
| 卡在危险品区 | 5 |
| 卡在救助区 | 1 |
| 卡在返航 | 1 |
| 驱动脚本失败 | 1 |

**决定性相关**：

| 该轮估计器状态 | 轮数 | 完整通过 |
|---|---|---|
| 干净（`blind_land`、`lockdown`、`no_offboard`、`ev_resets` 全为 0）| 3 | **2** |
| 有任何一项非 0 | 7 | **0** |

**避障逻辑从未独立失败过。** 每一次失败之前都先出现估计器或控制器故障；只要估计器全程干净，任务就在 180 秒内完成。所以「稳定复现」这件事，剩下的全部工作量在消除估计器抖动，不在避障。

时间上不紧张：两次通过是 `115.5s` 和 `154.0s`，任务段本身只要 `40s + 25s` 左右，返航 `45–90s`。

### 8. 录像运行（2026-09-01 凌晨）

手动分段发目标录了一次：**危险品区到达、人员救助区到达**，返航段卡在 `geofence_recovery` 未完成，随后 `gzserver` 崩溃（`/clock` 停发）。

值得记录的是这一轮 `blind_land = 0`、`ev_resets = 0` —— **估计器全程干净，返航段仍然卡住**。这与上一节「干净即通过」的相关性不完全一致，说明 `geofence_recovery` 除了已修的 5cm 释放条件外，还有第二个成因未查清。下一轮排查应优先看这个。

另外两条操作经验：

- VM 连续跑批量后负载会累积（实测 `load average 12.0`、可用内存降到 `167MB`），此时 `gzserver` 崩溃概率明显上升。录像或验收前先 `tools/sim_teardown.sh` 并等负载回落。
- 通过 SSH 直接跑驱动脚本时，会话中断会带走脚本且已发目标不生效（latch 发布者随进程消失）。必须 `setsid` 脱离，或用 `rostopic pub -1` 逐段发目标。

### 9. 2026-09-01：因果关系被推翻，飞机是先撞网

上一节「估计器干净就通过」的相关性**看反了因果**。用物理监视器（`/gazebo/model_states` 逐帧记录）加 ULog 对照，完整链条是：

```
t+96 ~ 101   一切干净：航向误差 ≤0.056，ev_hpos_ratio ≤0.016，位置误差 ≤0.067m
t+101.84     ev_hpos_ratio 0.752
t+102.18     ev_hpos_ratio 5.546        ← EV 位置开始被拒
t+102.85     真值航向 -1.304 / 估计 -0.597
t+103.52     真值航向 -1.540 / 估计 -0.440   航向误差 1.100 rad（63°）
```

同一时刻的 Gazebo 真值位置是 `(1.82, 0.53)`。北侧安全网在 `y=0.65`，机架半径 `0.20`，**机身边缘已经到 0.73，螺旋桨插进网里**。真值航向在 1 秒内于 `-0.24 ↔ -1.54` 之间来回摆动，这是撞击引起的物理振荡，不是估计误差。

之后：撞击 → 偏航剧烈振荡 → 估计被打乱 → EV 位置被拒（ratio 冲到 43）→ 失控 → 掉到地面 `z=0.147` → 重新爬升 → 速度尖峰 `7.78m/s` → Gazebo 物理输出 NaN、位置变 `(0,0,0)` → 规划器 `invalid_pose` → 永久卡死。

**结论：估计器是受害者。要修的是「飞机为什么会飞到网上」，不是估计器。**

已知事实：目标是 `(1.25, -0.10)`，距网 `0.75m`；飞机冲到 `y=+0.53`，**过冲约 0.6m**。巡航速度只有 `0.18m/s`，这个过冲量异常大，下一步应查设定点链路（`maximum_setpoint_lead 0.25` 与 `ramp_setpoint` 的相互作用），而不是继续调 EKF。

围栏警戒线在 `y=0.35`（本轮修复后），理论上应在 `y=0.35` 就介入，但飞机带着动量冲过去了 —— 围栏能发现，拦不住。

### 10. 尚未完成

- **复现率只有 20%**。这是当前唯一的拦路问题，且已定位到估计器抖动而非避障逻辑（见上一节相关性表）。下一步应针对「干净轮 vs 抖动轮」做 ULog 对比，找出抖动的触发条件；`EKF2_EVA_NOISE` 已到参数下限 `0.01`，候选方向是 `EKF2_EV_DELAY` 按实测管线延迟标定。
- **单次运行结论不可靠**。务必用 `tools/run_avoidance_campaign.sh` 跑 N 轮看通过率，不要用单次结果判断改动好坏。本轮就有多次单跑结论被批量数据推翻。
- **VM 上 Gazebo 本身不稳**。`gzserver` 多次 `Aborted (core dumped)`（退出码 134）。表现为 GUI 窗口还在（`gzclient` 活着）但物理已停、`/clock` 不再发布、`model_states` 超时。排查时先确认 `gzserver` 进程是否还在，不要被残留窗口误导。
- **完整比赛链未跑通**。识别与投放由队友模块负责，当前用脚本直发目标绕开。`mission_manager` 在识别无结果时会在仿真 `167` 秒后自行判定 `mission complete` 退出，因其为 `required` 节点会拖垮整个 launch。
- **`altitude_out_of_band`**：起飞瞬态会短暂触发，属正常；但降落后会持续报该原因，需要区分处理。

## 2026-08-30 PX4 起飞健康门禁

- `competition_main.py` 新增纯逻辑 `PreflightHealthGate`。
- 起飞前组合检查 MAVROS 连接、PX4 `MAV_STATE_STANDBY`、估计器姿态有效、加速度计无错误、IMU 消息新鲜度与数值范围。
- 默认要求连续健康 `3.0s`；消息最大年龄 `1.5s`；静止加速度模长范围 `5.0–20.0m/s²`。
- VM 实测 `/mavros/estimator_status` 约 `1.000Hz`、`/mavros/imu/data` 约 `50Hz`。一次 4.6 秒窗口内 233 个 IMU 样本最大值 `15.026m/s²`；旧上限 `15.0` 会被单个正常尖峰反复重置，故上调到 `20.0`。
- 任一条件失败保持 `WAIT_SENSOR`，不发布起飞设定点、不请求 OFFBOARD、不解锁，并清空旧预发送计数。
- 已解锁后不再用预飞门禁切断设定点；飞行中异常继续由位姿、地理围栏和安全监控处理。
- 本地相关测试 `35/35` 通过；VM 全量测试 `205/205` 通过；`catkin_make` 构建至 `100%`。
- 校准后首轮种子 1：门禁约在仿真 24 秒后放行，PX4 成功进入 `OFFBOARD` 并解锁，证明门禁不再因 1Hz 估计器话题永久阻塞。
- 该轮仍碰北侧安全网：MAVROS 约 `(0.153, 0.220, 1.238)`，Gazebo 约 `(0.166, 0.406, 1.265)`，Y 误差约 `0.186m`。这是既有外部视觉定位重复性问题；种子 1 三连通过门槛仍失败。

## 2026-08-30 外部视觉跟踪诊断

- seed 1 原始障碍流已录制。深度节点持续发布 `ready=true`，并能生成障碍簇；但多次运行在到达随机圆柱前已于原地转向阶段撞北侧安全网。
- 碰撞前路径设定点保持向南，Gazebo 机体却向北移动，排除“规划目标直接指向安全网”。
- 对齐录包显示 PX4 本地航向与 Gazebo 航向差通常小于 `0.10rad`，仅修正地图坐标旋转不能解决问题。
- `EKF2_AID_MASK=280`（视觉位置+速度+航向、禁用磁力计）仍出现约 `0.22m` 跟踪误差；去掉视觉速度改为 `24` 后误差扩大到约 `0.43m`，该失败实验已回退。
- PX4 ULog 显示视觉位置融合仍激活、水平位置 innovation test ratio 未超限，但 `output_tracking_error` 位置约 `0.223m`、速度约 `0.344m/s`。下一步重点检查输出预测器时间常数 `EKF2_TAU_POS`、`EKF2_TAU_VEL`，而非继续改路径几何。
- 当前保留 `EKF2_AID_MASK=280` 作为诊断检查点，不代表完成验收。

## 本轮新增

- 新增 `gazebo_vision_bridge.py`：把 `iris_0` Gazebo 位姿、世界速度转换为 MAVROS 机体系里程计，发布到 `/mavros/odometry/out`。
- 新增 `external_vision.py`：模型选择、世界速度到机体系转换等纯逻辑。
- 新增 `config/px4/10016_iris.post`：启用 PX4 v1.11 外部视觉位置、速度、高度融合；清除持久磁偏置影响。
- `start_sitl.sh` 在启动前安装受版本控制的 PX4 `.post` 配置。
- smoke 取证新增 MAVROS/Gazebo 水平对齐误差。
- 路径规划器可在转向阶段记住视野内远端随机圆柱，并在近距局部制动前触发动态 A* 重规划。
- 安全网预警/恢复边界调整为 `0.35m / 0.55m`，减少过早恢复动作。

## 最新验证证据

### 已通过

- 本地专项测试：84/84 通过（路径规划、外部视觉、包结构、场地图）。
- VM：`catkin_make` 通过。
- VM 相关测试：71/71 通过（路径规划 55、外部视觉 6、包结构 10）。
- 种子 1 曾一次完整通过：`collision=false`、`reached_goal=true`、`path_state=REACHED`。
- 该次高度范围：`1.158–1.211m`；最大 MAVROS/Gazebo 水平误差：`0.042m`；事件含 `dynamic_route_replanned`。

### 尚未通过

- 种子 1 重复性门槛失败。
- 一次重复运行撞西侧安全网，最大对齐误差约 `0.224m`。
- 最新运行较早触发北侧安全网，Gazebo 与 MAVROS Y 方向位置差约 `0.22m`。
- 四种随机圆柱矩阵尚未运行；完整比赛链尚未验收。

不得把“单次种子 1 通过”写成稳定完成。

## 当前判断与下一步

P0 是 PX4 外部视觉融合后的输出预测器跟踪误差，不是场地图几何。起飞健康门禁已阻止明显不健康状态解锁；桥接 50Hz、视觉航向融合、禁用磁力计仍不能单独解决转向漂移。

1. A/B 测试 `EKF2_TAU_POS`、`EKF2_TAU_VEL`，以原地转向期间 Gazebo/MAVROS 误差和 ULog `output_tracking_error` 为判据。
2. 先消除北侧漂移，再录制完整 `/fire_mission/obstacles` 流，继续验证随机圆柱进入动态重规划/局部绕桩状态机。
3. 修正定位重复性后连续运行种子 1 三次；每次检查是否误解锁、碰撞、到达、高度和最大位姿误差。
4. 三次均通过后，运行随机圆柱种子 `1、4、10、2` 矩阵。
5. 矩阵通过后，再恢复危险品识别、投放、人员救助、返航完整链。
6. 真机前替换 Gazebo 真值源为标定后的双目 VIO；当前外部视觉桥只准用于 SITL。

## 仓库与环境

- GitHub：`https://github.com/NOSTALGIA1000/firefighting-mission-sitl`
- 分支：`feature/firefighting-sitl`
- VM：Ubuntu 18.04、ROS Melodic、Gazebo 9、PX4 v1.11、MAVROS
- VM 工作区：`/home/ss/catkin_ws`
- ROS 包：`/home/ss/catkin_ws/src/firefighting_mission`

首次接手：

```bash
git clone https://github.com/NOSTALGIA1000/firefighting-mission-sitl.git
cd firefighting-mission-sitl
git checkout feature/firefighting-sitl
```

## 控制链与边界

```text
/fire_mission/point_goal
  -> 固定地图 A* + 双目视觉动态重规划 path_planner.py
  -> /fire_mission/path_setpoint
  -> competition_main.py（唯一 MAVROS 位置设定点发布者）
  -> /mavros/setpoint_position/local
  -> PX4 OFFBOARD
```

- 禁止其他节点直接发布 MAVROS 位置设定点。
- 禁止读取随机圆柱 Gazebo 真值作为活动避障输入。
- `gazebo_vision_bridge.py` 仅提供 SITL 定位，不参与随机圆柱检测。
- 高度目标 `1.20m`，验收带 `1.10–1.30m`。

## 关键文件

- `scripts/competition_main.py`：唯一 MAVROS 位置设定点发布节点。
- `src/firefighting_mission/path_planner.py`：A*、视觉圆柱记忆、动态重规划和局部绕障。
- `scripts/gazebo_vision_bridge.py`：Gazebo 到 MAVROS 外部视觉桥。
- `src/firefighting_mission/external_vision.py`：外部视觉纯逻辑。
- `config/px4/10016_iris.post`：PX4 SITL EKF2 配置。
- `scripts/start_sitl.sh`：SITL 启动和 PX4 配置安装。
- `test/visual_avoidance_smoke.py`：SITL 取证测试。
- `test/test_external_vision.py`：外部视觉单测。
- `test/test_path_planner.py`：路径规划单测。

## VM 运行命令

```bash
cd /home/ss/catkin_ws
source /opt/ros/melodic/setup.bash
catkin_make
source devel/setup.bash
export ROS_IP=127.0.0.1
export ROS_HOSTNAME=127.0.0.1
export ROS_MASTER_URI=http://127.0.0.1:11311
export DISPLAY=:0
rostest firefighting_mission visual_avoidance_smoke.test seed:=1
```

旧 PX4 参数文件已保留在：`/home/ss/.ros/eeprom/parameters_10016.corrupt-20260829-1936`。
