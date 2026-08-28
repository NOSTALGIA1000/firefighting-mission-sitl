# 当前阶段交接（先读此文件）

更新时间：2026-08-28

## 一句话状态

赛场、PX4/MAVROS 自动起飞悬停、双通道物资投放、固定地图 A*、双目深度障碍提取、1.2 米视觉绕桩状态机均已实现。当前已定位重复 SITL 碰撞的主要剩余问题：Gazebo 场地 Y 轴运动方向与 PX4/MAVROS 本地 Y 轴在本机仿真链路中相反；适配尚未实现，不能视为验收完成。

## 2026-08-28 最新诊断

- 统一控制权：`competition_main.py` 仍是唯一 MAVROS 位置设定点发布者；场地图到 PX4 本地坐标的转换已集中到该节点，覆盖起飞、悬停、航路。
- `path_planner.py` 只输出场地图坐标；仿真可用 `/gazebo/model_states` 做规划姿态对齐，不读取随机圆柱真值进行避障。
- PX4 在约 5.2 秒开始 GPS 融合并重置位置/速度。SITL 预发送从 40 帧改为 120 帧，解锁推迟到约 8.4 秒；`invalid setpoints`/盲降问题消失。
- smoke 测试新增 MAVROS 位姿、Gazebo 位姿、设定点、模式、解锁状态和首碰撞对象证据。
- 最新种子 1：仍碰北侧安全网。首碰撞时 MAVROS 位姿约 `(0.138, -0.119, 1.207)`，Gazebo 位姿约 `(0.310, 0.399, 1.448)`，实际下发设定点约 `(-0.127, -0.605, 1.2)`。本地负 Y 命令对应世界正 Y 运动，已明确需要轴符号适配。
- 本地专项测试：117/117 通过。VM `catkin_make` 通过。SITL smoke 仍失败，四种子矩阵未运行。
- 下一步首项：给 `map_target_to_local` 增加可配置轴符号；SITL 使用 `x_sign=+1、y_sign=-1`，并同步处理航向差符号。先写失败单测，再跑种子 1；通过后才能跑 `1、4、10、2` 矩阵。

## 仓库与环境

- GitHub：`https://github.com/NOSTALGIA1000/firefighting-mission-sitl`
- 工作分支：`feature/firefighting-sitl`
- VM：Ubuntu 18.04、ROS Melodic、Gazebo 9、PX4 SITL、MAVROS
- VM 工作区：`/home/ss/catkin_ws`
- ROS 包：`/home/ss/catkin_ws/src/firefighting_mission`
- VM ROS 网络：`~/.bashrc` 已修成动态 `ROS_IP=$(hostname -I | awk '{print $1}')`，`ROS_MASTER_URI=http://127.0.0.1:11311`；旧配置备份为 `~/.bashrc.codex-rosnet-20260827.bak`。

首次接手：

```bash
git clone https://github.com/NOSTALGIA1000/firefighting-mission-sitl.git
cd firefighting-mission-sitl
git checkout feature/firefighting-sitl
```

## 当前控制链

```text
任务目标 /fire_mission/point_goal
  -> 固定地图 A* + 双目视觉绕桩 path_planner.py
  -> 内部设定点 /fire_mission/path_setpoint
  -> competition_main.py（唯一 MAVROS 位置设定点发布者）
  -> /mavros/setpoint_position/local
  -> PX4 OFFBOARD
```

禁止让其他节点直接向 MAVROS 位置设定点话题发布，否则会出现控制权冲突。

## 已完成并验证

### 场地与模型

- 4m × 4m × 3m 场地、透明安全网、500mm 起飞区。
- 固定障碍 1–4；固定障碍 2 为 45°，下方对应固定障碍 4。
- 两个随机圆柱及四种摆放组合。
- 两个 400mm × 400mm 危险品区、三个救助候选位置和目标贴图。
- 前向双目/深度仿真传感器已安装到 `fire_iris` 模型。

### 队员 A 控制

- MAVROS 连接、设定点预发送、OFFBOARD、自动解锁。
- 自动起飞至约 1.2m并稳定悬停。
- `competition_main.py` 为唯一 MAVROS 设定点发布者。
- VM 实测稳定悬停约 `(0.04, 0.00, 1.25)`，模式 `OFFBOARD`。

### 队员 C 路径与投放

- 旧 `2.30m` 越障方案已退出活动链路。
- 当前飞行高度目标为 `1.20m`，允许验收带 `1.10–1.30m`。
- 固定障碍由 A* 地图规划绕行；随机圆柱只允许由双目视觉发现。
- 绕桩状态：`BRAKE -> OBSERVE -> SELECT_SIDE -> SIDESTEP -> PASS -> REJOIN`。
- 双通道投放：`1=FIRE`，`2=RESCUE`；Gazebo 分离插件物理测试已通过。

### 当前自动测试基线

- 本轮相关纯逻辑测试：48/48 通过（路径规划 31、场地图 10、双目障碍 7）。
- Windows 内置 Python 完整 discover：149 项中 147 项执行通过，2 项因本机缺 `cv2` 无法导入；需在 VM 的 ROS/Python2 环境重跑完整基线。
- 上一阶段完整 Python 2.7 测试基线：140/140 通过；本轮修改后尚未在 VM 重跑全部 140 项。
- `catkin_make`：最近一次通过。
- `rostest firefighting_mission path_planner.test`：2026-08-27 在 VM 通过；测试前修复了旧网段 ROS 环境变量。

不要把“四种随机圆柱实飞矩阵”写成已通过；它仍是当前验收目标。

## 当前未解决问题

### P0：绕桩实飞重复性不足

当前代码已加入：

- 相机前置偏移补偿，仿真默认 `0.32m`。
- 固定地图表面过滤，避免已知墙体重复触发局部绕障。
- 视觉圆柱短时记忆，并写入动态障碍地图供 A* 重规划。
- 动态重规划起点逃逸，避免起点落入膨胀区后无路可走。
- 地面/固定结构/圆柱候选分层，减少深度误检。
- `BRAKE/OBSERVE/SELECT_SIDE/HOLD_UNSAFE` 悬停位置锁存，避免悬停目标跟随机体漂移。
- 视野横向触发范围为 `±0.55m`，近距局部触发为 `1.00m`。
- 远处稳定圆柱触发全局动态重规划：默认 `2.00m` 内连续 2 帧确认后写入动态障碍地图。

验证结果：

- 种子 1 曾一次完整经过 `SIDESTEP / PASS / REJOIN` 并通过 smoke test。
- 重复运行仍需在 SITL 里验证；远距动态 A* 已落地，但尚未完成四种子实飞矩阵。
- 将局部触发范围简单放宽，会生成过长侧移路线，并可能撞固定障碍 3/4 或南侧安全网；该实验已回退。
- 当前架构：远处稳定视觉圆柱写入动态地图并触发全局 A*；近处突发障碍继续使用局部状态机。

仍需解决：`HOLD_UNSAFE` 后不能仅因目标暂时离开视野就恢复直飞。应要求重新观测确认、完成安全侧选择，或保持悬停等待。

### P0：传感器节点断流保护不完整

`safety_monitor` 能对双目超时执行悬停/降落；`path_planner` 节点自身还缺“最后一帧障碍消息时间”看门狗。若双目节点直接退出，规划器可能保留旧的 `ready=true`。下一步需增加消息超时后强制 `HOLD_UNSAFE` 的节点级保护和测试。

### P1：完整比赛链未最终通过

危险品识别、消防投放、人员识别、救援投放、返航、降落已有模块和接口，但尚未完成一次无碰撞、全流程、三分钟内的整链路验证。

### 真机边界

实机确认有双目摄像头，但型号、内参、基线、是否输出深度仍未知。真机前必须确认话题、标定和坐标外参；禁止直接沿用仿真 `0.32m` 参数飞行。

## 接手后执行顺序

1. 阅读 `docs/superpowers/specs/2026-08-24-stereo-visual-avoidance-design.md`。
2. 跑纯逻辑与 ROS 合约测试，确认基线。
3. 用种子 1 复现绕桩，查看 `artifacts/avoidance_matrix/1/smoke.json` 的 `event_trace`。
4. 先在 VM 重跑完整 Python2 测试和路径 ROS 合约。
5. 再跑种子 `1、4、10、2`，覆盖四种圆柱位置组合。
6. 增加双目断流测试，确认规划器悬停、安全模块最终降落。
7. 最后恢复完整任务链验证，不要同时调识别与绕桩。

## 常用命令

```bash
cd /home/ss/catkin_ws
source /opt/ros/melodic/setup.bash
catkin_make
source devel/setup.bash
export ROS_IP=127.0.0.1
export ROS_HOSTNAME=127.0.0.1
export ROS_MASTER_URI=http://127.0.0.1:11311
export DISPLAY=:0
```

路径专项测试：

```bash
cd /home/ss/catkin_ws/src/firefighting_mission
PYTHONPATH=src python -m unittest test.test_path_planner
cd /home/ss/catkin_ws
rostest firefighting_mission path_planner.test
```

单一种子绕桩诊断：

```bash
rostest firefighting_mission visual_avoidance_smoke.test seed:=1
cat /home/ss/catkin_ws/src/firefighting_mission/artifacts/avoidance_matrix/1/smoke.json
```

离线分析控制、障碍和规划决策：

```bash
cd /home/ss/catkin_ws/src/firefighting_mission
PYTHONPATH=src python scripts/analyze_control_bag.py /tmp/control_diag2.bag
```

四种位置矩阵（仅在种子 1 通过后执行）：

```bash
rosrun firefighting_mission run_avoidance_matrix.sh
```

## 关键文件

- `scripts/competition_main.py`：唯一 MAVROS 设定点发布节点。
- `src/firefighting_mission/competition_main.py`：起飞/悬停状态机。
- `scripts/path_planner.py`：路径 ROS 节点、相机外参参数入口。
- `src/firefighting_mission/path_planner.py`：A* 航路跟随与视觉绕桩状态机。
- `src/firefighting_mission/field_map.py`：固定地图、A*、固定表面匹配。
- `scripts/stereo_obstacle_node.py`：深度图/点云/原始双目适配。
- `src/firefighting_mission/stereo_obstacles.py`：障碍簇提取。
- `test/visual_avoidance_smoke.py`：SITL 绕桩取证测试。
- `scripts/run_avoidance_matrix.sh`：四种随机圆柱组合验收入口。
- `scripts/analyze_control_bag.py`：离线对齐模式、位姿、设定点、障碍世界坐标和触发判定。
- `docs/TEAM_C_HANDOFF.zh-CN.md`：队员 C 详细接口。

## 提交原则

- 不读取 `CYLINDER_POSES` 或 `/gazebo/model_states` 作为活动避障输入。
- 不把未跑通的实飞写成通过。
- 每次修复先补失败测试，再运行专项测试和四种子实飞。
- 真机参数与仿真参数分开配置。
