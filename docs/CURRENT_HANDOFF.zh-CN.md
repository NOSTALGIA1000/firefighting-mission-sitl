# 当前阶段交接（先读此文件）

更新时间：2026-08-30

## 一句话状态

场地、1.2m OFFBOARD 飞行、固定地图 A*、双目视觉绕桩、双通道投放均已实现。50Hz 外部视觉三轮同配置仿真仅一次通过；PX4 起飞健康门禁已通过 VM 全测和构建，并能安全等待后正常放行。首轮整飞仍因约 0.186m 定位偏差触碰北侧安全网；当前不是验收完成版。

## 2026-08-30 PX4 起飞健康门禁

- `competition_main.py` 新增纯逻辑 `PreflightHealthGate`。
- 起飞前组合检查 MAVROS 连接、PX4 `MAV_STATE_STANDBY`、估计器姿态有效、加速度计无错误、IMU 消息新鲜度与数值范围。
- 默认要求连续健康 `3.0s`；消息最大年龄 `1.5s`；静止加速度模长范围 `5.0–15.0m/s²`。
- VM 实测 `/mavros/estimator_status` 约 `1.000Hz`、`/mavros/imu/data` 约 `50Hz`；健康静止 IMU 样本模长曾达到约 `13.17m/s²`，因此未采用会周期性误拒绝的 `0.5s / 7–12m/s²` 初始值。
- 任一条件失败保持 `WAIT_SENSOR`，不发布起飞设定点、不请求 OFFBOARD、不解锁，并清空旧预发送计数。
- 已解锁后不再用预飞门禁切断设定点；飞行中异常继续由位姿、地理围栏和安全监控处理。
- 本地相关测试 `35/35` 通过；VM 全量测试 `205/205` 通过；`catkin_make` 构建至 `100%`。
- 校准后首轮种子 1：门禁约在仿真 24 秒后放行，PX4 成功进入 `OFFBOARD` 并解锁，证明门禁不再因 1Hz 估计器话题永久阻塞。
- 该轮仍碰北侧安全网：MAVROS 约 `(0.153, 0.220, 1.238)`，Gazebo 约 `(0.166, 0.406, 1.265)`，Y 误差约 `0.186m`。这是既有外部视觉定位重复性问题；种子 1 三连通过门槛仍失败。

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

P0 是 PX4 IMU/估计器启动健康与外部视觉重复性，不是场地图几何。桥接发布频率已提高到 50Hz，但单独提高频率不能保证稳定起飞。

1. 录制下一次种子 1 的 MAVROS/Gazebo 位姿、设定点和姿态，定位北侧漂移来源。
2. 修正定位重复性后连续运行种子 1 三次；每次检查是否误解锁、碰撞、到达、高度和最大位姿误差。
3. 三次均通过后，运行随机圆柱种子 `1、4、10、2` 矩阵。
4. 矩阵通过后，再恢复危险品识别、投放、人员救助、返航完整链。
5. 真机前替换 Gazebo 真值源为标定后的双目 VIO；当前外部视觉桥只准用于 SITL。

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
