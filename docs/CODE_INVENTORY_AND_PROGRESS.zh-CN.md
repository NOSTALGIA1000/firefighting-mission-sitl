# 代码清单与进展

更新时间：2026-08-24。详细状态见 `docs/CURRENT_HANDOFF.zh-CN.md`。

## 代码入口

### 主控与任务

- `src/firefighting_mission/competition_main.py`：OFFBOARD 起飞/悬停纯状态机。
- `scripts/competition_main.py`：MAVROS 节点，唯一位置设定点发布者。
- `src/firefighting_mission/state_machine.py`：比赛阶段状态机。
- `scripts/mission_manager_node.py`：任务编排节点。

### 场地与固定地图

- `src/firefighting_mission/world_generator.py`：4m 场地、障碍、任务区、安全网生成。
- `src/firefighting_mission/field_map.py`：固定地图、A*、固定表面匹配。
- `scripts/generate_world.py`：种子世界生成入口。

### 双目与路径

- `src/firefighting_mission/stereo_obstacles.py`：深度/点云障碍簇提取。
- `scripts/stereo_obstacle_node.py`：`depth/points/raw_stereo` ROS 适配。
- `src/firefighting_mission/path_planner.py`：A* 航路跟随和视觉绕桩状态机。
- `scripts/path_planner.py`：目标、姿态、障碍消息接线与设定点平滑。
- `msg/Obstacle.msg`、`msg/ObstacleArray.msg`：障碍消息。
- `msg/AvoidanceStatus.msg`：绕障诊断消息。

### 识别、投放、安全、记录

- `scripts/target_detector_node.py`：危险品和人员检测入口。
- `src/firefighting_mission/supply_drop.py`、`scripts/supply_drop.py`：双通道投放门控。
- `src/payload_plugin.cpp`：Gazebo 载荷分离插件。
- `scripts/safety_monitor_node.py`：边界、碰撞、感知超时保护。
- `scripts/mission_recorder_node.py`：轨迹、事件、状态、证据记录。
- `scripts/mission_overlay_node.py`：回传画面状态叠加。

### Launch 与工具

- `launch/firefighting.launch`：当前图形化活动任务链。
- `launch/firefighting_headless.launch`：无界面任务链。
- `scripts/start_sitl.sh`：PX4/Gazebo/MAVROS 启动。
- `scripts/run_avoidance_matrix.sh`：四种圆柱组合手动验收。

### 测试

- `test/test_path_planner.py`：路径和绕障纯逻辑。
- `test/path_planner_ros_test.py`：路径 ROS 合约。
- `test/visual_avoidance_smoke.py`：真实 SITL 绕桩与取证。
- `test/test_stereo_obstacles.py`：双目障碍提取。
- `test/test_competition_main.py`：起飞状态机。
- `test/test_world_generator.py`：场地几何。
- `test/test_supply_drop.py`、`test/payload_drop_ros_test.py`：投放门控与物理分离。
- 其他 `test/test_*.py`：任务编排、安全、识别、评分、包结构。

## 当前进展

- 场地建模：完成。
- 自动起飞与 1.2m悬停：VM 通过。
- 固定地图 A*：完成，测试通过。
- 双目障碍提取：完成，测试通过。
- 视觉绕桩状态机：完成纯逻辑，SITL 实飞未通过。
- 固定墙/安全网误识别过滤：已实现，专项测试 19/19 通过。
- 双通道 Gazebo 投放：通过。
- 完整比赛链：未最终验证。
- 真机适配：未开始标定。

## 当前 P0

种子 1 实飞中，圆柱暂时离开视野后规划器可能从 `HOLD_UNSAFE` 恢复 `FOLLOW_ROUTE` 并撞柱。下一位开发者应先修该状态恢复条件，再跑种子 `1、4、10、2`。

另需补 `path_planner` 节点自身障碍消息超时看门狗；当前安全节点虽能悬停/降落，但规划器可能保留旧 `ready=true`。

## 验证记录

- 完整 Python 2.7 套件：140/140 通过（含固定地图融合回归）。
- 路径专项：19/19 通过。
- `catkin_make`：最近一次通过。
- 路径 ROS 合约：最近一次通过。
- 四种子绕桩矩阵：未通过，不可标记完成。
