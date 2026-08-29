# 代码清单与进展

更新时间：2026-08-29。真实状态和接手顺序见 `docs/CURRENT_HANDOFF.zh-CN.md`。

## 代码入口

- `scripts/competition_main.py`：MAVROS 主控，唯一位置设定点发布者。
- `src/firefighting_mission/competition_main.py`：OFFBOARD 起飞、1.2m 悬停纯状态机。
- `src/firefighting_mission/state_machine.py`、`scripts/mission_manager_node.py`：比赛阶段编排。
- `src/firefighting_mission/world_generator.py`：4m 场地、固定/随机障碍、任务区、安全网。
- `src/firefighting_mission/field_map.py`：固定地图、A*、固定表面匹配。
- `src/firefighting_mission/path_planner.py`：航路跟随、视觉圆柱记忆、动态 A*、局部绕障。
- `scripts/stereo_obstacle_node.py`、`src/firefighting_mission/stereo_obstacles.py`：双目深度障碍提取。
- `scripts/gazebo_vision_bridge.py`、`src/firefighting_mission/external_vision.py`：SITL 外部视觉定位桥和纯逻辑。
- `config/px4/10016_iris.post`、`scripts/start_sitl.sh`：PX4 EKF2 外部视觉配置与启动。
- `src/firefighting_mission/supply_drop.py`、`scripts/supply_drop.py`、`src/payload_plugin.cpp`：双通道物资投放。
- `scripts/target_detector_node.py`：危险品/人员识别入口。
- `scripts/safety_monitor_node.py`：边界、碰撞、感知超时保护。
- `test/visual_avoidance_smoke.py`：SITL 绕桩、碰撞、高度、位姿对齐取证。

## 当前能力

- 场地建模：完成。
- 自动起飞与约 1.2m 悬停：通过。
- 固定地图 A*：完成。
- 双目障碍提取、视觉圆柱动态重规划和局部绕障：已实现。
- 双通道 Gazebo 投放：通过。
- PX4 外部视觉位置/速度融合：已实现，重复性未验收。
- 完整比赛链：模块具备，尚未完成无碰撞整链验证。
- 真机双目适配：未开始，缺型号、内参、基线、深度/VIO 输出确认。

## 最新证据

- 本地专项测试：84/84 通过。
- VM 相关测试：71/71 通过；`catkin_make` 通过。
- 种子 1：曾单次 `collision=false`、`REACHED`，高度 `1.158–1.211m`，最大定位误差 `0.042m`。
- 重复种子 1：仍可能出现约 `0.22m` 定位滞后并碰安全网。
- 四种圆柱矩阵：未运行。

## 当前 P0

提高外部视觉发布频率并验证重复性。先连续通过两次种子 1，再运行 `1、4、10、2` 矩阵。未达到此门槛前，不得标记视觉绕桩或完整任务“完成”。
