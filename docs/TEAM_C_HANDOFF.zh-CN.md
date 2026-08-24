# 队员 C 路径规划与物资投放接手说明

## 已交付能力

- 点到点固定高度避障：`CLIMB -> CRUISE -> DESCEND -> REACHED`。
- 默认安全巡航高度：`2.30 m`；水平到点阈值：`0.12 m`；垂直到点阈值：`0.08 m`。
- `competition_main.py` 保持为唯一 MAVROS 位置设定点发布者，路径节点只发布内部设定点，避免控制冲突。
- 双通道投放：`1=FIRE`（消防物资），`2=RESCUE`（救援物资）。
- 高层投放服务检查对准、水平速度、投放高度和重复投放，再调用 Gazebo 低层分离服务。
- Gazebo 插件支持 ROS Service，并保留原有 `drop_fire`、`drop_rescue` Bool 话题兼容。

## 代码位置

- 纯路径逻辑：`src/firefighting_mission/path_planner.py`
- 路径 ROS 节点：`scripts/path_planner.py`
- OFFBOARD 接线：`scripts/competition_main.py`、`src/firefighting_mission/competition_main.py`
- 纯投放门控：`src/firefighting_mission/supply_drop.py`
- 投放 ROS 节点：`scripts/supply_drop.py`
- Gazebo 分离插件：`include/firefighting_mission/payload_plugin.hpp`、`src/payload_plugin.cpp`
- 服务定义：`srv/DropSupply.srv`
- 单元与集成测试：`test/test_path_planner.py`、`test/path_planner_ros_test.py`、`test/test_supply_drop.py`、`test/payload_drop_ros_test.py`

## ROS 接口

| 名称 | 类型 | 方向 | 用途 |
| --- | --- | --- | --- |
| `/fire_mission/point_goal` | `geometry_msgs/PoseStamped` | 输入 | 最终目标点 |
| `/fire_mission/path_setpoint` | `geometry_msgs/PoseStamped` | 输出 | 当前分段设定点 |
| `/fire_mission/path_status` | `std_msgs/String` | 输出 | `IDLE/CLIMB/CRUISE/DESCEND/REACHED` |
| `/fire_mission/aligned` | `std_msgs/Bool` | 输入 | 目标对准确认 |
| `/fire_mission/drop_supply` | `firefighting_mission/DropSupply` | 高层服务 | 安全门控投放 |
| `/fire_iris/drop_supply` | `firefighting_mission/DropSupply` | 低层服务 | Gazebo 关节分离 |

高层投放条件：已对准、水平速度不超过 `0.10 m/s`、高度处于 `1.15-1.45 m`、该通道未释放。低层调用失败不会消耗通道，可排障后重试。

## 编译与测试

```bash
cd /home/ss/catkin_ws
source /opt/ros/melodic/setup.bash
catkin_make
source devel/setup.bash
rostest firefighting_mission path_planner.test
rostest firefighting_mission payload_drop.test
```

## 路径飞行演示

终端 1：

```bash
roslaunch firefighting_mission competition_takeoff.launch gui:=true enable_path_planner:=true
```

待状态进入 `HOVER` 后，终端 2 发布目标：

```bash
rostopic pub -1 /fire_mission/point_goal geometry_msgs/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 2.70, y: -1.90, z: 1.20}, orientation: {w: 1.0}}}"
```

观察阶段：

```bash
rostopic echo /fire_mission/path_status
rostopic echo /mavros/local_position/pose
```

## 投放调用

完整消防 launch 已启动 `supply_drop.py`。满足投放条件并保持 `/fire_mission/aligned=true` 后：

```bash
rosservice call /fire_mission/drop_supply "channel: 1"
rosservice call /fire_mission/drop_supply "channel: 2"
```

仅调试 Gazebo 分离机构时，可绕过高层门控：

```bash
rosservice call /fire_iris/drop_supply "channel: 1"
```

## 当前边界

- 策略是固定高度分段飞行，不是在线 A*、RRT 或动态重规划。
- `2.30 m` 安全高度按当前最高 `2.00 m` 障碍设计；更换场地或机体尺寸后需重新校核净空。
- 真机使用时应把 Gazebo 低层服务替换成舵机驱动服务，高层安全门控与通道语义可保持不变。

## 验证记录

- Windows Python 单元测试：`99/99 PASS`。
- VM Python 2.7 单元测试：`99/99 PASS`。
- VM `catkin_make`：通过。
- VM 路径 ROS 合约：通过，确认 `CLIMB -> CRUISE -> DESCEND -> REACHED`。
- VM Gazebo 物理投放测试：通过；已验证通道 1 分离、通道 2 保持、重复拒绝和旧话题兼容。
- VM 原生 PX4 Iris 实飞：通过。目标 `(2.70,-1.90,1.20)` 到达；随后返回 `(0,0,1.20)`，最终位置 `(-0.044,0.041,1.233)`，状态 `REACHED`，飞控保持 `OFFBOARD`。
