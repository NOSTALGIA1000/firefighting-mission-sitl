# 队员 C：路径规划与物资投放交接

先读：`docs/CURRENT_HANDOFF.zh-CN.md`。

## 当前方案

- 活动方案：固定地图 A* + 前向双目视觉随机圆柱绕障。
- 全程目标高度：`1.20m`；验收范围：`1.10–1.30m`。
- 旧 `CLIMB -> CRUISE(2.30m) -> DESCEND` 仅保留在历史设计文档，不再用于比赛活动链。
- `competition_main.py` 是唯一 MAVROS 位置设定点发布者。

## 路径接口

| 接口 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/fire_mission/point_goal` | `geometry_msgs/PoseStamped` | 输入 | 任务目标点 |
| `/fire_mission/obstacles` | `ObstacleArray` | 输入 | 双目障碍簇与感知状态 |
| `/fire_mission/path_setpoint` | `geometry_msgs/PoseStamped` | 输出 | 发给主控的内部设定点 |
| `/fire_mission/path_status` | `std_msgs/String` | 输出 | 当前规划状态 |
| `/fire_mission/avoidance_status` | `AvoidanceStatus` | 输出 | 绕障方向、净空、原因、目标 |

路径状态：

```text
FOLLOW_ROUTE
  -> BRAKE -> OBSERVE -> SELECT_SIDE
  -> SIDESTEP -> PASS -> REJOIN
  -> FOLLOW_ROUTE -> REACHED
```

安全异常使用 `HOLD_UNSAFE`。

## 双目输入模式

`stereo_obstacle_node.py` 支持：

- `depth`：已对齐深度图，仿真当前使用。
- `points`：点云输入。
- `raw_stereo`：左右图内部计算视差。

仿真相机前置偏移默认 `0.32m`，由 `path_planner.py` 参数 `sensor_forward_offset` 设置。真机必须重新测量，不可照抄。

活动路径代码不得读取随机圆柱真值 `CYLINDER_POSES` 或 Gazebo `/gazebo/model_states`。

## 投放接口

| 接口 | 类型 | 说明 |
| --- | --- | --- |
| `/fire_mission/drop_supply` | `DropSupply` 服务 | 高层安全门控 |
| `/fire_iris/drop_supply` | `DropSupply` 服务 | Gazebo 低层分离 |

- `channel=1`：消防物资。
- `channel=2`：救援物资。
- 高层门控：已对准、水平速度不超过 `0.10m/s`、高度 `1.15–1.45m`、通道未使用。
- 真机替换低层 Gazebo 服务为舵机驱动服务；高层接口可保持。

## 已验证

- OFFBOARD 自动起飞、1.2m悬停。
- 路径节点到主控的单一设定点链路。
- 纯路径规划专项测试 19/19 通过。
- 固定地图 A*、相机偏移补偿、固定墙/安全网视觉过滤。
- 双通道 Gazebo 物理分离。

## 尚未通过

- 四种随机圆柱组合的完整 `SIDESTEP -> PASS -> REJOIN` 实飞。
- 圆柱暂时离开视野后的安全恢复。
- 路径节点自身的双目消息断流看门狗。
- 真机双目标定与外参配置。

## 复现当前问题

```bash
cd /home/ss/catkin_ws
source devel/setup.bash
export ROS_IP=127.0.0.1 ROS_HOSTNAME=127.0.0.1
export ROS_MASTER_URI=http://127.0.0.1:11311 DISPLAY=:0
rostest firefighting_mission visual_avoidance_smoke.test seed:=1
cat src/firefighting_mission/artifacts/avoidance_matrix/1/smoke.json
```

重点看 `event_trace`、`contact_pairs`、`clearances`、`obstacles`、`last_pose` 和 `last_yaw`。
