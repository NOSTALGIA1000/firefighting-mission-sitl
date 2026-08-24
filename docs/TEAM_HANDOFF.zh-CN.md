# 小组接手指南

## 先读

1. `docs/CURRENT_HANDOFF.zh-CN.md`：当前真实状态、风险、下一步。
2. `docs/TEAM_C_HANDOFF.zh-CN.md`：路径、双目、投放接口。
3. `docs/CODE_INVENTORY_AND_PROGRESS.zh-CN.md`：代码目录说明。

## 获取代码

```bash
git clone https://github.com/NOSTALGIA1000/firefighting-mission-sitl.git
cd firefighting-mission-sitl
git checkout feature/firefighting-sitl
```

## VM 基线

- Ubuntu 18.04
- ROS Melodic
- Gazebo 9
- PX4 SITL + MAVROS
- 工作区 `/home/ss/catkin_ws`

```bash
cd /home/ss/catkin_ws
source /opt/ros/melodic/setup.bash
catkin_make
source devel/setup.bash
```

## 当前能力边界

已完成：赛场、自动起飞悬停、固定地图 A*、双目障碍提取、1.2m视觉绕桩状态机、双物资投放插件、任务识别与编排接口。

未完成：无碰撞绕桩实飞矩阵、双目断流节点级保护、完整比赛全流程、真机双目标定。

请勿把“模块已实现”写成“完整任务已通过”。

## 协作规则

- 每人独立功能分支，提交后发 Pull Request。
- 不直接改 PX4/XTDrone 上游源码。
- 不让多个 ROS 节点同时发布 MAVROS 位置设定点。
- 不使用 Gazebo 模型真值完成视觉绕障。
- 推送前写清测试结果和未通过项。
