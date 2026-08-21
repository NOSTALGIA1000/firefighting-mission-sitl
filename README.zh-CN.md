# 消防救援 SITL 任务

本包为 PX4、XTDrone、ROS Melodic 与 Gazebo 9 的自主低空消防救援仿真。任务自动完成起飞、危险品识别和红框标注、消防物资投放、人员识别和蓝框标注、救援物资投放、返航、降落与停桨。

## 接手说明

小组成员第一次接手时，先阅读：

- `docs/TEAM_HANDOFF.zh-CN.md`：运行环境、接手步骤、当前可用能力和待办事项。
- `docs/CODE_INVENTORY_AND_PROGRESS.zh-CN.md`：完整代码清单、模块用途、验证记录和当前进展。

## 一键运行

在已完成 catkin 编译、并已 source 工作空间与 XTDrone/PX4 环境的 Ubuntu 18.04 虚拟机中运行：

```bash
roslaunch firefighting_mission firefighting_headless.launch seed:=4501 record:=true
```

或：

```bash
rosrun firefighting_mission run_mission.sh 4501
```

`seed` 决定两个圆柱、正确危险品与人员位置；相同种子可重复生成相同场景。桌面模式使用 `firefighting.launch`，无界面模式使用 `firefighting_headless.launch`。

任务完成后，`artifacts/<seed>/` 包含：

- `score.json`：机器可读成绩与失败原因；写入使用临时文件再原子替换。
- `events.log`：阶段转换与投放事件。
- `trajectory.csv`：带任务阶段的飞行轨迹。
- `mission.bag` 与 `annotated.mp4`：在 `record:=true` 时保存的话题记录和标注视频。

## 硬性判定

通过必须同时满足：总用时不超过 180 秒、最小净空不少于 0.35 m、危险品和人员均已确认、两次投放误差均不超过 0.20 m、最终落点距起飞圆心不超过 0.25 m、无碰撞、任务完成且飞控已解锁关闭。

## 常见问题

- 未连接 PX4、未获得初始位姿或安全状态不为 `CLEAR` 时，任务不会解锁。
- 无视频、bag 或分数文件时，确认 `record:=true`，并检查 Gazebo、MAVROS 与下视相机话题是否可用。
- 本包不修改 PX4_Firmware 或 XTDrone。替换为 450 机体时，只需保持现有 MAVROS、速度命令、雷达、相机和双投放话题契约。
