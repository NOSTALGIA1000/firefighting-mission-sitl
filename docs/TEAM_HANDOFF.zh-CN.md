# 小组成员接手指南

本文档面向接手项目的成员，说明当前代码能做什么、如何运行、如何验证，以及哪些问题还没有完成。

## 先看结论

当前仓库已经完成消防仿真任务包的主体代码和赛题场地建模，适合继续在 VM 中查看 Gazebo 场地、调试飞控和推进完整自主任务。

当前可以较明确使用的部分：

- ROS 包结构、消息、launch、脚本入口完整。
- Gazebo 消防场地已按参考图修正。
- 危险品区、人员救助区、随机障碍、固定障碍、安全网已生成。
- 主机和 VM 的单元测试已通过。
- VM 中可以打开 Gazebo 图形场景检查场地。
- PX4 Iris 已完成 OFFBOARD 起飞和固定高度分段点到点实飞。
- 双通道 Gazebo 物资分离服务已通过物理测试。

当前不能承诺已经完成的部分：

- 完整自主飞行任务还没有最终跑通。
- 状态机尚未自动生成完整比赛航点并串联识别、两次投放和返航。
- 450 实机或最终比赛机体替换还没有完成硬件级验证。

## GitHub 仓库

仓库地址：

`https://github.com/NOSTALGIA1000/firefighting-mission-sitl`

当前分支：

`feature/firefighting-sitl`

建议成员直接从该分支接手，不要先切到其他分支。

```bash
git clone https://github.com/NOSTALGIA1000/firefighting-mission-sitl.git
cd firefighting-mission-sitl
git checkout feature/firefighting-sitl
```

如果仓库是私有仓库，成员需要先让仓库所有者在 GitHub 上添加协作者权限。

## 推荐运行环境

当前验证过的环境是：

- Ubuntu 18.04
- ROS Melodic
- Gazebo 9
- PX4 SITL
- MAVROS
- XTDrone 相关环境
- Python 2.7，用于 ROS Melodic 节点和测试

当前 VM 中的部署路径是：

`/home/ss/catkin_ws/src/firefighting_mission`

部署前备份在：

`/home/ss/firefighting_mission.backup-before-field-fix`

## 成员接手后的第一步

在 VM 中进入 catkin 工作空间：

```bash
cd /home/ss/catkin_ws
catkin_make
source devel/setup.bash
```

如果脚本没有执行权限，修正一次：

```bash
chmod +x src/firefighting_mission/scripts/*.sh
chmod +x src/firefighting_mission/scripts/*.py
```

如果脚本出现 Windows 换行问题，修正一次：

```bash
sed -i 's/\r$//' src/firefighting_mission/scripts/*.sh
sed -i 's/\r$//' src/firefighting_mission/scripts/*.py
```

## 查看 Gazebo 场地

图形界面查看场地：

```bash
cd /home/ss/catkin_ws/src/firefighting_mission
./scripts/start_sitl.sh 4501 /home/ss/catkin_ws/src/firefighting_mission/artifacts/4501/firefighting.world true
```

种子 `4501` 是当前常用检查种子。它会生成固定场地、随机障碍、危险品贴图和人员贴图。

Gazebo 视角操作：

- 鼠标左键拖动：旋转视角。
- 鼠标中键拖动：平移视角。
- 鼠标滚轮：缩放。
- 选中对象后按 `F`：聚焦该对象。
- 按 `Esc`：取消当前选择。

## 运行任务

无界面运行：

```bash
roslaunch firefighting_mission firefighting_headless.launch seed:=4501 record:=true
```

或：

```bash
rosrun firefighting_mission run_mission.sh 4501
```

注意：这部分目前用于继续调试任务链路，不能视为已经稳定完成比赛任务。

## 主要代码入口

场地生成：

- `src/firefighting_mission/world_generator.py`
- `scripts/generate_world.py`

任务流程：

- `src/firefighting_mission/orchestration.py`
- `src/firefighting_mission/state_machine.py`
- `scripts/mission_manager_node.py`

导航和安全：

- `src/firefighting_mission/navigation.py`
- `src/firefighting_mission/safety.py`
- `scripts/navigator_node.py`
- `scripts/safety_monitor_node.py`

识别和投放：

- `src/firefighting_mission/perception.py`
- `src/firefighting_mission/payload.py`
- `scripts/target_detector_node.py`
- `scripts/payload_controller_node.py`

飞控桥接：

- `src/firefighting_mission/mavros_bridge.py`
- `scripts/mavros_bridge_node.py`

队员 C 路径和投放：

- `src/firefighting_mission/path_planner.py`
- `scripts/path_planner.py`
- `src/firefighting_mission/supply_drop.py`
- `scripts/supply_drop.py`
- `srv/DropSupply.srv`
- 详细接口见 `docs/TEAM_C_HANDOFF.zh-CN.md`

记录和评分：

- `src/firefighting_mission/scoring.py`
- `scripts/mission_recorder_node.py`

Gazebo 投放插件：

- `include/firefighting_mission/payload_plugin.hpp`
- `src/payload_plugin.cpp`

模型和贴图：

- `models/fire_iris/`
- `models/fire_payload/`
- `models/rescue_payload/`
- `models/targets/`
- `assets/templates/`

## 已验证内容

最近一次整理时的验证结果：

- Windows 主机 Python 测试：`99/99 PASS`
- VM Python 2.7 测试：`99/99 PASS`
- VM `catkin_make`：通过
- VM 路径 ROS 合约测试：通过
- VM 原生 PX4 Iris 分段点到点实飞：通过
- VM Gazebo 双通道投放物理测试：通过
- 生成世界文件检查：通过
- Gazebo 图形场景可以打开

重点场地内容已检查：

- 固定障碍物 2 和固定障碍物 4 的相对位置已修正。
- 四周透明安全网已生成。
- 危险品任务区为 400mm x 400mm，红色边框。
- 人员救助任务区为 400mm x 400mm，蓝色边框。
- 贴图资源已打包到 Gazebo 模型路径。

## 当前待办

建议后续成员按这个顺序继续：

1. 人工确认 Gazebo 场地和赛题图纸一致。
2. 由状态机向 `/fire_mission/point_goal` 发布比赛航点，并设置 `/fire_mission/aligned`。
3. 串联识别、两次高层投放、返航和降落，跑通完整自主任务。
4. 替换或适配最终 450 机体。
5. 固化比赛运行脚本和最终演示流程。

## 注意事项

- 不要直接修改 `/home/ss/PX4_Firmware` 或 `/home/ss/XTDrone`，当前任务包设计为不改上游环境。
- 不要删除 `/home/ss/firefighting_mission.backup-before-field-fix`，这是部署前备份。
- 如果更换随机种子，要重新检查随机障碍、危险品和人员位置。
- 如果把代码复制到新 VM，先确认脚本权限和换行格式。
