# 代码清单与当前进展

本文档用于说明当前仓库中各类代码的用途、已经完成的工作、验证结果和剩余风险。

## 项目定位

本仓库是第十五届上海市工程实践与创新能力大赛“智能低空消防”赛项的 ROS/Gazebo/PX4 仿真任务包。

目标是在 Ubuntu 18.04、ROS Melodic、Gazebo 9、PX4 SITL 和 XTDrone 相关环境中，搭建 4m x 4m x 3m 低空消防场地，并支持无人机执行以下流程：

- 从直径 500mm 起飞区起飞。
- 穿越固定障碍和随机障碍。
- 识别危险品任务区，并投放消防物资。
- 识别人员救助任务区，并投放救援物资。
- 返回起飞区降落。
- 记录轨迹、事件、分数和仿真证据。

## 代码结构

### ROS 包元数据

- `package.xml`：ROS 包依赖声明。
- `CMakeLists.txt`：catkin 编译、安装、消息、插件和测试配置。
- `setup.py`：Python 模块安装入口。
- `README.zh-CN.md`：中文运行说明。

### 配置文件

- `config/mission.yaml`：任务参数、判定阈值、投放误差、时间限制等配置。
- `config/scenarios.yaml`：种子场景、随机障碍、危险品和人员位置配置。

### Launch 启动文件

- `launch/firefighting.launch`：带 Gazebo 图形界面的仿真启动入口。
- `launch/firefighting_headless.launch`：无界面仿真启动入口，适合自动测试和记录。

### 脚本入口

- `scripts/start_sitl.sh`：生成指定种子世界，并启动 Gazebo/PX4/MAVROS。
- `scripts/run_mission.sh`：一键运行任务并保存产物。
- `scripts/generate_world.py`：生成指定种子的 Gazebo 世界文件。
- `scripts/mission_manager_node.py`：ROS 任务管理节点入口。
- `scripts/mission_recorder_node.py`：ROS 记录节点入口。
- `scripts/navigator_node.py`：ROS 导航节点入口。
- `scripts/safety_monitor_node.py`：ROS 安全监测节点入口。
- `scripts/target_detector_node.py`：ROS 目标识别节点入口。
- `scripts/payload_controller_node.py`：ROS 投放控制节点入口。
- `scripts/mavros_bridge_node.py`：MAVROS 与现有 XTDrone 风格控制命令之间的桥接节点入口。
- `scripts/path_planner.py`：分段固定高度路径 ROS 节点，输出内部位置设定点和阶段状态。
- `scripts/supply_drop.py`：带对准、速度、高度和重复投放检查的高层投放服务。

### Python 核心逻辑

- `src/firefighting_mission/world_generator.py`：根据赛题尺寸生成场地、固定障碍、随机障碍、危险品区、人员区和安全网。
- `src/firefighting_mission/state_machine.py`：任务阶段状态机。
- `src/firefighting_mission/orchestration.py`：任务管理、阶段切换、完成判定和关闭流程。
- `src/firefighting_mission/navigation.py`：航点导航、避障状态和到点判定。
- `src/firefighting_mission/safety.py`：边界、碰撞和安全状态监测。
- `src/firefighting_mission/perception.py`：危险品与人员目标检测结果处理。
- `src/firefighting_mission/payload.py`：消防物资和救援物资投放控制。
- `src/firefighting_mission/scoring.py`：成绩计算、失败原因和原子化 `score.json` 输出。
- `src/firefighting_mission/mavros_bridge.py`：将任务包控制命令转为 MAVROS 解锁、模式和降落接口。
- `src/firefighting_mission/path_planner.py`：`CLIMB/CRUISE/DESCEND/REACHED` 分段路径逻辑。
- `src/firefighting_mission/supply_drop.py`：双通道投放安全门控与失败重试语义。
- `src/firefighting_mission/__init__.py`：Python 包初始化。

### C++ Gazebo 插件

- `include/firefighting_mission/payload_plugin.hpp`：投放插件头文件。
- `src/payload_plugin.cpp`：Gazebo 物资投放插件实现，用于模拟双物资释放和落点结果。

### Gazebo 模型与资源

- `worlds/firefighting.world.in`：Gazebo 世界模板。
- `models/fire_iris/`：带任务载荷接口的无人机模型。
- `models/fire_payload/`：消防物资模型。
- `models/rescue_payload/`：救援物资模型。
- `models/payload_test/`：投放插件测试模型。
- `models/targets/`：危险品、干扰项、人员图像的 Gazebo 材质和贴图。
- `assets/templates/`：危险品、干扰项、人员图像源模板。

### ROS 消息

- `msg/TargetDetection.msg`：目标检测消息。
- `msg/DropResult.msg`：投放结果消息。
- `msg/MissionEvent.msg`：任务事件消息。
- `srv/DropSupply.srv`：消防物资通道 1、救援物资通道 2 的投放服务。

### 测试代码

- `test/test_world_generator.py`：场地尺寸、障碍位置、随机区、任务区贴图和安全网测试。
- `test/test_orchestration.py`：任务编排、MAVROS 接口、记录关闭和启动参数测试。
- `test/test_state_machine.py`：任务阶段状态机测试。
- `test/test_navigation.py` 与 `test/navigation.test`：导航逻辑和 ROS 测试入口。
- `test/test_safety.py`：安全边界、碰撞和净空测试。
- `test/test_perception.py`：目标识别逻辑测试。
- `test/test_payload.py` 与 `test/payload_drop.test`：投放逻辑和 Gazebo 插件测试入口。
- `test/test_scoring.py`：成绩计算和 `score.json` 输出测试。
- `test/test_package_metadata.py`：包元数据、脚本安装和资源安装测试。
- `test/test_path_planner.py` 与 `test/path_planner.test`：分段路径逻辑和 ROS 接口测试。
- `test/test_supply_drop.py` 与 `test/payload_drop.test`：高层安全门控和 Gazebo 物理分离测试。

### 设计和实施文档

- `docs/superpowers/specs/2026-08-20-firefighting-sitl-design.md`：消防 SITL 任务设计说明。
- `docs/superpowers/plans/2026-08-20-firefighting-sitl.md`：消防 SITL 任务实施计划。
- `docs/superpowers/specs/2026-08-20-field-layout-correction-design.md`：赛题场地修正设计说明。
- `docs/superpowers/plans/2026-08-20-field-layout-correction.md`：赛题场地修正实施计划。

## 已完成进展

### 任务仿真框架

已完成 ROS 包结构、消息、launch、脚本入口、任务状态机、导航、安全、识别、投放、记录、评分等基础模块。

当前任务链路已经能启动 PX4/Gazebo/MAVROS，并进入任务控制阶段；此前完整任务自动飞行仍存在机体物理起飞问题，已暂停完整任务运行，优先完成场地可视化和赛题环境搭建。

### 场地建模

已按赛题参考图修正场地：

- 场地范围：4m x 4m x 3m。
- 起飞区：直径 500mm。
- 固定障碍物 1、2、3、4 已按图纸位置和尺寸建模。
- 固定障碍物 2 调整为 45 度斜向布置，下方对应固定障碍物 4。
- 随机圆柱障碍物提供两个候选位置，并保证通行宽度约束。
- 危险品任务区为 400mm x 400mm，红色边框，两个区域中一个为正确危险品图像，另一个为干扰图像。
- 人员救助任务区为 400mm x 400mm，蓝色边框，在候选 1/2/3 中按种子选择位置。
- 四周已加入透明 3m 高安全网，无顶棚。

### 可视化资源

已加入并打包以下 Gazebo 贴图：

- 易燃危险品标识。
- 易爆危险品标识。
- 有毒危险品标识。
- 干扰项图像。
- 人员头像图像。

种子 `4501` 的已知生成结果中，危险品材质包含 `Toxic`，另一个危险品区为 `Distractor`，人员区为 `Person`。

### VM 部署状态

代码已同步到虚拟机：

- VM 用户：`ss`
- VM 路径：`/home/ss/catkin_ws/src/firefighting_mission`
- 部署前备份：`/home/ss/firefighting_mission.backup-before-field-fix`

已处理 Windows 到 Linux 同步后的脚本权限和换行问题。

### 当前 Gazebo 状态

已能在 VM 中打开新的 Gazebo 图形仿真场景，用于查看修正后的消防场地。用户已看到 Gazebo 界面，并正在检查视角和场地布局。

### 队员 C：路径规划与物资投放

已完成固定高度分段策略，默认先升至 `2.30 m`，再水平飞至目标上方，最后下降到任务高度。路径设定点经 `competition_main.py` 接入 OFFBOARD 控制，避免多个节点同时向 MAVROS 发布位置设定点。

已完成双通道 ROS 投放服务与 Gazebo 模型分离插件。高层服务检查对准、水平速度不超过 `0.10 m/s`、高度位于 `1.15-1.45 m`、通道未重复使用；低层服务执行舱门动作、关节分离和载荷重力恢复。完整接口见 `docs/TEAM_C_HANDOFF.zh-CN.md`。

## 验证记录

已完成的验证：

- Windows 主机 Python 测试：`99/99 PASS`
- VM Python 2.7 测试：`99/99 PASS`
- VM `catkin_make`：通过
- VM Gazebo 物理投放服务测试：通过
- VM 路径 ROS 合约测试：通过
- VM 原生 PX4 Iris 分段点到点实飞：通过；阶段顺序为 `CLIMB -> CRUISE -> DESCEND -> REACHED`
- 生成世界文件检查：通过
- 场地关键内容检查：通过
  - `fixed_obstacle_2`
  - `fixed_obstacle_4`
  - 四个 `safety_net_*`
  - 危险品和人员材质

## 当前未完成事项

- PX4/MAVROS 的 OFFBOARD 起飞与队员 C 分段点到点飞行已经通过；完整自主任务仍需由状态机生成比赛航点并串联识别、两次投放、返航和降落。
- 当前阶段重点已切换为：先完成并人工确认 Gazebo 场地环境。
- GitHub 上传已完成，仓库为 `https://github.com/NOSTALGIA1000/firefighting-mission-sitl`，当前分支为 `feature/firefighting-sitl`。

## GitHub 发布状态

当前分支 `feature/firefighting-sitl` 已推送到 GitHub，并保留当前提交历史。当前提交清楚区分了：

- SITL 任务框架。
- 任务编排与记录。
- 场地设计修正。
- 场地贴图和目标区。
- 安全网与载荷尺寸约束。

仓库名：

`firefighting-mission-sitl`

远程分支：

`feature/firefighting-sitl`
