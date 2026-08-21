# Firefighting Mission SITL

This repository contains a ROS Melodic, Gazebo 9, PX4 SITL, and XTDrone-compatible simulation package for the intelligent low-altitude firefighting competition task.

中文说明为主，请优先阅读以下文档：

- [README.zh-CN.md](README.zh-CN.md)：中文运行说明。
- [docs/TEAM_HANDOFF.zh-CN.md](docs/TEAM_HANDOFF.zh-CN.md)：小组成员接手指南，包括环境、运行步骤、当前可用能力和待办事项。
- [docs/CODE_INVENTORY_AND_PROGRESS.zh-CN.md](docs/CODE_INVENTORY_AND_PROGRESS.zh-CN.md)：代码清单、模块用途、验证记录和当前进展。

## Current Status

Completed:

- ROS package structure, messages, launch files, and scripts.
- Gazebo firefighting field layout based on the competition reference drawing.
- Fixed obstacles, random obstacle candidates, hazard zones, rescue zone, target textures, and transparent safety net.
- Mission orchestration, navigation, safety monitoring, perception, payload release, recording, and scoring modules.
- Host and VM test validation recorded in the Chinese handoff documents.

Not completed yet:

- The full autonomous mission flight is not fully validated.
- PX4/Gazebo physical takeoff behavior still needs further debugging.
- Final 450-frame vehicle adaptation is not yet hardware-validated.

## Recommended Branch

Use the current default branch:

```bash
git checkout feature/firefighting-sitl
```

