---
name: Bug 报告 (Bug Report)
about: 报告代码缺陷、编译错误、仿真异常或算法失效
title: "[BUG] <简短描述问题>"
labels: ["bug"]
assignees: ""
---

### 问题描述 (Bug Description)
<!-- 清晰且简明扼要地描述遇到的 Bug -->

### 复现步骤 (Steps to Reproduce)
1. 启动命令: `roslaunch fsd_launch <mission>.launch simulation:=true bag:=...`
2. 触发条件或执行步骤:
3. 观测到的异常输出或行为:

### 预期表现 (Expected Behavior)
<!-- 描述系统或算法正常情况下应该具有的表现 -->

### 运行环境 (Environment)
- **操作系统 (OS)**: Ubuntu 20.04 (x86_64 / aarch64)
- **ROS 发行版**: ROS Noetic
- **分支/Commit**: `git rev-parse HEAD`
- **相关硬件/传感器**: (如适用，如禾赛 LiDAR、Asensing 组合导航)

### 日志与堆栈 (Logs & Screenshots)
<!-- 如有终端输出、GTest 失败堆栈或 RViz 截图，请附在此处 -->
```text
<粘贴报错日志>
```

### 附加信息 (Additional Context)
<!-- 其他上下文、关联的 PR 或 Issue -->
