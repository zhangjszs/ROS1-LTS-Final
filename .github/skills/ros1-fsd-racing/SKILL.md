---
name: ros1-fsd-racing
description: ROS1/catkin_tools 车队项目的总控/路由技能。用于识别问题类型并切换到最合适的细分技能（构建诊断、包命名、launch/yaml 检查、TF 排查、点云性能计时等）。
---

# ros1-fsd-racing

## 目标
快速识别用户问题类型，并路由到最合适的细分技能执行具体流程。

## 路由规则（按优先级匹配）
1) 构建/编译问题 -> 使用 `catkin-build-triage`
2) 包命名不规范（含大写） -> 使用 `ros-package-naming-fix`
3) launch/yaml 参数或 remap 异常 -> 使用 `launch-yaml-validator`
4) TF 树断链/Fixed Frame/坐标漂移 -> 使用 `tf-tree-debugger`
5) 点云性能计时/10Hz -> 使用 `pointcloud-perf-profiler`

## 输出格式
- 先一句话说明识别到的问题类型
- 明确指向要使用的技能名称
- 若信息不足，列出 2-3 个最关键补充信息

## 约束
- 不执行危险命令（rm -rf / curl|sh / sudo）除非用户明确要求并解释风险
- 修改前先说明影响面与回滚命令
