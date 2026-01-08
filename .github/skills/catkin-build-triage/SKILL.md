---
name: catkin-build-triage
description: ROS1(catkin_tools) 构建诊断：从 catkin build 输出定位 error/warning，给最小复现命令、对应 logs 路径、修复优先级与回滚方式
---

# catkin-build-triage

## 你要做的事
当用户贴出 `catkin build` 输出或日志片段时：
1) 先确认构建对象（工作区路径、包名、构建命令、profile）
2) 给出“最小复现命令”（只构建相关包）
3) 明确 log 文件路径（以 log_space 为准）
4) 先定位首个 error（避免级联错误误导）
5) 把信息分成三类并按优先级输出：
   - P0 必修（会导致未定义行为/运行崩溃/链接失败）
   - P1 建议修（可能影响功能/可维护性）
   - P2 可忽略（依赖缺失导致某些可选功能关闭等）
6) 每个修复建议都给：
   - 修改点（文件/函数）
   - 验证命令
   - 回滚命令（git restore / git clean）

## 默认最小复现命令模板
- 只构建单包：
  `catkin build <pkg> --no-status --summarize`
- 清理并重建：
  `catkin clean -y <pkg> && catkin build <pkg> --no-status --summarize`

## log 路径规则
- 先确认 log_space：`catkin config --summary` 或查看 `.catkin_tools/profiles/<profile>/build.yaml`
- 默认 log_space 为 `logs`
- log 文件路径模板：`<ws>/<log_space>/<pkg>/build.*.log`

## 常见 warning 分级规则
- P0：`control reaches end of non-void function`、`undefined reference`、`segfault`、编译 error
- P1：包命名不规范（含大写)、unused-but-set、未初始化
- P2：PCL “pcap/png/libusb disabled”（确认是否需要，不需要可忽略）

## 输出格式（必须遵守）
- 先给 3 行结论（是否能编过、最关键问题、下一步命令）
- 再按 P0/P1/P2 分块列出
