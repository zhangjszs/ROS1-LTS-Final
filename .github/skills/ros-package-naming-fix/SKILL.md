---
name: ros-package-naming-fix
description: 修复 ROS1 包名不符合规范（必须全小写+数字+_/-）：改 package.xml/CMakeLists.txt，并同步所有依赖引用与目录建议
---

# ros-package-naming-fix

## 目标
处理类似 `ros_vehicle_racingNum`（含大写）导致的 catkin 警告。

## 规则
- 合法包名：`^[a-z][a-z0-9_-]*$`
- 建议改名风格：用 `_` 分词，例如 `ros_vehicle_racing_num`

## 必改位置
1) `<pkg>/package.xml`：
   - `<name>旧名</name>` → `<name>新名</name>`
2) `<pkg>/CMakeLists.txt`：
   - `project(旧名)` → `project(新名)`

## 同步引用（必须全仓库替换/检查）
- `find_package(catkin REQUIRED COMPONENTS ... 旧名 ...)`
- `catkin_package(CATKIN_DEPENDS ... 旧名 ...)`
- `package.xml` 的 `<depend>旧名</depend>` / `<build_depend>` / `<exec_depend>`
- launch、脚本里可能写死的包名

## 给用户的命令（必须提供）
- 全仓库查引用：
  `grep -RIn "旧名" .`
- 重构后验证：
  `catkin clean -y 新名 && catkin build 新名 --no-status --summarize`
- 回滚：
  `git restore -SW .`

## 目录名建议
- 文件夹名最好与新包名一致（但以 package.xml 为准）。
