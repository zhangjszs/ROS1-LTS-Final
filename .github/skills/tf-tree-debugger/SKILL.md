---
name: tf-tree-debugger
description: ROS1 TF 树诊断与修复（frame 缺失/断链/循环/固定帧错误/时间不同步）。当 RViz 报 Fixed Frame 错误、tf_echo 无输出、坐标漂移或传感器对齐异常时使用。
---

# tf-tree-debugger

## 目标
- 快速定位 TF 树断链、frame 命名不一致、静态/动态冲突、时间戳问题
- 输出可执行的验证命令与最小修复步骤

## 工作流
1) 收集最小证据
   - `rostopic list | rg "/tf"`
   - `rosnode list | rg tf`
   - `rosrun tf view_frames`（生成 frames.pdf）
   - `rosrun tf tf_echo <from> <to>`
2) 锁定异常类型并给出验证/修复
   - 断链：`tf_echo` 无输出或树分裂
   - 固定帧错误：RViz Fixed Frame 报错/红色
   - 坐标漂移：frame_id 不匹配或静态/动态发布冲突
   - 时间问题：TF 过期、外插、消息时间不单调
3) 给出最小修复建议
   - 指明修改点（发布节点/launch/参数）
   - 给出验证命令与回滚方式

## 核心检查项（优先顺序）
1) frame 命名一致性：`map/odom/base_link` 是否混用
2) 唯一发布者：同一子树不允许多节点同时发布
3) 静态/动态冲突：`static_transform_publisher` 是否覆盖动态 TF
4) 时间戳：TF 与传感器消息时间是否同步（仿真/真车）
5) 命名空间：`tf_prefix`/namespace 是否导致 frame 名被重写

## 输出格式
- 先列出 3 个最可能原因（按概率排序）
- 每个原因附：如何验证 + 如何修复 + 验证命令

## 参考资料
- 需要更细的排查表或命令示例时，读取 `references/tf-debug.md`
