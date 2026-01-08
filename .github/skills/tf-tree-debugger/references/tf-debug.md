# TF Debug Reference (ROS1)

## Quick Commands
- 生成 TF 树图：`rosrun tf view_frames`（输出 frames.pdf）
- 查看特定变换：`rosrun tf tf_echo <from> <to>`
- 查看 TF 话题：`rostopic echo -n 1 /tf`，`rostopic echo -n 1 /tf_static`
- 统计发布者：`rostopic info /tf`

## Symptom -> Likely Cause -> Fix
1) RViz Fixed Frame 报错
   - Cause: fixed_frame 不在 TF 树中
   - Fix: 改 RViz Fixed Frame 或确保对应 TF 发布
2) tf_echo 无输出
   - Cause: from/to 断链或 frame 名被 prefix/namespace 改写
   - Fix: 用 view_frames 找最近可达链路，修正 frame 名
3) 坐标漂移/跳变
   - Cause: 多节点同时发布同一变换
   - Fix: 保留唯一发布者，禁用重复发布节点
4) Extrapolation errors（时间外插）
   - Cause: 时间戳过旧/过新，或仿真时间未同步
   - Fix: 校准时间源，确保 /use_sim_time 配置一致

## Static vs Dynamic 冲突
- `static_transform_publisher` 只用于固定关系
- 动态 TF（如里程计/SLAM）应由算法节点发布
- 若二者都在发同一 child_frame，优先保留动态发布

## Frame 命名约束
- 避免混用 `map/odom/base_link`
- 保持传感器 frame_id 与 TF 树一致
- namespace/`tf_prefix` 会改写 frame 名，排查时先打印实际 frame_id

## Time & Sync
- 真车：检查系统时间与传感器时间一致
- 仿真：`/use_sim_time` 与 `/clock` 一致性
- 需要对齐传感器时，检查 message header.stamp 是否单调
