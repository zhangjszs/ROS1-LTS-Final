---
name: launch-yaml-validator
description: ROS1 launch/yaml 参数检查：frame_id(map/odom/base_link)、remap、命名空间冲突、单位一致性、默认值与动态参数覆盖
---

# launch-yaml-validator

## 适用场景
- “能跑但结果不对”：RViz 里锥桶/路径漂移、坐标不对、车辆不跟随
- launch 起不来、参数没生效、remap 错 topic

## 检查清单（按顺序）
1) frame_id / TF
   - 发布的 Marker/Path/Pose 的 header.frame_id 是否与 TF 树一致
   - map/odom/base_link 是否混用
2) remap
   - 输入点云 topic、IMU、里程计、定位输出 topic 是否对齐
3) 命名空间与节点名
   - node name 是否重复
   - namespace 是否导致参数查找失败
4) 参数文件
   - yaml 是否被正确加载（rosparam 命令）
   - 单位是否一致（米/厘米、度/弧度、秒/毫秒）
5) 运行时覆盖
   - launch 里的 param 是否覆盖了 yaml
   - dynamic_reconfigure 是否又改了值

## 必须给用户的“验证命令”
- 看参数是否生效：
  `rosparam get /<node_or_ns>`
- 看 topic 是否在发：
  `rostopic list | grep <keyword>`
  `rostopic echo -n 1 <topic>`
- 看 TF 树：
  `rosrun tf view_frames`
  `rosrun tf tf_echo <from> <to>`

## 输出格式
- 先列出“最可能的 3 个原因”
- 每个原因附：如何验证 + 如何修
