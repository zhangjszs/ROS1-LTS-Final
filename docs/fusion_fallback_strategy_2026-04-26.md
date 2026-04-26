# Fusion Fallback 策略归档

**日期**: 2026-04-26
**状态**: ✅ 已落地
**相关提交**: `2ee910e`

---

## 策略定义

| 条件 | 行为 |
|------|------|
| camera_info + TF 齐全 | 走 projection-based fusion，vision 提供颜色语义 |
| camera_info 缺失 | **禁用 legacy HFOV**，标记 `CAMERA_INFO_MISSING`，回退 LiDAR geometry color |
| TF 缺失 | 标记 `TF_MISSING`，回退 LiDAR geometry color |
| 投影出图/低置信度/无 bbox 匹配 | 对应独立 status，回退 LiDAR geometry color |

核心原则：**无标定不融合、不走 legacy HFOV、回退 LiDAR geometry。**

---

## 决策背景

1. **track.bag 无 camera_info**：确认 rosbag 中不存在 `/camera/camera_info` topic。
2. **无 camera-lidar 外参 TF**：全仓库无 TF 定义（launch、config、urdf 均缺失）。
3. **legacy HFOV 质量差**：角度匹配（5deg HFOV）无标定支撑，颜色一致性仅 0.76，且 cross-track 错误占比高。
4. **LiDAR geometry color 更稳定**：基于 cone 在车辆坐标系中的 y 位置（右蓝左黄/红），在单圈自洽场景下表现可靠。

---

## 配置改动

`src/perception_ros/config/lidar_base.yaml` 新增 `fusion` 配置块：

```yaml
fusion:
  enabled: true
  projection:
    require_camera_info: true
    require_tf: true
    fallback_to_lidar_color: true
  legacy_hfov_fallback:
    enable: false          # 明确禁用
```

旧 `vision_inject` 块保留并标注 deprecated，向后兼容无 `fusion/enabled` 的场景。

---

## 验证结果

**测试命令**:
```bash
bash scripts/run_fusion_sanity_test.sh ~/rosbag/track.bag
```

**输出**:
- 50 frames / 694 cones
- Association status: **100% CAMERA_INFO_MISSING**
- Color source: **100% lidar_geometry**
- Legacy HFOV: **0% (PASS)**

验证结论：配置生效，camera_info 缺失时正确回退到 LiDAR geometry color，legacy HFOV 未触发。

---

## 测试脚本

| 脚本 | 作用 |
|------|------|
| `scripts/run_fusion_sanity_test.sh` | 编排测试：启动 replay → 发布假 vision → 运行分析器 |
| `scripts/fake_vision_publisher.py` | 按 point cloud 时间戳同步发布空 `HUAT_VisionDetections`，确保 message_filters sync pair 形成 |
| `scripts/analyze_fusion_sanity.py` | 订阅 `/perception/fusion/detections`，统计 50 frames 的 association status 和 color source 分布 |

---

## 后续工作（Backlog）

| 优先级 | 事项 | 触发条件 |
|--------|------|----------|
| P1 | projection-based 颜色质量评估 | 录制包含 camera_info + TF 的新 bag |
| P2 | legacy HFOV diagnostics-only 模式 | 如需在赛场上收集 legacy HFOV vs projection 的差异数据 |
| P2 | CI 单元测试（config loading 无 live ROS） | 长期工程效率提升 |

---

## 相关文件

- `src/perception_ros/config/lidar_base.yaml`
- `src/perception_ros/src/lidar_cluster_ros.cpp`（读取配置逻辑，未修改）
- `scripts/run_fusion_sanity_test.sh`
- `scripts/fake_vision_publisher.py`
- `scripts/analyze_fusion_sanity.py`
