## 变更概述 (Summary)

<!-- 请简要描述此 Pull Request 解决的问题或引入的功能 -->

### 变更类型 (Change Type)
- [ ] `feat`: 新增功能 / 算法演进
- [ ] `fix`: 缺陷修复
- [ ] `perf`: 性能优化
- [ ] `refactor`: 代码重构（不改变外部行为）
- [ ] `docs`: 文档变更
- [ ] `test`: 测试用例补充 / 测试工具改进
- [ ] `ci`: CI/CD 配置或门禁脚本优化
- [ ] `chore`: 构建系统、依赖更新或常规维护

---

## 涉及模块 (Impacted Modules)
- [ ] `autodrive_msgs`
- [ ] `fsd_launch`
- [ ] `fsd_visualization`
- [ ] `perception_core` / `perception_ros`
- [ ] `localization_core` / `localization_ros`
- [ ] `planning_core` / `planning_ros`
- [ ] `control_core` / `control_ros`
- [ ] `vehicle_interface_core` / `vehicle_interface_ros`
- [ ] `vision_core` / `vision_ros`
- [ ] `simulation_core` / `simulation_ros`
- [ ] `scripts` / `perf_reports`

---

## 详细说明与设计背景 (Details & Rationale)

<!-- 详细说明改动逻辑、参数选择依据、架构设计决策或背景约束 -->

---

## 验证与测试 (Verification & Testing)

### 运行的验证命令及结果
- [ ] 本地单元测试与编译:
  ```bash
  catkin build <package_name>
  catkin run_tests <package_name>
  ```
- [ ] 契约门禁检查:
  ```bash
  ./scripts/check_topic_contracts.sh
  ./scripts/check_deprecation_contracts.sh
  ```
- [ ] Python 测试与格式检查:
  ```bash
  pytest
  flake8 src/ scripts/
  ```
- [ ] Rosbag 回放或仿真测试:
  ```bash
  roslaunch fsd_launch trackdrive.launch simulation:=true bag:=/path/to/bag.bag
  ```

---

## 安全性与实车影响自查 (Safety & Vehicle Checklist)
- [ ] 是否影响底盘急停（EBS）或转向极限约束？
- [ ] 接口或参数变更是否保持向后兼容（或已提供明确的 fallback 开关）？
- [ ] 是否修改了车辆几何参数或传感器外参？
- [ ] 相应文档是否已同步更新？
