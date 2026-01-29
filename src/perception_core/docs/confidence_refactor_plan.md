# LiDAR聚类置信度评估重构方案

## 1. 现状分析

### 1.1 当前实现问题
**位置**: `src/perception_core/src/utility.cpp:291-331`

```cpp
double lidar_cluster::getConfidence(PointType max, PointType min, Eigen::Vector4f centroid)
```

**主要问题**：
- 仅基于几何尺寸（长宽高、面积）的简单规则
- 固定权重的扣分机制，缺乏灵活性
- 未利用点云密度、强度反射、形状分布等关键特征
- 硬编码阈值（confidence > 0.5），无法适应不同场景
- 对小目标（锥桶）检测鲁棒性不足

### 1.2 欧式聚类现状
- **固定阈值聚类** (`EucClusterMethod`): 0.3m固定距离
- **自适应聚类** (`EuclideanAdaptiveClusterMethod`): 根据距离分段调整阈值
- 聚类参数：MinClusterSize=2, MaxClusterSize=50

---

## 2. 重构方案：传统方法 + 机器学习混合架构

### 2.1 整体架构

```
点云聚类 → 多特征提取 → 传统规则过滤 → ML分类器 → 置信度输出
   ↓           ↓              ↓              ↓           ↓
 欧式聚类   几何+物理特征   快速筛选      精细判别    最终决策
```

### 2.2 核心设计原则
1. **渐进式增强**：保持现有流程，逐步添加新特征
2. **轻量级优先**：优先使用传统方法，ML作为辅助
3. **可配置化**：所有阈值和权重可通过配置文件调整
4. **向后兼容**：保留原有接口，新功能可选启用

---

## 3. 阶段1：多特征提取与规则增强

### 3.1 新增特征类
**文件**: `src/perception_core/include/perception_core/cluster_features.hpp`

```cpp
struct ClusterFeatures {
    // 几何特征
    double length, width, height;
    double area, volume;
    double aspect_ratio;  // height/(length+width)

    // 点云特征
    int point_count;
    double point_density;  // points/volume
    double distance_to_sensor;

    // 强度特征
    double intensity_mean;
    double intensity_std;
    double intensity_max;

    // 形状特征
    double shape_elongation;  // PCA主成分比
    double shape_planarity;
    double verticality_score; // 垂直度评分

    // 位置特征
    Eigen::Vector3f centroid;
    double ground_height;  // 离地高度
};
```

### 3.2 特征提取器
**文件**: `src/perception_core/src/cluster_feature_extractor.cpp`

```cpp
class ClusterFeatureExtractor {
public:
    ClusterFeatures extract(const pcl::PointCloud<PointType>::Ptr& cluster);

private:
    void computeGeometricFeatures(const pcl::PointCloud<PointType>::Ptr& cluster,
                                   ClusterFeatures& features);
    void computeIntensityFeatures(const pcl::PointCloud<PointType>::Ptr& cluster,
                                   ClusterFeatures& features);
    void computeShapeFeatures(const pcl::PointCloud<PointType>::Ptr& cluster,
                               ClusterFeatures& features);
};
```

**关键实现**：
- **点云密度**: `point_count / volume`，近距离目标应有高密度
- **强度统计**: 锥桶反光贴纸通常有高反射强度
- **形状分析**: PCA分析点云分布，锥桶呈细高柱状
- **垂直度**: 检查点云是否垂直于地面

### 3.3 增强的置信度评分器
**文件**: `src/perception_core/src/confidence_scorer.cpp`

```cpp
class ConfidenceScorer {
public:
    struct Config {
        // 尺寸约束
        double min_height = 0.15;
        double max_height = 0.5;
        double min_area = 0.01;
        double max_area = 0.15;

        // 形状约束
        double min_aspect_ratio = 1.5;  // 锥桶应该是细高的
        double min_verticality = 0.8;

        // 密度约束
        double min_density_near = 50.0;  // <5m
        double min_density_far = 10.0;   // >5m

        // 强度约束
        double min_intensity_mean = 30.0;

        // 权重配置
        double weight_size = 0.3;
        double weight_shape = 0.25;
        double weight_density = 0.2;
        double weight_intensity = 0.15;
        double weight_position = 0.1;
    };

    double computeConfidence(const ClusterFeatures& features, const Config& config);

private:
    double scoreSizeConstraints(const ClusterFeatures& f, const Config& c);
    double scoreShapeConstraints(const ClusterFeatures& f, const Config& c);
    double scoreDensityConstraints(const ClusterFeatures& f, const Config& c);
    double scoreIntensityConstraints(const ClusterFeatures& f, const Config& c);
    double scorePositionConstraints(const ClusterFeatures& f, const Config& c);
};
```

**评分逻辑**：
```cpp
confidence = weight_size * size_score
           + weight_shape * shape_score
           + weight_density * density_score
           + weight_intensity * intensity_score
           + weight_position * position_score
```

---

## 4. 阶段2：几何模型拟合

### 4.1 锥桶模型拟合器
**文件**: `src/perception_core/src/cone_model_fitter.cpp`

```cpp
class ConeModelFitter {
public:
    struct FitResult {
        bool is_valid;
        double fit_error;
        double base_radius;
        double height;
        Eigen::Vector3f apex_position;
    };

    FitResult fitConeModel(const pcl::PointCloud<PointType>::Ptr& cluster);

private:
    // 底部圆形拟合
    bool fitCircleRANSAC(const std::vector<Eigen::Vector2f>& points_2d,
                         Eigen::Vector2f& center, double& radius);

    // 高度分布检查
    bool checkHeightDistribution(const pcl::PointCloud<PointType>::Ptr& cluster);
};
```

**拟合策略**：
1. 将点云投影到地面平面（XY平面）
2. 使用RANSAC拟合底部圆形
3. 检查高度方向点云分布是否收拢
4. 计算拟合误差作为置信度调整因子

### 4.2 集成到置信度评分
```cpp
double confidence = base_confidence;

// 如果启用模型拟合
if (config.enable_model_fitting && features.point_count > 10) {
    ConeModelFitter::FitResult fit = fitter.fitConeModel(cluster);
    if (fit.is_valid) {
        // 拟合误差越小，置信度越高
        double fit_bonus = (1.0 - fit.fit_error) * 0.2;
        confidence += fit_bonus;
    } else {
        confidence -= 0.15;  // 拟合失败降低置信度
    }
}
```

---

## 5. 阶段3：轻量级机器学习分类器

### 5.1 特征向量设计
**维度**: 15维特征向量

```cpp
std::vector<float> feature_vector = {
    features.length,
    features.width,
    features.height,
    features.aspect_ratio,
    features.point_density,
    features.intensity_mean,
    features.intensity_std,
    features.shape_elongation,
    features.verticality_score,
    features.distance_to_sensor,
    features.ground_height,
    features.area,
    features.volume,
    features.point_count,
    fit_result.fit_error  // 如果启用模型拟合
};
```

### 5.2 分类器选择
**推荐方案**: 随机森林 (Random Forest)

**优势**：
- 训练快速，推理高效（无需GPU）
- 对特征尺度不敏感
- 可解释性强（特征重要性分析）
- 鲁棒性好，不易过拟合

**备选方案**:
- SVM（支持向量机）：适合小样本
- XGBoost：更高精度，但计算开销稍大

### 5.3 实现框架
**文件**: `src/perception_core/src/ml_classifier.cpp`

```cpp
class MLClassifier {
public:
    struct Prediction {
        bool is_cone;
        double confidence;
        std::vector<double> class_probabilities;
    };

    bool loadModel(const std::string& model_path);
    Prediction predict(const std::vector<float>& features);

private:
    // 使用轻量级推理库（如ONNX Runtime或自定义实现）
    void* model_handle_;
};
```

### 5.4 混合决策策略
```cpp
double final_confidence;

if (config.enable_ml_classifier) {
    // 传统方法置信度
    double rule_confidence = scorer.computeConfidence(features, config);

    // ML分类器置信度
    MLClassifier::Prediction pred = classifier.predict(feature_vector);
    double ml_confidence = pred.confidence;

    // 加权融合
    final_confidence = config.rule_weight * rule_confidence
                     + config.ml_weight * ml_confidence;

    // 如果两者差异过大，降低置信度（不确定性惩罚）
    double disagreement = std::abs(rule_confidence - ml_confidence);
    if (disagreement > 0.3) {
        final_confidence -= 0.1;
    }
} else {
    final_confidence = scorer.computeConfidence(features, config);
}
```

---

## 6. 配置文件设计

**文件**: `src/perception_ros/config/confidence_config.yaml`

```yaml
confidence_scorer:
  # 特征提取
  enable_intensity_features: true
  enable_shape_features: true
  enable_density_features: true

  # 尺寸约束
  size_constraints:
    min_height: 0.15
    max_height: 0.5
    min_area: 0.01
    max_area: 0.15

  # 形状约束
  shape_constraints:
    min_aspect_ratio: 1.5
    min_verticality: 0.8

  # 密度约束
  density_constraints:
    min_density_near: 50.0
    min_density_far: 10.0
    distance_threshold: 5.0

  # 强度约束
  intensity_constraints:
    min_mean: 30.0
    min_max: 50.0

  # 权重配置
  weights:
    size: 0.3
    shape: 0.25
    density: 0.2
    intensity: 0.15
    position: 0.1

# 几何模型拟合
model_fitting:
  enable: true
  min_points_for_fitting: 10
  ransac_iterations: 100
  ransac_threshold: 0.05
  fit_bonus_weight: 0.2

# 机器学习分类器
ml_classifier:
  enable: false  # 初期禁用，训练完成后启用
  model_path: "$(find perception_ros)/models/cone_classifier.onnx"
  rule_weight: 0.4
  ml_weight: 0.6
  disagreement_penalty: 0.1

# 最终决策
decision:
  confidence_threshold: 0.5
  enable_adaptive_threshold: true  # 根据距离自适应调整
  near_threshold: 0.6  # <5m
  far_threshold: 0.4   # >10m
```

---

## 7. 渐进式实施路径

### Phase 1: 特征提取增强 (1-2周)
**目标**: 替换现有getConfidence，引入多特征评分

**任务清单**:
1. ✅ 创建 `ClusterFeatures` 结构体
2. ✅ 实现 `ClusterFeatureExtractor` 类
3. ✅ 实现 `ConfidenceScorer` 类
4. ✅ 修改 `clusterMethod16/32` 集成新评分器
5. ✅ 添加配置文件支持
6. ✅ 单元测试：验证特征提取正确性
7. ✅ 实车测试：对比新旧方法检测率

**验收标准**:
- 锥桶检测召回率 > 90%
- 误检率 < 5%
- 处理时间增加 < 20%

### Phase 2: 几何模型拟合 (1周)
**目标**: 添加锥桶模型拟合作为置信度调整

**任务清单**:
1. ✅ 实现 `ConeModelFitter` 类
2. ✅ 集成到置信度评分流程
3. ✅ 配置文件添加模型拟合参数
4. ✅ 性能优化：仅对高疑似簇进行拟合
5. ✅ 实车测试：评估拟合效果

**验收标准**:
- 误检率降低至 < 3%
- 处理时间增加 < 10%

### Phase 3: 数据收集与标注 (2-3周)
**目标**: 准备ML训练数据

**任务清单**:
1. ✅ 实现数据记录功能：保存聚类特征和标签
2. ✅ 收集多场景数据（赛道、加速、绕桩）
3. ✅ 人工标注：标记真实锥桶和误检
4. ✅ 数据增强：处理类别不平衡
5. ✅ 数据集划分：训练/验证/测试 = 70%/15%/15%

**目标数据量**:
- 真实锥桶样本: 5000+
- 负样本（非锥桶）: 3000+

### Phase 4: ML模型训练 (1周)
**目标**: 训练轻量级分类器

**任务清单**:
1. ✅ 特征工程：归一化、特征选择
2. ✅ 训练随机森林模型
3. ✅ 超参数调优：网格搜索
4. ✅ 模型评估：准确率、召回率、F1-score
5. ✅ 模型导出：转换为ONNX格式
6. ✅ 特征重要性分析

**验收标准**:
- 验证集准确率 > 95%
- 召回率 > 92%
- 推理时间 < 1ms/样本

### Phase 5: ML集成与部署 (1周)
**目标**: 将ML分类器集成到系统

**任务清单**:
1. ✅ 实现 `MLClassifier` 类
2. ✅ 集成ONNX Runtime推理
3. ✅ 实现混合决策策略
4. ✅ 配置文件添加ML参数
5. ✅ 实车测试：全流程验证
6. ✅ 性能优化：批量推理

**验收标准**:
- 检测准确率 > 95%
- 端到端处理时间 < 50ms
- 系统稳定性：无崩溃

### Phase 6: 优化与调优 (持续)
**目标**: 持续改进系统性能

**任务清单**:
1. ✅ A/B测试：对比不同配置
2. ✅ 自适应阈值调整
3. ✅ 在线学习：收集边缘案例
4. ✅ 模型更新：定期重训练
5. ✅ 性能监控：建立指标看板

---

## 8. 代码结构

```
src/perception_core/
├── include/perception_core/
│   ├── cluster_features.hpp          # 特征结构定义
│   ├── cluster_feature_extractor.hpp # 特征提取器
│   ├── confidence_scorer.hpp         # 置信度评分器
│   ├── cone_model_fitter.hpp         # 锥桶模型拟合
│   └── ml_classifier.hpp             # ML分类器
├── src/
│   ├── cluster_feature_extractor.cpp
│   ├── confidence_scorer.cpp
│   ├── cone_model_fitter.cpp
│   ├── ml_classifier.cpp
│   └── utility.cpp                   # 修改集成新功能
└── docs/
    └── confidence_refactor_plan.md   # 本文档

src/perception_ros/
├── config/
│   └── confidence_config.yaml        # 配置文件
├── models/
│   └── cone_classifier.onnx          # 训练好的模型
└── scripts/
    ├── collect_training_data.py      # 数据收集脚本
    └── train_classifier.py           # 模型训练脚本
```

---

## 9. 性能预期

| 指标 | 当前 | Phase 1 | Phase 2 | Phase 5 |
|------|------|---------|---------|---------|
| 召回率 | 85% | 90% | 92% | 95% |
| 精确率 | 90% | 93% | 95% | 97% |
| 误检率 | 10% | 7% | 5% | 3% |
| 处理时间 | 30ms | 35ms | 38ms | 45ms |

---

## 10. 风险与缓解

### 10.1 性能风险
**风险**: 特征提取和ML推理增加计算开销
**缓解**:
- 仅对候选簇进行精细处理
- 使用轻量级模型（随机森林）
- 批量推理优化

### 10.2 数据质量风险
**风险**: 训练数据不足或标注错误
**缓解**:
- 多场景数据收集
- 双人交叉标注
- 数据增强技术

### 10.3 泛化性风险
**风险**: 模型在新场景表现不佳
**缓解**:
- 传统方法作为基线保障
- 混合决策策略
- 定期模型更新

---

## 11. 后续扩展方向

1. **多目标类别支持**: 扩展到车辆、行人检测
2. **时序信息融合**: 利用历史帧信息提升稳定性
3. **深度学习升级**: 引入PointNet进行端到端检测
4. **传感器融合**: 结合相机信息提升小目标检测

---

## 12. 参考资料

- Autoware: https://github.com/autowarefoundation/autoware
- PCL Documentation: https://pointclouds.org/
- ONNX Runtime: https://onnxruntime.ai/
- Random Forest: scikit-learn.org/stable/modules/ensemble.html
