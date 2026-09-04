# Git仓库文件清理分析报告

## 执行日期
2026-03-09

## 当前仓库状态
- **Git跟踪文件总数**: 569个
- **JSON文件**: 4个（已从80个清理）
- **大文件(>100MB)**: 4个模型文件

---

## ✅ 已完成的清理

### 1. 性能数据文件（已完成）
- **移除**: 242个文件
- **类型**: `perf_reports/data/` 下的生成数据
- **本地保留**: ✅ 是
- **基线保留**: `perf_reports/baselines/` 仍在Git中

---

## 📋 建议处理清单

### 🔴 高优先级 - 建议移除

| 文件/目录 | 大小 | 原因 | 操作建议 |
|-----------|------|------|----------|
| `src/vision_ros/models/exp26l_gpu/best.onnx` | ~99MB | 大模型文件 | 使用Git LFS或外部存储 |
| `src/vision_ros/models/exp26l_gpu/last.onnx` | ~99MB | 大模型文件 | 使用Git LFS或外部存储 |
| `src/vision_ros/models/exp26l_gpu/best.pt` | ~53MB | 大模型文件 | 使用Git LFS或外部存储 |
| `src/vision_ros/models/exp26l_gpu/last.pt` | ~53MB | 大模型文件 | 使用Git LFS或外部存储 |
| `perf_reports/reports/*.png` | ~1.4MB | 生成的报告图片 | 可选：保留或移除 |

### 🟡 中优先级 - 可选处理

| 文件/目录 | 原因 | 建议 |
|-----------|------|------|
| `src/fsd_visualization/meshes/vehicle/*.stl` | 大网格文件(11MB+2.7MB) | 保留（项目必需） |
| `benchmark_results.csv` | 生成的基准测试结果 | 可选：添加到.gitignore |
| `perf_reports/gt_example.csv` | 示例数据 | 保留（文档用） |

### 🟢 低优先级 - 保留

| 文件/目录 | 原因 |
|-----------|------|
| `perf_reports/baselines/*.json` | 性能基线配置（必需） |
| `perf_reports/schemas/perf_data_schema.json` | 数据模式定义（必需） |
| `src/competition_guide/attachments/*.csv` | 比赛数据（必需） |
| `src/fsd_visualization/meshes/cone/materials/textures/*.png` | 纹理资源（必需） |

---

## 🔧 具体操作建议

### 1. 处理大模型文件（推荐）

这些模型文件总计约300MB，会显著增加仓库大小：

```bash
# 方案A：使用Git LFS（推荐）
git lfs track "*.onnx"
git lfs track "*.pt"
git add .gitattributes
git commit -m "chore: track model files with Git LFS"

# 方案B：从Git移除，保留本地
# 在.gitignore中添加：
# src/vision_ros/models/**/*.onnx
# src/vision_ros/models/**/*.pt
```

### 2. 添加更多忽略规则

建议更新 `.gitignore`：

```gitignore
# Model files - use Git LFS or external storage
# *.onnx
# *.pt

# Generated benchmark results
benchmark_results.csv

# Generated report images (optional)
# perf_reports/reports/*.png
```

### 3. 清理命令（如需执行）

```bash
# 从Git移除模型文件（保留本地）
git rm --cached src/vision_ros/models/exp26l_gpu/best.onnx
git rm --cached src/vision_ros/models/exp26l_gpu/last.onnx
git rm --cached src/vision_ros/models/exp26l_gpu/best.pt
git rm --cached src/vision_ros/models/exp26l_gpu/last.pt

# 提交更改
git commit -m "chore: remove large model files from git tracking

- Remove 4 model files (~300MB total)
- Models should be stored in Git LFS or external storage
- Local files are preserved"
```

---

## 📊 文件统计详情

### 按类型分布
| 类型 | 数量 | 说明 |
|------|------|------|
| C++文件 | 213 | 源代码 |
| Python文件 | 29 | 脚本和节点 |
| Markdown文件 | 92 | 文档 |
| YAML/YML文件 | 49 | 配置和Launch |
| Launch文件 | 44 | ROS启动文件 |
| CMake文件 | 22 | 构建配置 |
| JSON文件 | 4 | 基线和模式 |
| PNG图片 | 5 | 报告和纹理 |

### 大文件列表(>100KB)
| 文件 | 大小 | 处理建议 |
|------|------|----------|
| `src/vision_ros/models/exp26l_gpu/last.onnx` | 99MB | Git LFS |
| `src/vision_ros/models/exp26l_gpu/best.onnx` | 99MB | Git LFS |
| `src/vision_ros/models/exp26l_gpu/last.pt` | 53MB | Git LFS |
| `src/vision_ros/models/exp26l_gpu/best.pt` | 53MB | Git LFS |
| `src/fsd_visualization/meshes/vehicle/whole_car.stl` | 11MB | 保留 |
| `src/fsd_visualization/meshes/vehicle/wheel.stl` | 2.7MB | 保留 |
| `src/fsd_visualization/meshes/cone/materials/textures/Construction_Cone_Diffuse.png` | 468KB | 保留 |

---

## ✅ 当前状态检查

### 已正确忽略的文件类型
- ✅ `build/`, `devel/`, `install/` - 构建目录
- ✅ `*.bag` - ROS bag文件
- ✅ `__pycache__/`, `*.pyc` - Python缓存
- ✅ `.vscode/`, `.idea/` - IDE配置
- ✅ `perf_reports/data/` - 生成的性能数据

### 需要关注的文件类型
- ⚠️ 模型文件 (*.onnx, *.pt) - 300MB
- ⚠️ 网格文件 (*.stl) - 14MB（项目必需）

---

## 总结

**当前仓库状态良好**，主要问题是大模型文件。建议：

1. **立即行动**: 配置Git LFS管理模型文件
2. **可选**: 清理生成的报告图片
3. **保留**: 基线配置、文档、源代码

仓库大小可以通过Git LFS优化到合理范围。
