# 代码覆盖率测试指南

本文档介绍如何在本地和CI/CD中生成和查看代码覆盖率报告。

## 工具链

- **gcov**: GCC内置的覆盖率工具
- **lcov**: 图形化前端，生成HTML报告
- **gcovr**: 生成XML报告（Codecov兼容）
- **Codecov**: 在线覆盖率分析和可视化平台

---

## 快速开始

### 本地生成覆盖率报告

```bash
# 使用提供的脚本（推荐）
./scripts/generate_coverage.sh

# 或者清理后重新生成
./scripts/generate_coverage.sh clean
```

脚本会自动：
1. 检查并安装依赖（lcov, gcovr）
2. 配置catkin使用覆盖率编译选项
3. 构建项目
4. 运行测试
5. 生成覆盖率报告

### 查看报告

```bash
# HTML报告
firefox coverage-report/index.html
# 或
google-chrome coverage-report/index.html

# 命令行摘要
lcov --summary coverage.info
```

---

## 手动配置步骤

### 1. 安装依赖

```bash
sudo apt-get update
sudo apt-get install -y lcov gcovr
```

### 2. 配置catkin

```bash
cd /home/kerwin/2025huat
source /opt/ros/noetic/setup.bash

# 配置覆盖率编译
catkin config \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Coverage \
    -DCMAKE_CXX_FLAGS="--coverage -fprofile-arcs -ftest-coverage" \
    -DCMAKE_C_FLAGS="--coverage -fprofile-arcs -ftest-coverage"
```

### 3. 构建和测试

```bash
# 构建
catkin build

# 运行测试
source devel/setup.bash
catkin run_tests
```

### 4. 生成报告

```bash
# 捕获覆盖率数据
lcov --capture --directory build --output-file coverage.info

# 过滤系统文件
lcov --remove coverage.info '/opt/*' '/usr/*' '*/test/*' --output-file coverage.info

# 生成HTML报告
genhtml coverage.info --output-directory coverage-report

# 生成XML报告（用于Codecov）
gcovr --root . --build-dir build --xml --output coverage.xml
```

---

## CI/CD集成

### GitHub Actions

项目已配置 `.github/workflows/code-coverage.yml`，会自动：

1. 在每次PR和Push时运行覆盖率测试
2. 生成HTML和XML报告
3. 上传到Codecov
4. 在PR中评论覆盖率摘要

### Codecov配置

配置文件 `.codecov.yml` 包含：

- **项目级覆盖率**: 整体代码覆盖率检查
- **Patch级覆盖率**: PR变更的覆盖率检查
- **组件覆盖率**: 按模块（perception, planning等）分析
- **忽略规则**: 排除测试文件和生成文件

### 查看Codecov报告

1. 访问 [codecov.io](https://codecov.io)
2. 链接您的GitHub仓库
3. 在PR中查看覆盖率评论
4. 查看历史覆盖率趋势

---

## 覆盖率配置详解

### 在CMake中启用覆盖率

在包的 `CMakeLists.txt` 中添加：

```cmake
# 查找覆盖率模块
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../cmake)
include(CodeCoverage)

# 为目标启用覆盖率
target_enable_coverage(${PROJECT_NAME}_node)

# 添加覆盖率目标
add_coverage_target()
```

然后可以运行：

```bash
# 生成单个包的覆盖率报告
catkin build <package_name> -v
make <package_name>_coverage
```

### 排除文件

在 `codecov.yml` 中配置：

```yaml
ignore:
  - "src/*/test/**"      # 测试文件
  - "**/msg/**"          # ROS消息
  - "**/CMakeFiles/**"   # CMake生成文件
```

---

## 覆盖率指标说明

| 指标 | 说明 | 目标值 |
|------|------|--------|
| **Lines** | 代码行覆盖率 | >70% |
| **Functions** | 函数覆盖率 | >70% |
| **Branches** | 分支覆盖率 | >60% |

### 如何解读报告

**HTML报告中的颜色：**
- 🟢 绿色: 已覆盖
- 🔴 红色: 未覆盖
- 🟡 黄色: 部分覆盖（分支）

**关键指标：**
- **Line Coverage**: 最基本的指标，表示执行的代码行百分比
- **Branch Coverage**: 更严格的指标，考虑if/else等分支
- **Function Coverage**: 被调用函数的百分比

---

## 故障排除

### 问题1: "lcov: command not found"

```bash
sudo apt-get install lcov
```

### 问题2: 覆盖率数据为空

```bash
# 确保使用Coverage构建类型
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Coverage

# 清理后重新构建
catkin clean
catkin build
```

### 问题3: 系统文件出现在报告中

```bash
# 确保正确过滤
lcov --remove coverage.info '/opt/*' '/usr/*' --output-file coverage.info
```

### 问题4: Codecov上传失败

```bash
# 检查token是否配置
# 在GitHub仓库Settings -> Secrets中添加 CODECOV_TOKEN
```

---

## 最佳实践

1. **定期运行覆盖率测试**: 建议每次提交前本地运行
2. **设置覆盖率门槛**: 在Codecov中设置最低覆盖率要求
3. **关注新代码覆盖率**: PR的patch覆盖率比整体覆盖率更重要
4. **排除不可测试代码**: 如main函数、错误处理等
5. **结合静态分析**: 覆盖率+静态分析=更全面的质量保障

---

## 参考链接

- [lcov文档](http://ltp.sourceforge.net/coverage/lcov.php)
- [gcovr文档](https://gcovr.com/en/stable/)
- [Codecov文档](https://docs.codecov.io/)
- [ROS测试指南](http://wiki.ros.org/rostest)
