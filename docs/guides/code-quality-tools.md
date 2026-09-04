# 代码质量工具使用指南

本文档介绍项目中配置的代码规格化和漏洞检测工具的使用方法。

## 已配置工具概览

| 工具 | 用途 | 配置文件 |
|------|------|----------|
| clang-format | C++代码格式化 | `.clang-format` |
| black | Python代码格式化 | `pyproject.toml` |
| isort | Python import排序 | `pyproject.toml` |
| flake8 | Python风格检查 | `.flake8` |
| clang-tidy | C++静态分析 | `.clang-tidy` |
| cppcheck | C++漏洞扫描 | `.cppcheck-suppressions` |
| bandit | Python安全扫描 | `.bandit.yaml` |
| pre-commit | 提交前检查 | `.pre-commit-config.yaml` |

---

## C++ 代码格式化

### 使用 clang-format

```bash
# 格式化单个文件
clang-format -i src/perception_core/src/cone_sensor.cpp

# 格式化所有C++文件
find src -type f \( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" \) | xargs clang-format -i

# 检查格式（不修改文件）
find src -type f \( -name "*.cpp" -o -name "*.hpp" \) | xargs clang-format --dry-run --Werror
```

### 配置说明

- 基于 Google Style
- 缩进：2空格
- 行宽限制：100字符
- 指针左对齐：`int* ptr`

---

## Python 代码格式化

### 使用 black

```bash
# 格式化所有Python文件
black src/

# 检查格式（不修改文件）
black --check src/
```

### 使用 isort

```bash
# 排序所有import
isort src/

# 检查排序（不修改文件）
isort --check-only src/
```

### 使用 flake8

```bash
# 风格检查
flake8 src/
```

---

## C++ 静态分析

### 使用 clang-tidy

```bash
# 分析单个文件（需要compile_commands.json）
clang-tidy src/perception_core/src/cone_sensor.cpp -p build/

# 分析整个包
cd src/perception_core
find . -name "*.cpp" | xargs clang-tidy -p ../../build/
```

### 集成到CMake

在包的 `CMakeLists.txt` 中添加：

```cmake
find_program(CLANG_TIDY_EXE NAMES clang-tidy)
if(CLANG_TIDY_EXE)
  set(CMAKE_CXX_CLANG_TIDY ${CLANG_TIDY_EXE})
endif()
```

---

## C++ 漏洞扫描

### 使用 cppcheck

```bash
# 基础扫描
cppcheck --enable=all --std=c++17 src/

# 生成XML报告
cppcheck \
  --enable=all \
  --std=c++17 \
  --xml \
  --xml-version=2 \
  src/ 2> cppcheck-report.xml

# 生成HTML报告
cppcheck-htmlreport --file=cppcheck-report.xml --report-dir=cppcheck-report
```

---

## Python 安全扫描

### 使用 bandit

```bash
# 扫描整个项目
bandit -r src/

# 生成JSON报告
bandit -r src/ -f json -o bandit-report.json

# 只显示中高风险问题
bandit -r src/ -ll
```

---

## 代码覆盖率

### 生成覆盖率报告

```bash
# 1. 清理之前的构建
catkin clean

# 2. 配置覆盖率编译
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Coverage -DCMAKE_CXX_FLAGS="--coverage"

# 3. 构建
catkin build

# 4. 运行测试
catkin run_tests

# 5. 生成报告
lcov --capture --directory build --output-file coverage.info
lcov --remove coverage.info '/opt/*' '/usr/*' '*/test/*' --output-file coverage.info
genhtml coverage.info --output-directory coverage-report

# 6. 查看报告
# 打开 coverage-report/index.html
```

---

## Pre-commit 钩子

### 安装 pre-commit

```bash
pip install pre-commit
pre-commit install
```

### 手动运行检查

```bash
# 检查所有文件
pre-commit run --all-files

# 检查特定钩子
pre-commit run clang-format --all-files
pre-commit run black --all-files
```

---

## CI/CD 工作流

项目配置了以下GitHub Actions工作流：

| 工作流 | 触发条件 | 功能 |
|--------|----------|------|
| `code-style.yml` | PR/Push | C++和Python格式检查 |
| `static-analysis.yml` | PR/Push/定时 | 静态分析和安全扫描 |
| `code-coverage.yml` | PR/Push/定时 | 代码覆盖率报告 |

---

## 本地开发建议

### 提交代码前

```bash
# 1. 格式化代码
find src -type f \( -name "*.cpp" -o -name "*.hpp" \) | xargs clang-format -i
black src/
isort src/

# 2. 运行测试
catkin run_tests

# 3. 检查格式
find src -type f \( -name "*.cpp" -o -name "*.hpp" \) | xargs clang-format --dry-run --Werror
flake8 src/
```

### 推荐IDE配置

**VS Code 扩展：**
- clang-format
- Python (Microsoft)
- Flake8
- C/C++

**CLion 配置：**
- 设置 clang-format 路径
- 启用 clang-tidy 检查

---

## 故障排除

### clang-format 配置错误

```bash
# 验证配置
clang-format --dump-config > /dev/null

# 常见错误：ReferenceAlignment 需要 clang-format 11+
# 解决方案：移除该选项或使用较新版本
```

### clang-tidy 找不到头文件

```bash
# 确保生成 compile_commands.json
catkin config --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
catkin build
```

---

## 参考链接

- [clang-format 文档](https://clang.llvm.org/docs/ClangFormat.html)
- [clang-tidy 文档](https://clang.llvm.org/extra/clang-tidy/)
- [cppcheck 文档](http://cppcheck.sourceforge.net/)
- [black 文档](https://black.readthedocs.io/)
- [bandit 文档](https://bandit.readthedocs.io/)
