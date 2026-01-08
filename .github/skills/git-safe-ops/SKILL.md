---
name: git-safe-ops
description: Git 安全操作：回到上次提交、撤销单文件/全仓库改动、清理 untracked（先 dry-run），避免误删与不可逆操作
---

# git-safe-ops

## 常用目标与命令（必须按这个优先给）
### 回到上次提交（HEAD）
1) 丢弃已跟踪文件修改：
   - `git restore .`
2) 清理未跟踪文件（先预览再执行）：
   - `git clean -nd`
   - `git clean -fd`

### 只撤销某个文件
- `git restore path/to/file`

### 同时撤销 staged（若有）
- `git restore --staged .`
- `git restore .`

## 安全规则
- `git clean` 必须先给 `-n` 预览
- 不建议直接 `git reset --hard`，除非解释风险并确认无未提交重要修改

## 输出格式
- 先一句话确认用户目标
- 给 2～3 条命令（含 dry-run）
- 最后提示 `git status` 验证
