# Codecov Token 配置指南

## Token 信息
```
CODECOV_TOKEN: <your-codecov-token>
```

## 添加到 GitHub Secrets

### 步骤：
1. 访问: https://github.com/zhangjszs/ROS1-LTS-Final/settings/secrets/actions
2. 点击 "New repository secret"
3. 填写:
   - Name: `CODECOV_TOKEN`
   - Value: `<从 Codecov 获取的新 token>`
4. 点击 "Add secret"

## 验证

添加后，下次 push 将触发覆盖率上传到 Codecov。

查看报告: https://codecov.io/gh/zhangjszs/ROS1-LTS-Final
