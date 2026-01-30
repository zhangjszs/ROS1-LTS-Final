# Codecov Token Setup Guide

## Step 1: Get Codecov Token

1. Visit https://codecov.io and sign in with your GitHub account
2. Navigate to your repository: `zhangjszs/ROS1-LTS-Final`
3. Go to Settings → General
4. Copy the "Repository Upload Token"

## Step 2: Add Token to GitHub Secrets

1. Go to your GitHub repository: https://github.com/zhangjszs/ROS1-LTS-Final
2. Click on "Settings" tab
3. In the left sidebar, click "Secrets and variables" → "Actions"
4. Click "New repository secret"
5. Name: `CODECOV_TOKEN`
6. Value: Paste the token from Codecov
7. Click "Add secret"

## Step 3: Update Workflow (Already Done)

The workflow file `.github/workflows/code-coverage.yml` is already configured to use the token:

```yaml
- name: Upload coverage to Codecov
  uses: codecov/codecov-action@v4
  with:
    files: ./coverage.info
    fail_ci_if_error: false
```

The `codecov-action@v4` automatically uses the `CODECOV_TOKEN` secret if available.

## Verification

After adding the token:
1. Push a commit to trigger the workflow
2. Check the Actions tab to see if coverage upload succeeds
3. Visit Codecov dashboard to see coverage reports

## Note

The workflow will still run without the token, but coverage reports won't be uploaded to Codecov. The token is only needed for uploading reports to the Codecov service.
