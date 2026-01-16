#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import subprocess
from pathlib import Path

def run_command(cmd, description):
    print(f"\n{'='*60}")
    print(f"{description}")
    print(f"{'='*60}")
    try:
        result = subprocess.run(cmd, shell=True, check=True, capture_output=True, text=True)
        print(result.stdout)
        if result.stderr:
            print("STDERR:", result.stderr)
        return True
    except subprocess.CalledProcessError as e:
        print(f"Error: {e}")
        print("STDOUT:", e.stdout)
        print("STDERR:", e.stderr)
        return False

def main():
    print("性能报告自动化工具")
    print("="*60)

    base_dir = "/home/kerwin/2025huat"
    perf_dir = os.path.join(base_dir, "perf_reports")
    scripts_dir = os.path.join(perf_dir, "scripts")
    data_dir = os.path.join(perf_dir, "data")
    reports_dir = os.path.join(perf_dir, "reports")

    os.makedirs(data_dir, exist_ok=True)
    os.makedirs(reports_dir, exist_ok=True)

    print(f"\n工作目录:")
    print(f"  项目根目录: {base_dir}")
    print(f"  性能报告目录: {perf_dir}")
    print(f"  数据目录: {data_dir}")
    print(f"  报告目录: {reports_dir}")

    success = True

    success &= run_command(
        f"python3 {scripts_dir}/collect_perf_data.py --output-dir {data_dir}",
        "步骤 1: 收集性能数据"
    )

    success &= run_command(
        f"python3 {scripts_dir}/generate_report.py --data-dir {data_dir} --output-dir {reports_dir}",
        "步骤 2: 生成性能报告"
    )

    success &= run_command(
        f"python3 {scripts_dir}/generate_charts.py --data-dir {data_dir} --output-dir {reports_dir}",
        "步骤 3: 生成性能图表"
    )

    print(f"\n{'='*60}")
    if success:
        print("✅ 所有步骤完成！")
        print(f"\n报告和图表已生成到: {reports_dir}")
        print("\n最新文件:")
        for file in sorted(os.listdir(reports_dir), reverse=True)[:5]:
            filepath = os.path.join(reports_dir, file)
            size = os.path.getsize(filepath)
            print(f"  - {file} ({size} bytes)")
    else:
        print("❌ 部分步骤失败，请检查错误信息")
    print(f"{'='*60}\n")

    return 0 if success else 1

if __name__ == "__main__":
    sys.exit(main())
