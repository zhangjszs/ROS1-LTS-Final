#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import json
import glob
from datetime import datetime
from pathlib import Path

class PerfReportGenerator:
    def __init__(self, data_dir=None, output_dir=None):
        self.data_dir = data_dir or "/home/kerwin/2025huat/perf_reports/data"
        self.output_dir = output_dir or "/home/kerwin/2025huat/perf_reports/reports"
        self.all_data = []

    def load_all_data(self):
        data_files = glob.glob(os.path.join(self.data_dir, "perf_data_*.json"))
        for data_file in sorted(data_files):
            with open(data_file, 'r') as f:
                self.all_data.append(json.load(f))
        print(f"Loaded {len(self.all_data)} data files")

    def get_metric_value(self, metrics, metric_name, stat='mean'):
        if metric_name not in metrics:
            return None
        return metrics[metric_name].get(stat, 0.0)

    def format_value(self, value, precision=3):
        if value is None:
            return "N/A"
        return f"{value:.{precision}f}"

    def format_delta(self, value, precision=3):
        if value is None:
            return "N/A"
        return f"{value:+.{precision}f}"

    def _parse_commit_prefixes(self, commit_hashes):
        if not commit_hashes:
            return []
        return [prefix.strip() for prefix in commit_hashes.split(',') if prefix.strip()]

    def _collection_time_key(self, data):
        value = data.get("collection_time", "")
        try:
            return (0, datetime.fromisoformat(value))
        except Exception:
            return (1, value)

    def _get_note(self, data):
        note = data.get("note")
        if note is None or (isinstance(note, str) and not note.strip()):
            note = data.get("git_info", {}).get("commit_message", "")
        if isinstance(note, str):
            note = note.strip()
        return note

    def _get_scenario(self, data):
        scenario = data.get("scenario", "")
        if isinstance(scenario, str):
            scenario = scenario.strip()
        return scenario

    def _get_tags(self, data):
        tags = data.get("tags", [])
        if isinstance(tags, list):
            return [str(tag).strip() for tag in tags if str(tag).strip()]
        if isinstance(tags, str):
            return [tag.strip() for tag in tags.split(',') if tag.strip()]
        return []

    def _format_metadata_value(self, value):
        if value is None:
            return "N/A"
        if isinstance(value, list):
            value = ", ".join(value)
        value = str(value).strip()
        return value if value else "N/A"

    def _format_table_value(self, value):
        value = self._format_metadata_value(value)
        return value.replace("\n", " ").replace("|", "\\|")

    def _get_latest_metrics_for_node(self, data, node_name):
        entries = data.get("nodes", {}).get(node_name, [])
        if not entries:
            return None
        latest_entry = entries[-1]
        return latest_entry.get("metrics", {})

    def generate_single_report(self, data):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        report_filename = f"perf_report_{data['git_info']['commit_hash'][:8]}_{timestamp}.md"
        report_path = os.path.join(self.output_dir, report_filename)

        with open(report_path, 'w', encoding='utf-8') as f:
            f.write(self._generate_markdown(data))

        print(f"Report generated: {report_path}")
        return report_path

    def generate_comparison_report(self, commit_hashes=None):
        if not self.all_data:
            print("No data available for comparison")
            return None

        if commit_hashes:
            prefixes = self._parse_commit_prefixes(commit_hashes)
            filtered_data = []
            missing = []
            for prefix in prefixes:
                matches = [
                    d for d in self.all_data
                    if d.get('git_info', {}).get('commit_hash', '').startswith(prefix)
                ]
                if matches:
                    filtered_data.append(matches[-1])
                else:
                    missing.append(prefix)
            if missing:
                print(f"Warning: No data found for commits: {', '.join(missing)}")
        else:
            filtered_data = self.all_data[-2:] if len(self.all_data) >= 2 else self.all_data

        if len(filtered_data) < 2:
            print("Need at least 2 data files for comparison")
            return None

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        report_filename = f"perf_comparison_{timestamp}.md"
        report_path = os.path.join(self.output_dir, report_filename)

        with open(report_path, 'w', encoding='utf-8') as f:
            f.write(self._generate_comparison_markdown(filtered_data))

        print(f"Comparison report generated: {report_path}")
        return report_path

    def _generate_markdown(self, data):
        md = []
        md.append("# 性能测试报告\n")
        md.append(f"**生成时间**: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")

        md.append("## 1. 测试环境\n\n")
        md.append("### Git 信息\n")
        md.append(f"- **Commit**: `{data['git_info']['commit_hash']}`\n")
        md.append(f"- **分支**: `{data['git_info']['branch']}`\n")
        md.append(f"- **提交信息**: {data['git_info']['commit_message']}\n\n")

        md.append("### 系统信息\n")
        md.append(f"- **主机名**: {data['system_info']['hostname']}\n")
        md.append(f"- **操作系统**: {data['system_info']['os']} {data['system_info']['os_version']}\n")
        md.append(f"- **Python版本**: {data['system_info']['python_version']}\n")
        md.append(f"- **CPU**: {data['system_info']['cpu']}\n\n")

        md.append(f"- **数据收集时间**: {data['collection_time']}\n\n")

        md.append("### 测试场景\n")
        md.append(f"- **场景**: {self._format_metadata_value(self._get_scenario(data))}\n")
        md.append(f"- **变更说明**: {self._format_metadata_value(self._get_note(data))}\n")
        md.append(f"- **标签**: {self._format_metadata_value(self._get_tags(data))}\n\n")

        md.append("## 2. 性能指标\n\n")

        for node_name, entries in data['nodes'].items():
            if not entries:
                continue

            latest_entry = entries[-1]
            metrics = latest_entry['metrics']

            md.append(f"### {node_name}\n\n")
            md.append(f"**统计窗口**: {latest_entry['window_size']} 个样本\n\n")

            md.append("#### 处理时间 (ms)\n")
            md.append("| 指标 | 平均值 | 中位数 | P95 | P99 | 最大值 |\n")
            md.append("|------|--------|--------|-----|-----|--------|\n")

            time_metrics = ['t_pass_ms', 't_ground_ms', 't_cluster_ms', 't_delaunay_ms', 't_way_ms', 't_total_ms']
            for metric in time_metrics:
                if metric in metrics:
                    m = metrics[metric]
                    md.append(f"| {metric} | {self.format_value(m['mean'])} | {self.format_value(m['p50'])} | "
                            f"{self.format_value(m['p95'])} | {self.format_value(m['p99'])} | {self.format_value(m['max'])} |\n")

            md.append("\n#### 数据量统计\n")
            md.append("| 指标 | 平均值 | 中位数 | P95 | P99 | 最大值 |\n")
            md.append("|------|--------|--------|-----|-----|--------|\n")

            data_metrics = ['N', 'K', 'T', 'E', 'D', 'bytes']
            for metric in data_metrics:
                if metric in metrics:
                    m = metrics[metric]
                    md.append(f"| {metric} | {self.format_value(m['mean'])} | {self.format_value(m['p50'])} | "
                            f"{self.format_value(m['p95'])} | {self.format_value(m['p99'])} | {self.format_value(m['max'])} |\n")

            md.append("\n")

        md.append("## 3. 性能分析\n\n")

        for node_name, entries in data['nodes'].items():
            if not entries:
                continue

            latest_entry = entries[-1]
            metrics = latest_entry['metrics']

            md.append(f"### {node_name}\n\n")

            if 't_total_ms' in metrics:
                total_time = metrics['t_total_ms']
                md.append(f"- **平均总处理时间**: {self.format_value(total_time['mean'])} ms\n")
                md.append(f"- **最大处理时间**: {self.format_value(total_time['max'])} ms\n")
                md.append(f"- **P95处理时间**: {self.format_value(total_time['p95'])} ms\n\n")

            if 't_total_ms' in metrics and 't_pass_ms' in metrics and 't_ground_ms' in metrics and 't_cluster_ms' in metrics:
                total = metrics['t_total_ms']['mean']
                pass_time = metrics['t_pass_ms']['mean']
                ground_time = metrics['t_ground_ms']['mean']
                cluster_time = metrics['t_cluster_ms']['mean']

                if total > 0:
                    pass_pct = (pass_time / total) * 100
                    ground_pct = (ground_time / total) * 100
                    cluster_pct = (cluster_time / total) * 100

                    md.append("#### 时间占比分析\n")
                    md.append(f"- 点云过滤: {pass_pct:.2f}%\n")
                    md.append(f"- 地面分割: {ground_pct:.2f}%\n")
                    md.append(f"- 聚类处理: {cluster_pct:.2f}%\n\n")

            if 'N' in metrics:
                md.append(f"- **平均输入点数**: {int(metrics['N']['mean'])}\n")
            if 'K' in metrics and metrics['K']['mean'] > 0:
                md.append(f"- **平均聚类数**: {int(metrics['K']['mean'])}\n")
            if 'T' in metrics and metrics['T']['mean'] > 0:
                md.append(f"- **平均三角形数**: {int(metrics['T']['mean'])}\n")
            if 'E' in metrics and metrics['E']['mean'] > 0:
                md.append(f"- **平均边数**: {int(metrics['E']['mean'])}\n")

            md.append("\n")

        md.append("## 4. 建议\n\n")
        md.append("基于当前性能数据，建议关注以下方面：\n\n")
        md.append("1. 监控P95和P99值，确保系统在高负载下的稳定性\n")
        md.append("2. 识别处理时间占比最大的模块，优先优化\n")
        md.append("3. 关注内存使用和消息发布量（bytes指标）\n")
        md.append("4. 定期进行性能回归测试，确保代码变更不会导致性能下降\n\n")

        return ''.join(md)

    def _generate_comparison_markdown(self, data_list):
        md = []
        md.append("# 性能对比报告\n")
        md.append(f"**生成时间**: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")

        md.append("## 1. 版本信息\n\n")
        md.append("| 版本 | Commit | 分支 | 提交信息 | 测试时间 |\n")
        md.append("|------|--------|------|----------|----------|\n")

        for i, data in enumerate(data_list):
            commit = data['git_info']['commit_hash'][:8]
            branch = data['git_info']['branch']
            msg = data['git_info']['commit_message'][:50] + "..." if len(data['git_info']['commit_message']) > 50 else data['git_info']['commit_message']
            time = data['collection_time'][:19]
            md.append(f"| v{i+1} | `{commit}` | `{branch}` | {msg} | {time} |\n")

        md.append("\n### 变更说明与测试场景\n")
        md.append("| 版本 | 场景 | 变更说明 | 标签 |\n")
        md.append("|------|------|----------|------|\n")
        for i, data in enumerate(data_list):
            scenario = self._format_table_value(self._get_scenario(data))
            note = self._format_table_value(self._get_note(data))
            tags = self._format_table_value(self._get_tags(data))
            md.append(f"| v{i+1} | {scenario} | {note} | {tags} |\n")

        md.append("\n## 2. 性能对比\n\n")

        all_nodes = set()
        for data in data_list:
            all_nodes.update(data['nodes'].keys())

        for node_name in sorted(all_nodes):
            md.append(f"### {node_name}\n\n")

            md.append("#### 总处理时间对比 (ms)\n")
            md.append("| 版本 | 平均值 | 中位数 | P95 | P99 | 最大值 |\n")
            md.append("|------|--------|--------|-----|-----|--------|\n")

            for i, data in enumerate(data_list):
                if node_name in data['nodes'] and data['nodes'][node_name]:
                    metrics = data['nodes'][node_name][-1]['metrics']
                    if 't_total_ms' in metrics:
                        m = metrics['t_total_ms']
                        md.append(f"| v{i+1} | {self.format_value(m['mean'])} | {self.format_value(m['p50'])} | "
                                f"{self.format_value(m['p95'])} | {self.format_value(m['p99'])} | {self.format_value(m['max'])} |\n")

            md.append("\n#### 关键指标对比\n")
            md.append("| 指标 | v1 | v2 | 变化 |\n")
            md.append("|------|----|----|------|\n")

            if len(data_list) >= 2:
                for metric in ['t_total_ms', 'N', 'K', 'T', 'E']:
                    v1_metrics = self._get_latest_metrics_for_node(data_list[0], node_name) or {}
                    v2_metrics = self._get_latest_metrics_for_node(data_list[1], node_name) or {}

                    v1_val = v1_metrics.get(metric, {}).get('mean')
                    v2_val = v2_metrics.get(metric, {}).get('mean')

                    if v1_val is not None and v1_val > 0 and v2_val is not None:
                        change = ((v2_val - v1_val) / v1_val) * 100
                        change_str = f"{change:+.2f}%"
                        if abs(change) > 10:
                            change_str += " ⚠️"
                        elif change < 0:
                            change_str += " ✅"
                    else:
                        change_str = "N/A"

                    md.append(f"| {metric} | {self.format_value(v1_val)} | {self.format_value(v2_val)} | {change_str} |\n")

            md.append("\n")

        md.append("## 3. 分析结论\n\n")
        md.append("### 性能变化趋势\n")
        md.append("- 查看各版本间性能指标的变化趋势\n")
        md.append("- 关注超过10%的性能变化，标记为⚠️\n")
        md.append("- 性能提升标记为✅\n\n")

        md.append("### 优化建议\n")
        md.append("1. 识别性能下降的模块，分析原因\n")
        md.append("2. 保持性能提升的优化\n")
        md.append("3. 建立性能基准，防止回归\n\n")

        return ''.join(md)

    def generate_timeline_report(self):
        if not self.all_data:
            print("No data available for timeline report")
            return None

        sorted_data = sorted(self.all_data, key=self._collection_time_key)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        report_filename = f"perf_timeline_{timestamp}.md"
        report_path = os.path.join(self.output_dir, report_filename)

        with open(report_path, 'w', encoding='utf-8') as f:
            f.write(self._generate_timeline_markdown(sorted_data))

        print(f"Timeline report generated: {report_path}")
        return report_path

    def _generate_timeline_markdown(self, data_list):
        md = []
        md.append("# 性能时间线报告\n")
        md.append(f"**生成时间**: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")

        md.append("## 1. 时间线\n\n")

        all_nodes = set()
        for data in data_list:
            all_nodes.update(data.get("nodes", {}).keys())

        for node_name in sorted(all_nodes):
            rows = []
            prev_mean = None
            prev_p95 = None

            for data in data_list:
                metrics = self._get_latest_metrics_for_node(data, node_name)
                if not metrics or 't_total_ms' not in metrics:
                    continue

                total = metrics['t_total_ms']
                mean_val = total.get('mean')
                p95_val = total.get('p95')
                delta_mean = mean_val - prev_mean if mean_val is not None and prev_mean is not None else None
                delta_p95 = p95_val - prev_p95 if p95_val is not None and prev_p95 is not None else None

                if mean_val is not None:
                    prev_mean = mean_val
                if p95_val is not None:
                    prev_p95 = p95_val

                collection_time = data.get('collection_time', '')
                collection_time = collection_time[:19] if collection_time else "N/A"
                commit = data.get('git_info', {}).get('commit_hash', 'unknown')[:8]
                scenario = self._format_table_value(self._get_scenario(data))
                note = self._format_table_value(self._get_note(data))
                tags = self._format_table_value(self._get_tags(data))

                rows.append(
                    {
                        "time": collection_time,
                        "commit": commit,
                        "scenario": scenario,
                        "note": note,
                        "tags": tags,
                        "mean": self.format_value(mean_val),
                        "p95": self.format_value(p95_val),
                        "delta_mean": self.format_delta(delta_mean),
                        "delta_p95": self.format_delta(delta_p95),
                    }
                )

            if not rows:
                continue

            md.append(f"### {node_name}\n\n")
            md.append("| 时间 | Commit | 场景 | 变更说明 | 标签 | t_total_mean(ms) | t_total_p95(ms) | delta_mean | delta_p95 |\n")
            md.append("|------|--------|------|----------|------|------------------|------------------|------------|-----------|\n")
            for row in rows:
                md.append(
                    f"| {row['time']} | `{row['commit']}` | {row['scenario']} | "
                    f"{row['note']} | {row['tags']} | {row['mean']} | {row['p95']} | "
                    f"{row['delta_mean']} | {row['delta_p95']} |\n"
                )
            md.append("\n")

        return ''.join(md)

def main():
    import argparse

    parser = argparse.ArgumentParser(description='Generate performance reports')
    parser.add_argument('--data-dir', help='Directory containing performance data files')
    parser.add_argument('--output-dir', help='Output directory for reports')
    parser.add_argument('--compare', action='store_true', help='Generate comparison report')
    parser.add_argument('--timeline', action='store_true', help='Generate timeline report')
    parser.add_argument('--commits', help='Comma-separated list of commit hashes to compare')
    args = parser.parse_args()

    generator = PerfReportGenerator(data_dir=args.data_dir, output_dir=args.output_dir)
    generator.load_all_data()

    if args.timeline:
        generator.generate_timeline_report()
    elif args.compare:
        generator.generate_comparison_report(commit_hashes=args.commits)
    else:
        if generator.all_data:
            generator.generate_single_report(generator.all_data[-1])
        else:
            print("No data available. Run collect_perf_data.py first.")

if __name__ == "__main__":
    main()
