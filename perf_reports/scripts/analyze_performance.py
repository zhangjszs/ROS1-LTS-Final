#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
自动化性能分析脚本
用于生成详细的性能评估报告
"""

import os
import re
import json
import argparse
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Tuple, Optional
import subprocess


class PerformanceAnalyzer:
    """性能分析器 - 自动化性能瓶颈识别和分析"""

    def __init__(self, project_path: str, perf_data_file: Optional[str] = None):
        self.project_path = Path(project_path)
        self.perf_data_file = perf_data_file
        self.issues = []
        self.components = {}

        # 性能阈值配置
        self.thresholds = {
            'processing_time_ms': {
                'total': {'warning': 40.0, 'critical': 50.0},
                'ground_segmentation': {'warning': 25.0, 'critical': 35.0},
                'clustering': {'warning': 8.0, 'critical': 12.0},
                'delaunay': {'warning': 1.0, 'critical': 2.0},
                'path_planning': {'warning': 10.0, 'critical': 15.0},
            },
            'memory_bytes': {
                'message_size': {'warning': 200000, 'critical': 300000},  # 200KB, 300KB
            },
            'latency_percentiles': {
                'p99_p95_ratio': {'warning': 1.3, 'critical': 1.5},  # P99不应超过P95太多
            }
        }

    def load_performance_data(self, perf_file: str) -> Dict:
        """加载性能数据JSON文件"""
        with open(perf_file, 'r') as f:
            return json.load(f)

    def analyze_codebase(self) -> List[Dict]:
        """分析代码库，识别性能问题"""
        issues = []

        # 分析C++代码模式
        cpp_patterns = {
            'vector_reserve_missing': {
                'pattern': r'std::vector.*\n\s*for\s*\(',
                'severity': 'medium',
                'description': 'Vector未使用reserve()预分配',
                'fix': '在循环前添加 vector.reserve(expected_size);'
            },
            'redundant_copy': {
                'pattern': r'=\s*\*\w+;\s*\n\s*\w+\s*=\s*\w+;',
                'severity': 'high',
                'description': '检测到可能的冗余复制',
                'fix': '考虑使用const引用或避免不必要的拷贝'
            },
            'pow_for_square': {
                'pattern': r'pow\([^,]+,\s*2\)',
                'severity': 'low',
                'description': '使用pow(x, 2)计算平方',
                'fix': '使用 x * x 替代 pow(x, 2)'
            },
            'sine_cosine_in_loop': {
                'pattern': r'for.*\n.*\b(sin|cos)\(',
                'severity': 'medium',
                'description': '循环内重复计算三角函数',
                'fix': '在循环外预计算三角函数值'
            },
            'matrix_reconstruction': {
                'pattern': r'MatrixXf\s+\w+\([^)]+\);\s*\n\s*for\s*\(',
                'severity': 'critical',
                'description': '循环内重建矩阵',
                'fix': '在循环外预分配矩阵'
            },
        }

        # 扫描关键源文件
        critical_files = [
            'src/lidar_cluster/src/ground_segmentation.cpp',
            'src/lidar_cluster/src/lidar_cluster.cpp',
            'src/planning/high_speed_tracking/src/modules/DelaunayTri.cpp',
            'src/planning/high_speed_tracking/src/modules/WayComputer.cpp',
            'src/location_kdtree/src/location.cpp',
            'src/control/src/control.cpp',
        ]

        for file_path in critical_files:
            full_path = self.project_path / file_path
            if not full_path.exists():
                continue

            with open(full_path, 'r', encoding='utf-8', errors='ignore') as f:
                content = f.read()
                lines = content.split('\n')

            for pattern_name, pattern_info in cpp_patterns.items():
                matches = re.finditer(pattern_info['pattern'], content, re.MULTILINE)
                for match in matches:
                    line_num = content[:match.start()].count('\n') + 1
                    issues.append({
                        'file': file_path,
                        'line': line_num,
                        'severity': pattern_info['severity'],
                        'type': pattern_name,
                        'description': pattern_info['description'],
                        'fix': pattern_info['fix'],
                        'code_snippet': lines[line_num - 1].strip() if line_num <= len(lines) else ''
                    })

        return issues

    def analyze_performance_metrics(self, perf_data: Dict) -> List[Dict]:
        """分析性能指标，识别瓶颈"""
        metric_issues = []

        if 'nodes' not in perf_data:
            return metric_issues

        for node_name, node_data in perf_data['nodes'].items():
            if not node_data:
                continue

            latest_entry = node_data[-1]
            metrics = latest_entry.get('metrics', {})

            # 检查总处理时间
            if 't_total_ms' in metrics:
                total = metrics['t_total_ms']['mean']
                threshold = self.thresholds['processing_time_ms']['total']
                if total > threshold['critical']:
                    metric_issues.append({
                        'node': node_name,
                        'type': 'critical_latency',
                        'severity': 'critical',
                        'metric': 't_total_ms',
                        'current': total,
                        'threshold': threshold['critical'],
                        'description': f'总处理时间 {total:.2f}ms 超过临界阈值 {threshold["critical"]}ms'
                    })
                elif total > threshold['warning']:
                    metric_issues.append({
                        'node': node_name,
                        'type': 'warning_latency',
                        'severity': 'warning',
                        'metric': 't_total_ms',
                        'current': total,
                        'threshold': threshold['warning'],
                        'description': f'总处理时间 {total:.2f}ms 超过警告阈值 {threshold["warning"]}ms'
                    })

            # 检查各阶段时间
            stage_keys = ['t_ground_ms', 't_cluster_ms', 't_delaunay_ms', 't_way_ms']
            for stage_key in stage_keys:
                if stage_key in metrics:
                    stage_time = metrics[stage_key]['mean']
                    stage_name = stage_key.replace('t_', '').replace('_ms', '')
                    if stage_name in self.thresholds['processing_time_ms']:
                        threshold = self.thresholds['processing_time_ms'][stage_name]
                        if stage_time > threshold['critical']:
                            metric_issues.append({
                                'node': node_name,
                                'type': 'stage_critical',
                                'severity': 'critical',
                                'stage': stage_name,
                                'metric': stage_key,
                                'current': stage_time,
                                'threshold': threshold['critical'],
                                'description': f'{stage_name} 时间 {stage_time:.2f}ms 超过临界阈值'
                            })

            # 检查消息大小
            if 'bytes' in metrics:
                msg_size = metrics['bytes']['mean']
                threshold = self.thresholds['memory_bytes']['message_size']
                if msg_size > threshold['critical']:
                    metric_issues.append({
                        'node': node_name,
                        'type': 'message_size_critical',
                        'severity': 'critical',
                        'metric': 'bytes',
                        'current': msg_size / 1024.0,  # KB
                        'threshold': threshold['critical'] / 1024.0,
                        'description': f'平均消息大小 {msg_size/1024:.2f}KB 超过临界阈值'
                    })

            # 检查P99/P99比率
            for metric_name, metric_data in metrics.items():
                if 'p95' in metric_data and 'p99' in metric_data:
                    ratio = metric_data['p99'] / (metric_data['p95'] if metric_data['p95'] > 0 else 1.0)
                    threshold = self.thresholds['latency_percentiles']['p99_p95_ratio']
                    if ratio > threshold['critical']:
                        metric_issues.append({
                            'node': node_name,
                            'type': 'latency_variance',
                            'severity': 'warning',
                            'metric': metric_name,
                            'p95': metric_data['p95'],
                            'p99': metric_data['p99'],
                            'ratio': ratio,
                            'description': f'{metric_name} P99/P95比率 {ratio:.2f} 超过阈值，存在性能抖动'
                        })

        return metric_issues

    def prioritize_issues(self, issues: List[Dict]) -> List[Dict]:
        """按影响优先级排序问题"""
        priority_map = {
            'critical': 1,
            'high': 2,
            'medium': 3,
            'warning': 3,
            'low': 4
        }

        # 计算每个问题的优先级分数
        for issue in issues:
            severity = issue.get('severity', 'low')
            issue['priority'] = priority_map.get(severity, 4)

        # 按优先级排序
        return sorted(issues, key=lambda x: x['priority'])

    def generate_markdown_report(self, perf_data_file: str, output_file: str = None) -> str:
        """生成Markdown格式的性能分析报告"""
        # 加载性能数据
        perf_data = self.load_performance_data(perf_data_file)

        # 分析代码
        code_issues = self.analyze_codebase()

        # 分析性能指标
        metric_issues = self.analyze_performance_metrics(perf_data)

        # 合并并排序所有问题
        all_issues = code_issues + metric_issues
        prioritized_issues = self.prioritize_issues(all_issues)

        # 生成报告
        report = self._generate_report_content(perf_data, prioritized_issues, code_issues, metric_issues)

        # 保存报告
        if output_file is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_dir = self.project_path / "perf_reports" / "reports"
            output_dir.mkdir(parents=True, exist_ok=True)
            output_file = output_dir / f"performance_analysis_{timestamp}.md"

        with open(output_file, 'w', encoding='utf-8') as f:
            f.write(report)

        print(f"性能分析报告已生成: {output_file}")
        return str(output_file)

    def _generate_report_content(self, perf_data: Dict, issues: List[Dict],
                               code_issues: List[Dict], metric_issues: List[Dict]) -> str:
        """生成报告内容"""
        lines = []

        # 标题
        lines.append("# 性能分析报告\n")
        lines.append(f"**生成时间**: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")

        # 环境信息
        lines.append("## 环境信息\n")
        if 'git_info' in perf_data:
            git = perf_data['git_info']
            lines.append(f"- **Commit**: `{git.get('commit_hash', 'unknown')}`")
            lines.append(f"- **分支**: `{git.get('branch', 'unknown')}`")
            lines.append(f"- **提交信息**: {git.get('commit_message', 'unknown')}")
        if 'system_info' in perf_data:
            sys = perf_data['system_info']
            lines.append(f"- **主机名**: {sys.get('hostname', 'unknown')}")
            lines.append(f"- **操作系统**: {sys.get('os', 'unknown')} {sys.get('os_version', '')}")
            lines.append(f"- **Python版本**: {sys.get('python_version', 'unknown')}")
            lines.append(f"- **CPU**: {sys.get('cpu', 'unknown')}")
        lines.append("")

        # 执行摘要
        lines.append("## 执行摘要\n")

        critical_count = sum(1 for i in issues if i.get('severity') == 'critical')
        high_count = sum(1 for i in issues if i.get('severity') == 'high')
        medium_count = sum(1 for i in issues if i.get('severity') in ['medium', 'warning'])

        lines.append(f"- **严重问题**: {critical_count} 个")
        lines.append(f"- **高优先级问题**: {high_count} 个")
        lines.append(f"- **中等优先级问题**: {medium_count} 个")
        lines.append("")

        # 性能指标概览
        if 'nodes' in perf_data:
            lines.append("## 性能指标概览\n")

            for node_name, node_data in perf_data['nodes'].items():
                if not node_data:
                    continue

                lines.append(f"### {node_name}\n")
                latest = node_data[-1]
                metrics = latest.get('metrics', {})

                # 处理时间表
                lines.append("#### 处理时间 (ms)\n")
                lines.append("| 指标 | 平均值 | P95 | P99 | 最大值 |")
                lines.append("|------|--------|-----|-----|--------|")

                time_metrics = ['t_pass_ms', 't_ground_ms', 't_cluster_ms',
                              't_delaunay_ms', 't_way_ms', 't_total_ms']
                for tm in time_metrics:
                    if tm in metrics:
                        m = metrics[tm]
                        lines.append(f"| {tm} | {m['mean']:.3f} | {m['p95']:.3f} | "
                                   f"{m['p99']:.3f} | {m['max']:.3f} |")
                lines.append("")

                # 数据量表
                lines.append("#### 数据量统计\n")
                lines.append("| 指标 | 平均值 | P95 | P99 | 最大值 |")
                lines.append("|------|--------|-----|-----|--------|")

                data_metrics = ['n_points', 'n_clusters', 'n_triangles', 'n_edges', 'bytes']
                for dm in data_metrics:
                    if dm in metrics:
                        m = metrics[dm]
                        display_value = f"{m['mean']:.0f}" if dm != 'bytes' else f"{m['mean']:.0f} ({m['mean']/1024:.1f}KB)"
                        p95_value = f"{m['p95']:.0f}" if dm != 'bytes' else f"{m['p95']:.0f}"
                        p99_value = f"{m['p99']:.0f}" if dm != 'bytes' else f"{m['p99']:.0f}"
                        max_value = f"{m['max']:.0f}" if dm != 'bytes' else f"{m['max']:.0f}"
                        lines.append(f"| {dm} | {display_value} | {p95_value} | {p99_value} | {max_value} |")
                lines.append("")

        # 详细问题分析
        lines.append("## 详细问题分析\n")

        # 按优先级分组
        for severity in ['critical', 'high', 'medium', 'warning', 'low']:
            severity_issues = [i for i in issues if i.get('severity') == severity]
            if not severity_issues:
                continue

            severity_name = {
                'critical': '严重问题 (P1)',
                'high': '高优先级问题 (P2)',
                'medium': '中等优先级问题 (P3)',
                'warning': '警告',
                'low': '低优先级问题'
            }.get(severity, severity)

            lines.append(f"### {severity_name}\n")

            for idx, issue in enumerate(severity_issues, 1):
                lines.append(f"#### {idx}. {issue.get('description', issue.get('type', 'Unknown'))}\n")

                if 'node' in issue:
                    lines.append(f"- **节点**: {issue['node']}")

                if 'file' in issue:
                    lines.append(f"- **位置**: [{issue['file']}]({issue['file']})"
                              f" (第 {issue['line']} 行)")

                if 'metric' in issue:
                    lines.append(f"- **指标**: {issue['metric']}")

                if 'current' in issue:
                    current_val = issue['current']
                    threshold_val = issue.get('threshold', 0)
                    unit = 'KB' if 'bytes' in issue.get('metric', '') else 'ms'
                    lines.append(f"- **当前值**: {current_val:.2f} {unit}")
                    if threshold_val > 0:
                        lines.append(f"- **阈值**: {threshold_val:.2f} {unit}")

                if 'code_snippet' in issue and issue['code_snippet']:
                    lines.append(f"- **代码**: `{issue['code_snippet']}`")

                if 'fix' in issue:
                    lines.append(f"- **修复建议**: {issue['fix']}")

                lines.append("")

        # 性能建议
        lines.append("## 性能优化建议\n")

        if code_issues:
            lines.append("### 代码层面优化\n")
            for issue in code_issues[:5]:  # 只显示前5个
                if issue['severity'] in ['critical', 'high']:
                    lines.append(f"- **[{issue['file']}]({issue['file']}:{issue['line']}**: {issue['description']}")
                    lines.append(f"  - 建议: {issue['fix']}")
                    lines.append("")

        if metric_issues:
            lines.append("### 性能指标优化\n")
            # 按节点分组
            nodes = set(i['node'] for i in metric_issues if 'node' in i)
            for node in sorted(nodes):
                node_issues = [i for i in metric_issues if i.get('node') == node]
                lines.append(f"#### {node}\n")
                for issue in node_issues:
                    lines.append(f"- {issue['description']}")
                lines.append("")

        # 性能总结表
        lines.append("## 性能总结表\n")
        lines.append("| 组件 | 当前平均 | P99 | 目标 | 差距 | 优先级 |")
        lines.append("|------|---------|-----|------|------|--------|")

        if 'nodes' in perf_data:
            for node_name, node_data in perf_data['nodes'].items():
                if not node_data:
                    continue
                latest = node_data[-1]
                metrics = latest.get('metrics', {})

                if 't_total_ms' in metrics:
                    total = metrics['t_total_ms']
                    target = 25.0 if 'lidar_cluster' in node_name else 10.0
                    gap = total['mean'] - target
                    priority = 'P1' if gap > 15 else 'P2' if gap > 5 else 'P3'
                    lines.append(f"| {node_name} | {total['mean']:.2f}ms | {total['p99']:.2f}ms | "
                               f"{target}ms | {gap:.2f}ms | {priority} |")
        lines.append("")

        return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description='自动化性能分析工具')
    parser.add_argument('--perf-data', required=True, help='性能数据JSON文件路径')
    parser.add_argument('--output', help='输出报告文件路径')
    parser.add_argument('--project-path', default='.', help='项目根目录路径')

    args = parser.parse_args()

    analyzer = PerformanceAnalyzer(args.project_path)
    report_path = analyzer.generate_markdown_report(args.perf_data, args.output)

    print(f"\n报告生成完成!")
    print(f"文件位置: {report_path}")


if __name__ == '__main__':
    main()
