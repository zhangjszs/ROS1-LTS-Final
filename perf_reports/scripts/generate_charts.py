#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import json
import glob
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from datetime import datetime
from pathlib import Path

class PerfChartGenerator:
    def __init__(self, data_dir=None, output_dir=None):
        self.data_dir = data_dir or "/home/kerwin/2025huat/perf_reports/data"
        self.output_dir = output_dir or "/home/kerwin/2025huat/perf_reports/reports"
        self.all_data = []

    def load_all_data(self):
        data_files = glob.glob(os.path.join(self.data_dir, "perf_data_*.json"))
        for data_file in sorted(data_files):
            with open(data_file, 'r') as f:
                self.all_data.append(json.load(f))
        print(f"Loaded {len(self.all_data)} data files for chart generation")

    def generate_charts(self, data=None):
        if data is None and self.all_data:
            data = self.all_data[-1]

        if not data:
            print("No data available for chart generation")
            return []

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        chart_files = []

        for node_name, entries in data['nodes'].items():
            if not entries:
                continue

            latest_entry = entries[-1]
            metrics = latest_entry['metrics']

            chart_file = self._generate_node_charts(node_name, metrics, timestamp)
            if chart_file:
                chart_files.append(chart_file)

        return chart_files

    def generate_comparison_charts(self, data_list=None):
        if data_list is None:
            data_list = self.all_data[-2:] if len(self.all_data) >= 2 else self.all_data

        if len(data_list) < 2:
            print("Need at least 2 data files for comparison charts")
            return []

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        chart_files = []

        all_nodes = set()
        for data in data_list:
            all_nodes.update(data['nodes'].keys())

        for node_name in sorted(all_nodes):
            chart_file = self._generate_comparison_chart(node_name, data_list, timestamp)
            if chart_file:
                chart_files.append(chart_file)

        return chart_files

    def _generate_node_charts(self, node_name, metrics, timestamp):
        chart_filename = f"{node_name}_performance_{timestamp}.png"
        chart_path = os.path.join(self.output_dir, chart_filename)

        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        fig.suptitle(f'{node_name} - Performance Metrics', fontsize=16, fontweight='bold')

        time_metrics = ['t_pass_ms', 't_ground_ms', 't_cluster_ms', 't_delaunay_ms', 't_way_ms', 't_total_ms']
        time_labels = ['Pass', 'Ground', 'Cluster', 'Delaunay', 'Way', 'Total']
        time_values = [metrics.get(m, {}).get('mean', 0) for m in time_metrics]

        ax = axes[0, 0]
        bars = ax.bar(time_labels, time_values, color='skyblue', edgecolor='navy')
        ax.set_ylabel('Time (ms)', fontsize=12)
        ax.set_title('Processing Time Breakdown', fontsize=12)
        ax.grid(axis='y', alpha=0.3)
        for bar in bars:
            height = bar.get_height()
            if height > 0:
                ax.text(bar.get_x() + bar.get_width()/2., height,
                       f'{height:.2f}', ha='center', va='bottom', fontsize=9)

        ax = axes[0, 1]
        if 't_total_ms' in metrics:
            m = metrics['t_total_ms']
            stats = ['Mean', 'P50', 'P95', 'P99', 'Max']
            values = [m['mean'], m['p50'], m['p95'], m['p99'], m['max']]
            colors = ['lightcoral', 'lightblue', 'lightgreen', 'lightyellow', 'lightpink']
            bars = ax.bar(stats, values, color=colors, edgecolor='black')
            ax.set_ylabel('Time (ms)', fontsize=12)
            ax.set_title('Total Processing Time Distribution', fontsize=12)
            ax.grid(axis='y', alpha=0.3)
            for bar in bars:
                height = bar.get_height()
                ax.text(bar.get_x() + bar.get_width()/2., height,
                       f'{height:.2f}', ha='center', va='bottom', fontsize=9)

        ax = axes[1, 0]
        data_metrics = ['N', 'K', 'T', 'E']
        data_labels = ['Points', 'Clusters', 'Triangles', 'Edges']
        data_values = [metrics.get(m, {}).get('mean', 0) for m in data_metrics]
        colors = ['gold', 'lightcoral', 'lightgreen', 'lightblue']
        bars = ax.bar(data_labels, data_values, color=colors, edgecolor='black')
        ax.set_ylabel('Count', fontsize=12)
        ax.set_title('Data Volume Statistics', fontsize=12)
        ax.grid(axis='y', alpha=0.3)
        for bar in bars:
            height = bar.get_height()
            if height > 0:
                ax.text(bar.get_x() + bar.get_width()/2., height,
                       f'{int(height)}', ha='center', va='bottom', fontsize=9)

        ax = axes[1, 1]
        if 't_total_ms' in metrics and 't_pass_ms' in metrics and 't_ground_ms' in metrics and 't_cluster_ms' in metrics:
            total = metrics['t_total_ms']['mean']
            pass_time = metrics['t_pass_ms']['mean']
            ground_time = metrics['t_ground_ms']['mean']
            cluster_time = metrics['t_cluster_ms']['mean']

            if total > 0:
                labels = ['Pass', 'Ground', 'Cluster', 'Other']
                sizes = [pass_time, ground_time, cluster_time, total - pass_time - ground_time - cluster_time]
                colors = ['#ff9999', '#66b3ff', '#99ff99', '#ffcc99']
                explode = (0.1, 0.1, 0.1, 0)

                wedges, texts, autotexts = ax.pie(sizes, explode=explode, labels=labels, colors=colors,
                                                  autopct='%1.1f%%', shadow=True, startangle=90)
                for autotext in autotexts:
                    autotext.set_color('black')
                    autotext.set_fontsize(10)
                ax.set_title('Time Distribution', fontsize=12)

        plt.tight_layout()
        plt.savefig(chart_path, dpi=300, bbox_inches='tight')
        plt.close()

        print(f"Chart generated: {chart_path}")
        return chart_path

    def _generate_comparison_chart(self, node_name, data_list, timestamp):
        chart_filename = f"{node_name}_comparison_{timestamp}.png"
        chart_path = os.path.join(self.output_dir, chart_filename)

        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        fig.suptitle(f'{node_name} - Version Comparison', fontsize=16, fontweight='bold')

        versions = [f"v{i+1}" for i in range(len(data_list))]
        colors = ['blue', 'red', 'green', 'orange', 'purple'][:len(data_list)]

        ax = axes[0, 0]
        time_metrics = ['t_total_ms', 't_pass_ms', 't_ground_ms', 't_cluster_ms']
        time_labels = ['Total', 'Pass', 'Ground', 'Cluster']
        x = np.arange(len(time_labels))
        width = 0.8 / len(data_list)

        for i, data in enumerate(data_list):
            if node_name in data['nodes'] and data['nodes'][node_name]:
                metrics = data['nodes'][node_name][-1]['metrics']
                values = [metrics.get(m, {}).get('mean', 0) for m in time_metrics]
                offset = (i - len(data_list)/2 + 0.5) * width
                bars = ax.bar(x + offset, values, width, label=f"v{i+1}", color=colors[i], alpha=0.7)

        ax.set_ylabel('Time (ms)', fontsize=12)
        ax.set_title('Processing Time Comparison', fontsize=12)
        ax.set_xticks(x)
        ax.set_xticklabels(time_labels)
        ax.legend()
        ax.grid(axis='y', alpha=0.3)

        ax = axes[0, 1]
        if 't_total_ms' in data_list[0]['nodes'].get(node_name, [{}])[-1]['metrics']:
            p95_values = []
            p99_values = []
            for data in data_list:
                if node_name in data['nodes'] and data['nodes'][node_name]:
                    metrics = data['nodes'][node_name][-1]['metrics']
                    p95_values.append(metrics['t_total_ms']['p95'])
                    p99_values.append(metrics['t_total_ms']['p99'])

            x = np.arange(len(versions))
            width = 0.35
            bars1 = ax.bar(x - width/2, p95_values, width, label='P95', color='blue', alpha=0.7)
            bars2 = ax.bar(x + width/2, p99_values, width, label='P99', color='red', alpha=0.7)

            ax.set_ylabel('Time (ms)', fontsize=12)
            ax.set_title('P95/P99 Comparison', fontsize=12)
            ax.set_xticks(x)
            ax.set_xticklabels(versions)
            ax.legend()
            ax.grid(axis='y', alpha=0.3)

        ax = axes[1, 0]
        data_metrics = ['N', 'K', 'T', 'E']
        data_labels = ['Points', 'Clusters', 'Triangles', 'Edges']
        x = np.arange(len(data_labels))
        width = 0.8 / len(data_list)

        for i, data in enumerate(data_list):
            if node_name in data['nodes'] and data['nodes'][node_name]:
                metrics = data['nodes'][node_name][-1]['metrics']
                values = [metrics.get(m, {}).get('mean', 0) for m in data_metrics]
                offset = (i - len(data_list)/2 + 0.5) * width
                bars = ax.bar(x + offset, values, width, label=f"v{i+1}", color=colors[i], alpha=0.7)

        ax.set_ylabel('Count', fontsize=12)
        ax.set_title('Data Volume Comparison', fontsize=12)
        ax.set_xticks(x)
        ax.set_xticklabels(data_labels)
        ax.legend()
        ax.grid(axis='y', alpha=0.3)

        ax = axes[1, 1]
        if 't_total_ms' in data_list[0]['nodes'].get(node_name, [{}])[-1]['metrics']:
            mean_values = []
            for data in data_list:
                if node_name in data['nodes'] and data['nodes'][node_name]:
                    metrics = data['nodes'][node_name][-1]['metrics']
                    mean_values.append(metrics['t_total_ms']['mean'])

            ax.plot(versions, mean_values, marker='o', linewidth=2, markersize=8, color='blue', label='Mean')
            ax.fill_between(versions, mean_values, alpha=0.3, color='blue')

            ax.set_ylabel('Time (ms)', fontsize=12)
            ax.set_title('Performance Trend', fontsize=12)
            ax.set_xlabel('Version', fontsize=12)
            ax.legend()
            ax.grid(alpha=0.3)

        plt.tight_layout()
        plt.savefig(chart_path, dpi=300, bbox_inches='tight')
        plt.close()

        print(f"Comparison chart generated: {chart_path}")
        return chart_path

def main():
    import argparse

    parser = argparse.ArgumentParser(description='Generate performance charts')
    parser.add_argument('--data-dir', help='Directory containing performance data files')
    parser.add_argument('--output-dir', help='Output directory for charts')
    parser.add_argument('--compare', action='store_true', help='Generate comparison charts')
    args = parser.parse_args()

    generator = PerfChartGenerator(data_dir=args.data_dir, output_dir=args.output_dir)
    generator.load_all_data()

    if args.compare:
        generator.generate_comparison_charts()
    else:
        generator.generate_charts()

if __name__ == "__main__":
    main()
