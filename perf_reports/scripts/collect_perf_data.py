#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import re
import json
import subprocess
from datetime import datetime
from pathlib import Path

class PerfDataCollector:
    def __init__(self, log_file=None, note=None, scenario=None, tags=None):
        self.log_file = log_file or os.path.expanduser("~/.ros/log/latest/rosout.log")
        self.perf_data = {}
        self.git_info = self._get_git_info()
        self.system_info = self._get_system_info()
        self.note = note if note is not None else None
        if self.note is None:
            self.note = self.git_info.get("commit_message", "")
        self.scenario = scenario or ""
        self.tags = tags or []

    def _get_git_info(self):
        try:
            commit_hash = subprocess.check_output(
                ["git", "rev-parse", "HEAD"],
                cwd="/home/kerwin/2025huat",
                stderr=subprocess.DEVNULL
            ).decode().strip()
            commit_msg = subprocess.check_output(
                ["git", "log", "-1", "--pretty=%B"],
                cwd="/home/kerwin/2025huat",
                stderr=subprocess.DEVNULL
            ).decode().strip()
            branch = subprocess.check_output(
                ["git", "rev-parse", "--abbrev-ref", "HEAD"],
                cwd="/home/kerwin/2025huat",
                stderr=subprocess.DEVNULL
            ).decode().strip()
            return {
                "commit_hash": commit_hash,
                "commit_message": commit_msg,
                "branch": branch
            }
        except Exception as e:
            return {
                "commit_hash": "unknown",
                "commit_message": "unknown",
                "branch": "unknown"
            }

    def _get_system_info(self):
        try:
            import platform
            try:
                import cpuinfo
                cpu = cpuinfo.get_cpu_info()["brand_raw"] if hasattr(cpuinfo, 'get_cpu_info') else "unknown"
            except ImportError:
                cpu = "unknown"
            return {
                "hostname": platform.node(),
                "os": platform.system(),
                "os_version": platform.release(),
                "python_version": platform.python_version(),
                "cpu": cpu
            }
        except Exception as e:
            return {
                "hostname": "unknown",
                "os": "unknown",
                "os_version": "unknown",
                "python_version": "unknown",
                "cpu": "unknown"
            }

    def parse_perf_line(self, line):
        pattern = r'\[perf\] node=(\w+) window=(\d+) (.+)'
        match = re.search(pattern, line)
        if not match:
            return None

        node_name = match.group(1)
        window_size = int(match.group(2))
        metrics_str = match.group(3)

        metrics = {}
        metric_pattern = r'(\w+)\{mean=([\d.]+),p50=([\d.]+),p95=([\d.]+),p99=([\d.]+),max=([\d.]+)\}'
        for metric_match in re.finditer(metric_pattern, metrics_str):
            metric_name = metric_match.group(1)
            metrics[metric_name] = {
                "mean": float(metric_match.group(2)),
                "p50": float(metric_match.group(3)),
                "p95": float(metric_match.group(4)),
                "p99": float(metric_match.group(5)),
                "max": float(metric_match.group(6))
            }

        return {
            "node": node_name,
            "window_size": window_size,
            "metrics": metrics,
            "timestamp": datetime.now().isoformat()
        }

    def collect_from_log(self):
        if not os.path.exists(self.log_file):
            print(f"Warning: Log file not found: {self.log_file}")
            return

        collected_data = []
        with open(self.log_file, 'r') as f:
            for line in f:
                if '[perf]' in line:
                    perf_entry = self.parse_perf_line(line)
                    if perf_entry:
                        collected_data.append(perf_entry)

        if collected_data:
            for entry in collected_data:
                node_name = entry["node"]
                if node_name not in self.perf_data:
                    self.perf_data[node_name] = []
                self.perf_data[node_name].append(entry)

    def save_data(self, output_dir=None):
        if output_dir is None:
            output_dir = "/home/kerwin/2025huat/perf_reports/data"

        os.makedirs(output_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"perf_data_{timestamp}.json"

        data = {
            "git_info": self.git_info,
            "system_info": self.system_info,
            "collection_time": datetime.now().isoformat(),
            "note": self.note,
            "scenario": self.scenario,
            "tags": self.tags,
            "nodes": self.perf_data
        }

        filepath = os.path.join(output_dir, filename)
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)

        print(f"Performance data saved to: {filepath}")
        return filepath

    def get_latest_stats(self, node_name):
        if node_name not in self.perf_data or not self.perf_data[node_name]:
            return None
        return self.perf_data[node_name][-1]

def main():
    import argparse

    parser = argparse.ArgumentParser(description='Collect performance data from ROS logs')
    parser.add_argument('--log-file', help='Path to ROS log file')
    parser.add_argument('--output-dir', help='Output directory for collected data')
    parser.add_argument('--note', help='Short change note for this run')
    parser.add_argument('--scenario', help='Test scenario description')
    parser.add_argument('--tags', help='Comma-separated tags for this run')
    args = parser.parse_args()

    tags = [tag.strip() for tag in args.tags.split(',')] if args.tags else []
    tags = [tag for tag in tags if tag]
    collector = PerfDataCollector(
        log_file=args.log_file,
        note=args.note,
        scenario=args.scenario,
        tags=tags
    )
    collector.collect_from_log()
    filepath = collector.save_data(output_dir=args.output_dir)

    print(f"\nCollected data for {len(collector.perf_data)} nodes:")
    for node_name, entries in collector.perf_data.items():
        print(f"  - {node_name}: {len(entries)} entries")

if __name__ == "__main__":
    main()
