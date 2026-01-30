#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Unified Performance Reporting Tool
Usage:
    perf_tool.py collect [--note NOTE] [--scenario SCENARIO] [--tags TAGS]
    perf_tool.py report [--compare] [--timeline] [--commits COMMITS]
    perf_tool.py chart [--compare]
    perf_tool.py analyze [--data-file FILE]
    perf_tool.py full [--note NOTE] [--scenario SCENARIO]
"""

import argparse
import sys
from pathlib import Path
from common import Config
from collect_perf_data import PerfDataCollector
from generate_report import PerfReportGenerator
from generate_charts import PerfChartGenerator
from analyze_performance import PerformanceAnalyzer

class PerfTool:
    def __init__(self):
        self.config = Config()

    def collect(self, args):
        tags = [tag.strip() for tag in args.tags.split(',')] if args.tags else []
        tags = [tag for tag in tags if tag]
        collector = PerfDataCollector(
            log_file=args.log_file,
            note=args.note,
            scenario=args.scenario,
            tags=tags
        )
        collector.collect_from_log()
        return collector.save_data()

    def report(self, args):
        generator = PerfReportGenerator()
        generator.load_all_data()
        if args.compare:
            return generator.generate_comparison_report(args.commits)
        elif args.timeline:
            return generator.generate_timeline_report()
        else:
            if generator.all_data:
                return generator.generate_single_report(generator.all_data[-1])
            else:
                print("No data available for report generation")
                return None

    def chart(self, args):
        generator = PerfChartGenerator()
        generator.load_all_data()
        if args.compare:
            return generator.generate_comparison_charts()
        else:
            return generator.generate_charts()

    def analyze(self, args):
        analyzer = PerformanceAnalyzer(str(Config.PROJECT_ROOT))
        return analyzer.generate_markdown_report(args.data_file)

    def full(self, args):
        """Run complete workflow: collect -> report -> chart -> analyze"""
        print("=== Step 1: Collecting performance data ===")
        data_file = self.collect(args)

        print("\n=== Step 2: Generating report ===")
        self.report(argparse.Namespace(compare=False, timeline=False, commits=None))

        print("\n=== Step 3: Generating charts ===")
        self.chart(argparse.Namespace(compare=False))

        print("\n=== Step 4: Running analysis ===")
        self.analyze(argparse.Namespace(data_file=data_file))

        print("\n=== Performance reporting complete ===")

def main():
    parser = argparse.ArgumentParser(description='Unified Performance Reporting Tool')
    subparsers = parser.add_subparsers(dest='command', help='Command to run')

    # Collect command
    collect_parser = subparsers.add_parser('collect', help='Collect performance data from ROS logs')
    collect_parser.add_argument('--log-file', help='Path to ROS log file')
    collect_parser.add_argument('--note', help='Short change note for this run')
    collect_parser.add_argument('--scenario', help='Test scenario description')
    collect_parser.add_argument('--tags', help='Comma-separated tags for this run')

    # Report command
    report_parser = subparsers.add_parser('report', help='Generate performance report')
    report_parser.add_argument('--compare', action='store_true', help='Generate comparison report')
    report_parser.add_argument('--timeline', action='store_true', help='Generate timeline report')
    report_parser.add_argument('--commits', help='Comma-separated commit prefixes to compare')

    # Chart command
    chart_parser = subparsers.add_parser('chart', help='Generate performance charts')
    chart_parser.add_argument('--compare', action='store_true', help='Generate comparison charts')

    # Analyze command
    analyze_parser = subparsers.add_parser('analyze', help='Run performance analysis')
    analyze_parser.add_argument('--data-file', help='Specific data file to analyze')

    # Full command
    full_parser = subparsers.add_parser('full', help='Run complete workflow')
    full_parser.add_argument('--note', help='Short change note for this run')
    full_parser.add_argument('--scenario', help='Test scenario description')
    full_parser.add_argument('--tags', help='Comma-separated tags for this run')
    full_parser.add_argument('--log-file', help='Path to ROS log file')

    args = parser.parse_args()

    if not args.command:
        parser.print_help()
        sys.exit(1)

    tool = PerfTool()

    if args.command == 'collect':
        tool.collect(args)
    elif args.command == 'report':
        tool.report(args)
    elif args.command == 'chart':
        tool.chart(args)
    elif args.command == 'analyze':
        tool.analyze(args)
    elif args.command == 'full':
        tool.full(args)

if __name__ == "__main__":
    main()
