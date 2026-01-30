#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import json
import glob
import yaml
import subprocess
from pathlib import Path
from typing import Dict, List, Optional

# Load configuration
_config_path = Path(__file__).parent.parent / "config.yaml"
with open(_config_path) as f:
    _config = yaml.safe_load(f)

METRIC_DEFINITIONS = _config['metrics']
THRESHOLDS = _config['thresholds']
VISUALIZATION = _config['visualization']

class Config:
    _base_dir = Path(__file__).parent.parent
    DATA_DIR = _base_dir / _config['paths']['data_dir']
    REPORTS_DIR = _base_dir / _config['paths']['reports_dir']
    PROJECT_ROOT = _base_dir / _config['paths']['project_root']

def load_all_data(data_dir: Path = None) -> List[Dict]:
    if data_dir is None:
        data_dir = Config.DATA_DIR
    data_files = glob.glob(os.path.join(data_dir, "perf_data_*.json"))
    all_data = []
    for data_file in sorted(data_files):
        with open(data_file, 'r') as f:
            all_data.append(json.load(f))
    return all_data

def format_value(value: float, precision: int = 3) -> str:
    if value is None:
        return "N/A"
    return f"{value:.{precision}f}"

def format_delta(value: float, precision: int = 3) -> str:
    if value is None:
        return "N/A"
    return f"{value:+.{precision}f}"

def get_latest_metrics(data: Dict, node_name: str) -> Optional[Dict]:
    if node_name not in data.get('nodes', {}):
        return None
    entries = data['nodes'][node_name]
    if not entries:
        return None
    return entries[-1].get('metrics', {})

def get_git_info() -> Dict:
    try:
        commit_hash = subprocess.check_output(
            ["git", "rev-parse", "HEAD"],
            cwd=Config.PROJECT_ROOT,
            stderr=subprocess.DEVNULL
        ).decode().strip()
        commit_msg = subprocess.check_output(
            ["git", "log", "-1", "--pretty=%B"],
            cwd=Config.PROJECT_ROOT,
            stderr=subprocess.DEVNULL
        ).decode().strip()
        branch = subprocess.check_output(
            ["git", "rev-parse", "--abbrev-ref", "HEAD"],
            cwd=Config.PROJECT_ROOT,
            stderr=subprocess.DEVNULL
        ).decode().strip()
        return {
            "commit_hash": commit_hash,
            "commit_message": commit_msg,
            "branch": branch
        }
    except Exception:
        return {
            "commit_hash": "unknown",
            "commit_message": "unknown",
            "branch": "unknown"
        }

def get_system_info() -> Dict:
    try:
        hostname = subprocess.check_output(["hostname"], stderr=subprocess.DEVNULL).decode().strip()
    except Exception:
        hostname = "unknown"

    try:
        cpu_info = subprocess.check_output(
            ["lscpu"], stderr=subprocess.DEVNULL
        ).decode()
        cpu_model = [line for line in cpu_info.split('\n') if 'Model name' in line]
        cpu = cpu_model[0].split(':')[1].strip() if cpu_model else "unknown"
    except Exception:
        cpu = "unknown"

    try:
        mem_info = subprocess.check_output(
            ["free", "-h"], stderr=subprocess.DEVNULL
        ).decode()
        mem_lines = mem_info.split('\n')
        if len(mem_lines) > 1:
            mem_parts = mem_lines[1].split()
            memory = mem_parts[1] if len(mem_parts) > 1 else "unknown"
        else:
            memory = "unknown"
    except Exception:
        memory = "unknown"

    try:
        os_info = subprocess.check_output(
            ["uname", "-a"], stderr=subprocess.DEVNULL
        ).decode().strip()
    except Exception:
        os_info = "unknown"

    return {
        "hostname": hostname,
        "cpu": cpu,
        "memory": memory,
        "os": os_info
    }
