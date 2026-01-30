#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from dataclasses import dataclass
from typing import Dict, List, Optional

@dataclass
class MetricStats:
    mean: float
    p50: float
    p95: float
    p99: float
    max: float

@dataclass
class GitInfo:
    commit_hash: str
    branch: str
    commit_message: str

@dataclass
class SystemInfo:
    hostname: str
    cpu: str
    memory: str
    os: str

@dataclass
class NodeMetrics:
    node: str
    window_size: int
    metrics: Dict[str, MetricStats]
    timestamp: str

@dataclass
class PerformanceData:
    git_info: GitInfo
    system_info: SystemInfo
    collection_time: str
    note: str
    scenario: str
    tags: List[str]
    nodes: Dict[str, List[NodeMetrics]]
