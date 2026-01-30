# Performance Reporting System Architecture

## Overview

The performance reporting system is a unified, maintainable architecture for collecting, analyzing, and visualizing ROS node performance metrics. The system follows a modular design with clear separation of concerns.

## System Components

### Core Modules

#### 1. `common.py` - Shared Utilities
Central module providing shared functionality across all components:
- **Configuration Management**: Loads `config.yaml` and provides `Config` class
- **Metric Definitions**: Centralized metric registry with standardized naming
- **Data Loading**: Unified `load_all_data()` function
- **Formatting**: `format_value()`, `format_delta()` for consistent output
- **System Info**: `get_git_info()`, `get_system_info()` for environment data

#### 2. `models.py` - Data Models
Type-safe data structures using Python dataclasses:
- `MetricStats`: Statistical metrics (mean, p50, p95, p99, max)
- `GitInfo`: Git repository information
- `SystemInfo`: System environment information
- `NodeMetrics`: Performance metrics for a single node
- `PerformanceData`: Complete performance data structure

#### 3. `config.yaml` - Configuration
Centralized configuration file containing:
- File paths (data_dir, reports_dir, project_root)
- Metric definitions (time_metrics, data_metrics, display_names)
- Performance thresholds (processing_time_ms, memory_bytes, latency_percentiles)
- Visualization settings (chart_dpi, figure_size, colors)

### Functional Modules

#### 4. `collect_perf_data.py` - Data Collection
Collects performance metrics from ROS logs:
- Parses `[perf]` log entries
- Extracts git and system information
- Saves data to JSON files with timestamps
- **Dependencies**: `common.py` for git/system info

#### 5. `generate_report.py` - Report Generation
Generates markdown performance reports:
- Single report: Latest performance snapshot
- Comparison report: Compare multiple commits
- Timeline report: Historical performance trends
- **Dependencies**: `common.py` for data loading and formatting

#### 6. `generate_charts.py` - Chart Generation
Creates performance visualization charts:
- Time breakdown charts
- Data volume statistics
- Percentile distributions
- Comparison charts across versions
- **Dependencies**: `common.py` for metrics and visualization config

#### 7. `analyze_performance.py` - Performance Analysis
Automated performance issue detection:
- Threshold-based issue identification
- Code pattern analysis
- Markdown report generation with recommendations
- **Dependencies**: `common.py` for thresholds

#### 8. `perf_tool.py` - Unified CLI
Single entry point for all operations:
- `collect`: Collect performance data
- `report`: Generate reports (single/comparison/timeline)
- `chart`: Generate charts
- `analyze`: Run performance analysis
- `full`: Complete workflow (collect → report → chart → analyze)

## Data Flow

```
ROS Logs → collect_perf_data.py → JSON Files
                                      ↓
                            ┌─────────┴─────────┐
                            ↓                   ↓
                    generate_report.py   generate_charts.py
                            ↓                   ↓
                        Markdown             PNG Charts
                            ↓
                    analyze_performance.py
                            ↓
                    Analysis Report
```

## File Structure

```
perf_reports/
├── config.yaml                    # Centralized configuration
├── ARCHITECTURE.md                # This file
├── README.md                      # User documentation
├── QUICKSTART.md                  # Quick start guide
├── requirements.txt               # Python dependencies
├── quick_report.sh               # Quick report generation script
├── data/                         # Performance data (JSON)
├── reports/                      # Generated reports and charts
├── schemas/                      # JSON schema validation
│   └── perf_data_schema.json    # Data validation schema
└── scripts/
    ├── common.py                 # Shared utilities
    ├── models.py                 # Data models
    ├── perf_tool.py             # Unified CLI
    ├── collect_perf_data.py     # Data collection
    ├── generate_report.py       # Report generation
    ├── generate_charts.py       # Chart generation
    └── analyze_performance.py   # Performance analysis
```

## Configuration Management

All configuration is centralized in `config.yaml`:

```yaml
paths:
  data_dir: "data"
  reports_dir: "reports"
  project_root: "../.."

metrics:
  time_metrics: [t_pass_ms, t_ground_ms, ...]
  data_metrics: [N, K, T, E, D, bytes]

thresholds:
  processing_time_ms:
    total: {warning: 40.0, critical: 50.0}
    ...

visualization:
  chart_dpi: 300
  figure_size: [15, 10]
  colors: [...]
```

## Metric Naming Convention

Standardized metric names across the system:
- **Time metrics**: `t_*_ms` (e.g., `t_total_ms`, `t_ground_ms`)
- **Data metrics**: Single letter codes (e.g., `N` for points, `K` for clusters)
- **Display names**: Human-readable names in `config.yaml`

## Extension Points

### Adding New Metrics
1. Add metric name to `config.yaml` under `metrics`
2. Add display name to `metrics.display_names`
3. Metrics automatically appear in reports and charts

### Adding New Thresholds
1. Add threshold to `config.yaml` under `thresholds`
2. Update `analyze_performance.py` to check new threshold

### Adding New Visualizations
1. Add visualization config to `config.yaml` under `visualization`
2. Update `generate_charts.py` to use new config

## API Documentation

### Common Utilities

```python
from common import Config, load_all_data, format_value

# Access configuration
data_dir = Config.DATA_DIR
reports_dir = Config.REPORTS_DIR

# Load all performance data
all_data = load_all_data()

# Format values
formatted = format_value(42.123, precision=2)  # "42.12"
```

### Data Models

```python
from models import MetricStats, NodeMetrics

# Create metric stats
stats = MetricStats(mean=10.5, p50=9.8, p95=15.2, p99=18.1, max=20.3)

# Create node metrics
metrics = NodeMetrics(
    node="perception",
    window_size=100,
    metrics={"t_total_ms": stats},
    timestamp="2025-01-31T12:00:00"
)
```

## Best Practices

1. **Use shared utilities**: Always use functions from `common.py` instead of duplicating code
2. **Follow naming conventions**: Use standardized metric names from `config.yaml`
3. **Type safety**: Use dataclasses from `models.py` for type checking
4. **Configuration-driven**: Add new settings to `config.yaml` instead of hardcoding
5. **Unified CLI**: Use `perf_tool.py` for all operations

## Performance Considerations

- **Data loading**: `load_all_data()` caches results within a single execution
- **Chart generation**: Uses matplotlib with 'Agg' backend for headless operation
- **Memory usage**: Processes data files sequentially to minimize memory footprint

## Testing

```bash
# Test configuration loading
python3 -c "from scripts.common import Config; print(Config.DATA_DIR)"

# Test unified CLI
python3 scripts/perf_tool.py --help

# Test full workflow
python3 scripts/perf_tool.py full --note "Test run"
```

## Troubleshooting

### Import Errors
- Ensure `pyyaml` is installed: `pip3 install pyyaml`
- Run scripts from `perf_reports/` directory or use absolute imports

### Configuration Errors
- Verify `config.yaml` syntax with: `python3 -c "import yaml; yaml.safe_load(open('config.yaml'))"`
- Check file paths are relative to `perf_reports/` directory

### Data Validation Errors
- Validate JSON files against schema: `jsonschema -i data/perf_data_*.json schemas/perf_data_schema.json`
