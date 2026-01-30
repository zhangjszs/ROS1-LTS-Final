# Performance Reporting System - Thesis Guide

## Overview

This guide provides thesis-specific documentation for the performance reporting system, explaining the methodology, metrics, and how to present results in academic writing.

## Performance Metrics Explained

### Time Metrics

All time measurements are in milliseconds (ms):

- **t_pass_ms**: Point cloud filtering time - removes outliers and invalid points
- **t_ground_ms**: Ground segmentation time - separates ground from obstacles
- **t_cluster_ms**: Clustering time - groups points into cone candidates
- **t_delaunay_ms**: Delaunay triangulation time - builds track topology
- **t_way_ms**: Path planning time - generates waypoints
- **t_total_ms**: Total processing time - end-to-end pipeline latency

### Data Metrics

- **N**: Number of input points from LiDAR sensor
- **K**: Number of detected clusters (cone candidates)
- **T**: Number of Delaunay triangles in track topology
- **E**: Number of edges in track graph
- **D**: Number of waypoints in planned path
- **bytes**: Message size for ROS communication overhead

## Statistical Methodology

### Percentile Analysis

The system uses percentile-based analysis to understand performance distribution:

- **Mean**: Average performance across all samples
- **P50 (Median)**: 50% of samples are faster than this value
- **P95**: 95% of samples are faster - represents typical worst-case
- **P99**: 99% of samples are faster - represents rare worst-case
- **Max**: Absolute worst-case observed

**Why percentiles matter**: Mean can be misleading due to outliers. P95/P99 show real-world worst-case behavior that affects system reliability.

### Window-Based Statistics

Performance is measured over sliding windows (typically 100 samples) to:
- Smooth out transient spikes
- Capture steady-state behavior
- Enable trend analysis over time

## Benchmark Methodology

### Test Environment

Document these details in your thesis:

```yaml
Hardware:
  - CPU: [from system_info]
  - Memory: [from system_info]
  - Storage: SSD/HDD type

Software:
  - OS: Ubuntu 20.04 (ROS Noetic)
  - ROS Version: Noetic
  - Python: 3.8+
  - Key Libraries: PCL 1.10, Eigen 3.3

Sensor:
  - LiDAR: Velodyne VLP-16
  - Frequency: 10 Hz
  - Points per scan: ~30,000
```

### Test Scenarios

Standard test scenarios for reproducibility:

1. **Straight Track**: Simple acceleration test
2. **Hairpin Turns**: High curvature handling
3. **Skidpad**: Figure-8 maneuver
4. **Full Autocross**: Complex track with mixed features

### Data Collection Protocol

1. Record rosbag with all sensor data
2. Run system in playback mode (deterministic)
3. Collect performance logs with `[perf]` markers
4. Generate reports using `perf_tool.py full`
5. Repeat 3 times for statistical validity

## Visualization Interpretation

### Time Breakdown Chart

Shows relative time spent in each pipeline stage:
- Identify bottlenecks (tallest bars)
- Compare before/after optimization
- Understand computational budget allocation

### Percentile Distribution Chart

Shows performance consistency:
- Narrow distribution = consistent performance
- Wide distribution = high variance (investigate causes)
- P99/P95 ratio > 1.5 indicates tail latency issues

### Data Volume Statistics

Correlates input complexity with processing time:
- More points (N) → longer processing
- More clusters (K) → more complex scene
- Use for complexity analysis

### Time Distribution Pie Chart

Shows percentage breakdown:
- Useful for optimization prioritization
- Target largest slices for improvement
- Track changes over development

## Results Presentation Guidelines

### For Thesis Writing

**Performance Summary Table**:

| Metric | Mean | P95 | P99 | Unit |
|--------|------|-----|-----|------|
| Total Processing | 28.5 | 35.2 | 42.1 | ms |
| Ground Segmentation | 15.3 | 18.7 | 22.4 | ms |
| Clustering | 5.2 | 6.8 | 8.1 | ms |
| Path Planning | 4.1 | 5.3 | 6.7 | ms |

**Comparison Table** (Before/After Optimization):

| Metric | Before | After | Improvement |
|--------|--------|-------|-------------|
| Total (P95) | 45.2 ms | 35.2 ms | 22.1% ↓ |
| Ground (P95) | 28.1 ms | 18.7 ms | 33.5% ↓ |

### Key Findings Format

```
The perception pipeline achieves real-time performance with:
- Mean processing time: 28.5 ms (35 Hz capability)
- P95 latency: 35.2 ms (meets 10 Hz sensor rate with 3.5× margin)
- P99 latency: 42.1 ms (still within 50 ms deadline)

Ground segmentation is the primary bottleneck, consuming 53.7% of
total processing time. Optimization efforts focused on this stage
yielded 33.5% improvement in P95 latency.
```

## Performance Thresholds

Document your system requirements:

```yaml
Real-time Requirements:
  - Target frequency: 10 Hz (100 ms period)
  - Processing budget: 50 ms (50% duty cycle)
  - Safety margin: 2× (P95 < 50 ms)

Threshold Levels:
  - Optimal: < 30 ms (green)
  - Acceptable: 30-40 ms (yellow)
  - Warning: 40-50 ms (orange)
  - Critical: > 50 ms (red)
```

## Common Pitfalls

### 1. Reporting Only Mean
❌ "Average processing time is 28.5 ms"
✅ "Processing time: mean 28.5 ms, P95 35.2 ms, P99 42.1 ms"

### 2. Ignoring Variance
❌ "System runs at 35 Hz"
✅ "System achieves 35 Hz mean, with P95 at 28 Hz (35.2 ms)"

### 3. Cherry-Picking Data
❌ Using only best-case scenarios
✅ Report all test scenarios with their characteristics

### 4. Missing Context
❌ "Improved by 10 ms"
✅ "Reduced P95 latency from 45.2 ms to 35.2 ms (22.1% improvement)"

## Reproducibility Checklist

For thesis reviewers to reproduce your results:

- [ ] Hardware specifications documented
- [ ] Software versions specified
- [ ] Test rosbags available (or described in detail)
- [ ] Configuration files included
- [ ] Random seeds fixed (if applicable)
- [ ] Test protocol documented step-by-step
- [ ] Statistical significance reported (n=3 minimum)
- [ ] Outlier handling policy stated

## Academic Writing Tips

### Abstract/Introduction
- State real-time requirements upfront
- Mention key performance metrics (P95 latency)
- Highlight bottlenecks and optimizations

### Methodology
- Describe benchmark scenarios
- Explain percentile-based analysis
- Document test environment

### Results
- Use tables for numerical data
- Use charts for trends and comparisons
- Report both mean and percentiles
- Include error bars or confidence intervals

### Discussion
- Relate performance to system requirements
- Explain bottlenecks and their causes
- Discuss trade-offs (accuracy vs. speed)
- Compare with related work (if available)

## Example Thesis Sections

### Performance Evaluation Section

```markdown
## 5.3 Performance Evaluation

The perception system was evaluated on four standard test scenarios
using recorded rosbag data from the Formula Student Driverless track.
Performance metrics were collected over 100-sample sliding windows
and analyzed using percentile-based statistics.

### 5.3.1 Processing Latency

Table 5.1 shows the processing latency breakdown. The system achieves
a mean total processing time of 28.5 ms, well within the 50 ms budget
required for 10 Hz operation. The P95 latency of 35.2 ms provides a
2.8× safety margin, ensuring reliable real-time performance even under
worst-case conditions.

Ground segmentation is the dominant bottleneck, consuming 53.7% of
total processing time. This is expected given the algorithm's O(N)
complexity with respect to point cloud size.

### 5.3.2 Optimization Impact

Figure 5.3 compares performance before and after optimization. The
optimized ground segmentation algorithm reduced P95 latency from
28.1 ms to 18.7 ms (33.5% improvement), bringing total system P95
latency from 45.2 ms to 35.2 ms (22.1% improvement).
```

## References

For academic citations:

- Percentile-based performance analysis: Dean & Barroso (2013) "The Tail at Scale"
- Real-time systems: Buttazzo (2011) "Hard Real-Time Computing Systems"
- Statistical methodology: Jain (1991) "The Art of Computer Systems Performance Analysis"

## Tools and Commands

Generate thesis-ready reports:

```bash
# Full analysis with detailed notes
python3 scripts/perf_tool.py full \
  --note "Final thesis evaluation" \
  --scenario "Autocross track - 3 laps" \
  --tags "thesis,final,autocross"

# Comparison report for optimization chapter
python3 scripts/perf_tool.py report --compare --commits "before,after"

# Timeline report for development progress
python3 scripts/perf_tool.py report --timeline
```

## Contact

For questions about performance methodology or thesis presentation:
- Review ARCHITECTURE.md for system design
- Check README.md for usage instructions
- See QUICKSTART.md for getting started
