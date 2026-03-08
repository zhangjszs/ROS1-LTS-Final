# Regression Metrics Baseline (2026-02-13-step3)

## Coupling Indicators

| Metric | Value | Description |
|---|---:|---|
| Legacy alias refs | 23 | Code/launch references to old topic aliases |
| Absolute subscribe refs | 1 | subscribe("/...") hard-coded absolute subscriptions |
| Launch remap entries | 5 | <remap ...> usage count in launch files |
| TF sendTransform calls | 2 | Runtime TF broadcasters in source code |
| Fat source files (>500 LOC) | 13 | Large source files count |
| Fat ROS source files (>500 LOC) | 5 | Large ROS wrapper source files |
| perception_ros param loads | 222 | param(...) usages in lidar wrapper |
| localization_ros param loads | 116 | LoadParam/param(...) usages in location node |
| control_ros legacy refs | 17 | Legacy topic refs in control package |

## Artifact Files

- `runtime_endpoints_snapshot.txt`
- `static_package_deps_snapshot.txt`
- `legacy_alias_refs_snapshot.txt`
- `all_source_loc.txt`
- `top_source_files_by_loc.txt`

## Reproduce

```bash
bash scripts/export_architecture_baseline.sh 2026-02-13-step3
```
