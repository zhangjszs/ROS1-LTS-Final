# Mission-Specific Fusion Budget Design

## Goal

Introduce a single-source mission-level budget parameter, `decision_fusion_budget_sec`, so each mission can choose its own legacy fusion wait budget without changing adapter semantics or reintroducing multiple timing contracts.

## Design

`decision_fusion_budget_sec` is defined at mission entry launches and passed through unchanged:
- mission entry launch
- `mission_stack.launch`
- `perception.launch`
- `lidar_cluster.launch`

At the LiDAR cluster wrapper, the shared mission budget is mapped to the existing legacy contract key:
- `vision_inject.max_age_sec <- decision_fusion_budget_sec`

The adapter continues to derive holdoff from the runtime contract exactly as it does today. No state-machine, TF, sync-policy, or calibrated-fusion behavior changes in this patch.

## Initial Mission Matrix

- `acceleration`: `0.10s`
- `trackdrive`: `0.12s`
- `autocross`: `0.12s`
- `skidpad`: `0.15s`

## Fallback

If a mission entry does not provide `decision_fusion_budget_sec`, the stack falls back to the current global default `0.15s`.

## Non-Goals

- No new adapter timing modes
- No calibrated-fusion integration changes
- No secondary budget names such as separate adapter/perception/planning holdoff constants
