# Fusion Mainline Adapter Design

**Goal:** Introduce a thin adapter that converts LiDAR raw detections plus optional fused color results into one planner/localization-ready `HUAT_ConeDetections` stream, then route planning/localization to that stream without changing their algorithm code.

## Context

The current stack publishes three relevant perception topics:

- `perception/lidar_cluster/detections`: raw `HUAT_ConeDetections`
- `perception/fusion/detections`: fused `HUAT_FusedConeDetections`
- `localization/cone_map`: localization output used by high-speed planning

Line and skidpad planning subscribe directly to raw detections. Localization also subscribes to raw detections, so fused color semantics never reach localization or planning mainline behavior.

## Options Considered

### Option 1: Overwrite raw detections in `perception_ros`

Replace raw `color_types` in `HUAT_ConeDetections` before publishing.

- Pros: Fewest topics, smallest runtime graph.
- Cons: Destroys raw-vs-fused observability, breaks current raw topic semantics, makes debugging harder.

### Option 2: Add thin adapter/mux node and remap consumers

Keep raw and fused topics untouched. Add one node that publishes a standard `HUAT_ConeDetections` topic for downstream consumers.

- Pros: Preserves topic semantics, smallest safe behavior change, easy runtime fallback, launch-controlled rollout.
- Cons: One extra node and one new topic contract.

### Option 3: Inject fusion later at `HUAT_ConeMap`

Leave localization input raw and only modify the map/planning side.

- Pros: Smaller localization impact.
- Cons: Does not fix line/skidpad mainline, incomplete mainline repair.

## Decision

Use **Option 2**.

## Architecture

Add a new `perception_ros` node:

- Inputs:
  - raw `HUAT_ConeDetections`
  - fused `HUAT_FusedConeDetections`
- Output:
  - unified `HUAT_ConeDetections`

The adapter will use the raw message as the base payload and replace `color_types` with fused colors when a matching fused message arrives for the same detection stamp. If no matching fused message arrives within a small holdoff window, the adapter publishes the raw message unchanged.

## Data Flow

1. `lidar_cluster_ros` keeps publishing raw detections and fused detections exactly as today.
2. The adapter caches raw and fused messages by `header.stamp`.
3. If both messages exist for the same stamp and their cone counts match:
   - publish one unified `HUAT_ConeDetections`
   - geometry arrays come from raw
   - `color_types` come from `fused.fused_color_types`
4. If no fused match arrives before holdoff expiry:
   - publish the raw message unchanged
5. Planning/localization consumers are pointed at the unified topic through launch parameters, not by changing their internal defaults.

## Topic Contract

- `perception/lidar_cluster/detections`: raw-only LiDAR detections
- `perception/fusion/detections`: fusion/debug side output with association metadata
- `perception/decision/detections`: downstream decision input in standard `HUAT_ConeDetections` format

## Error Handling

- Missing fused message: fallback to raw after holdoff.
- Fused arrives before raw: cache until raw arrives or drop on timeout.
- Stamp match but cone counts differ: publish raw and log warning.
- Oversized caches: evict oldest pending entries with throttled warnings.

## Rollout

Wire the adapter into `fsd_launch` perception subsystem. Pass `perception/decision/detections` into:

- `location.launch` via `topics/cone`
- `planning_pipeline.launch` via `topics/cone` for line/skidpad

High-speed planning remains unchanged directly; it benefits indirectly through localization consuming unified detections.

## Testing

1. Unit test adapter behavior:
   - fused color overrides raw when both match
   - raw publishes after holdoff without fused
   - fused-first then raw still publishes merged output
   - stamp/count mismatch falls back to raw
2. Focused launch/config check:
   - perception subsystem launches adapter
   - localization/planning receive unified topic via args/params

## Non-Goals

- No change to fusion algorithm itself
- No change to localization/planning algorithms
- No attempt in this change to retune TF, sync, or camera calibration behavior
