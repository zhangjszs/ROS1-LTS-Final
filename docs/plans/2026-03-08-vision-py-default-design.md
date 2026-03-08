# Vision Python Default and Fusion Default Design

**Context:** The repository already contains a Python vision node and an existing LiDAR vision color injection path, but the default launch chain still prefers C++ vision and leaves vision fusion disabled.

**Decision:** Make Python vision the default implementation across the shared launch chain and enable LiDAR vision color injection by default, while preserving explicit overrides back to C++ or no-vision flows.

**Scope**
- Change `vision_ros/launch/vision.launch` default `impl` from `cpp` to `py`.
- Add `impl` pass-through to `fsd_launch/launch/subsystems/vision.launch`.
- Add `vision_impl` pass-through to shared mission launch entry points and default it to `py`.
- Change shared LiDAR base config `vision_inject.enabled` to `true`.

**Non-Goals**
- Do not remove the C++ vision implementation.
- Do not change the LiDAR-vision matching algorithm.
- Do not change message contracts or topic names.

**Risks**
- Shared launch defaults will change behavior for any consumer relying on previous defaults.
- Python vision currently remains less feature-complete than the C++ node, so switching defaults changes operational characteristics, not only implementation language.

**Validation**
- Build `vision_ros`, `perception_ros`, and `fsd_launch`.
- Launch the shared mission path and verify Python vision is the selected node by default.
- Verify LiDAR loads with `vision_inject/enabled` set to `true`.
