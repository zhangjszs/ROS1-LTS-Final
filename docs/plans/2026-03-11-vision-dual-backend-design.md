# Vision Dual Backend Design

**Context:** `vision_ros` currently ships a Python node that supports ONNX Runtime inference plus HSV fallback, but the active parser assumes one YOLO-style tensor layout. The repository already contains both `/home/kerwin/2025huat/src/vision_ros/models/yolo26l_48g/weights/best.onnx` and `/home/kerwin/2025huat/src/vision_ros/models/yolo26l_48g/weights/best.pt`, and the local environment already has `onnxruntime`, `torch`, and `ultralytics` installed.

**Problem:** The checked-in `best.onnx` loads successfully, but its output shape is `[1, 300, 6]`, which does not match the parser currently implemented in `vision_node_py.py`. The node also cannot run `best.pt` directly, so there is no supported escape hatch when ONNX inference is unstable onsite.

**Decision:** Extend the Python node to support two explicit model backends:
- `onnx`: keep ONNX Runtime support and add output parsing for both the existing raw YOLO tensor layout and end-to-end detection outputs shaped like `[1, N, 6]` or `[N, 6]`
- `yolo_pt`: load a `.pt` model through `ultralytics.YOLO` and convert inference results into the existing internal `Detection` structure

**Scope**
- Modify `src/vision_ros/scripts/vision_node_py.py` to support `backend_type: yolo_pt`
- Fix ONNX postprocessing so `best.onnx` in `yolo26l_48g/weights/` is accepted without changing the ROS message contract
- Keep quality assessment, image enhancement, HSV fallback, tracker, diagnostics topic, and output topic names unchanged
- Add or update local config overlays so the node can be switched between the new `best.onnx` and `best.pt`
- Add regression tests for ONNX output parsing, backend initialization, and PT result conversion

**Non-Goals**
- Do not change the C++ vision node
- Do not change `HUAT_VisionDetections`
- Do not add TensorRT or other new backends
- Do not retrain or re-export the YOLO model as part of this task

**Architecture**
- Keep the current worker loop and shared post-inference pipeline intact
- Split backend initialization into three explicit modes: `onnx`, `yolo_pt`, `fallback_only`
- Normalize all backend outputs into the existing `Detection` objects before fusion/tracking/publish
- Validate backend/model_path combinations early so misconfiguration fails loudly when `node/require_model` is true

**Data Flow**
1. Receive image and run the existing quality assessment and enhancement steps
2. Select the configured backend
3. Produce model detections as normalized `Detection` objects
4. Fuse with HSV fallback detections if required
5. Apply the existing tracker, top-k filter, and publishers

**Backend Parsing Rules**
- ONNX end-to-end outputs:
  - Accept `[1, N, 6]` and `[N, 6]`
  - Interpret columns as `x1, y1, x2, y2, score, class_id`
  - Filter by confidence threshold and map model classes through `class_to_color`
- ONNX raw YOLO outputs:
  - Keep support for `cx, cy, w, h + class scores`
  - Continue auto-transposing anchors-first outputs into features-first layout
- YOLO PT outputs:
  - Use `ultralytics.YOLO(...).predict(...)`
  - Read `boxes.xyxy`, `boxes.conf`, and `boxes.cls`
  - Convert into the same center-width-height `Detection` representation used by the rest of the node

**Error Handling**
- Initialization failures for `onnx` and `yolo_pt` follow the existing `require_model` behavior: fatal when true, warning and fallback-only when false
- Runtime inference exceptions are throttled and treated as empty model detections for that frame
- Explicitly reject mismatched file extensions when the backend requires a specific model type

**Validation**
- Run focused Python regression tests for ONNX `[N, 6]` parsing and PT conversion
- Verify the Python node can initialize with:
  - `backend_type=onnx` and `model_path=.../yolo26l_48g/weights/best.onnx`
  - `backend_type=yolo_pt` and `model_path=.../yolo26l_48g/weights/best.pt`
- Run the existing `vision_ros` test target or focused rostest coverage after code changes

**Risks**
- `ultralytics` result objects vary slightly across versions, so conversion code should use the stable `boxes` API only
- The ONNX export may already contain post-NMS detections, so parsing must not reapply YOLO-style class score logic to `[N, 6]`
- The user has uncommitted workspace changes in `vision_ros`, so edits must be incremental and avoid reverting unrelated modifications
