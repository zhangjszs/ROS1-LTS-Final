# Vision-based cone detection results
# Publishes independently; lidar_cluster_ros optionally subscribes for color injection.

Header header                    # original image timestamp
uint8 image_quality              # 0=GOOD, 1=DEGRADED, 2=POOR, 3=UNUSABLE
float32 quality_score            # 0.0~1.0 composite quality

float64[] x                      # bbox center x (pixels)
float64[] y                      # bbox center y (pixels)
uint8[] color_types              # same enum as HUAT_ConeDetections: 0=BLUE,1=YELLOW,2=ORANGE_SMALL,3=ORANGE_BIG,4=NONE,5=RED
int32[] confidences              # 0~1000 (scaled, consistent with LiDAR side)
float32[] bbox_widths            # pixels
float32[] bbox_heights           # pixels

string backend_name              # "onnx" / "tensorrt" / "fallback_hsv"
bool fallback_active             # true if fallback method is in use
uint32 inference_time_us         # inference latency in microseconds
