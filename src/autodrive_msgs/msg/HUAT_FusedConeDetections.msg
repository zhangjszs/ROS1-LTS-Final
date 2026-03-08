#
# LiDAR-Vision fused cone detections (late fusion output).
# Geometry remains LiDAR-native, while semantic/color fields come from vision matching.
#

Header header

# Frame metadata
string lidar_frame
string camera_frame

# Temporal sync metadata
time lidar_stamp
time vision_stamp
float32 sync_delta_sec

# Fusion prerequisites
bool camera_info_available
bool tf_available

# Per-cone LiDAR geometry / attributes (same ordering as associated arrays below)
geometry_msgs/Point32[] points
float32[] obj_dist
float32[] lidar_confidences
uint8[] lidar_color_types
int32[] track_ids

# Per-cone fused outputs
uint8[] fused_color_types
int32[] vision_confidences
uint8[] association_status
float32[] association_scores
string[] association_reasons

# Frame summary
uint32 matched_count
uint32 unmatched_count

# association_status enum:
# 0 = MATCHED
# 1 = NO_SYNC_PAIR
# 2 = CAMERA_INFO_MISSING
# 3 = TF_MISSING
# 4 = BEHIND_CAMERA
# 5 = OUT_OF_IMAGE
# 6 = LOW_VISION_CONFIDENCE
# 7 = NO_BBOX_MATCH
# 8 = LEGACY_HFOV_MATCH
