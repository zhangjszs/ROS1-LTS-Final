#!/usr/bin/env python3
import rospy
import numpy as np
import open3d as o3d
import time
import cv2
from ultralytics import YOLO
import threading
from queue import Queue
from collections import deque

from sensor_msgs.msg import PointCloud2, PointField, Image
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs.point_cloud2 as pc2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseArray, Pose, Point
from std_msgs.msg import Header, ColorRGBA
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray
from perf_stats import PerfStats, estimate_message_bytes

class MultiModalFusion:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('multimodal_fusion', anonymous=True)
        # 读取ROS参数
        self.sensor_model = rospy.get_param('~sensor_model', 'HDL-32E')
        self.print_fps = rospy.get_param('~print_fps', True)
        self.leaf = rospy.get_param('~leaf', 1)
        self.z_axis_min = rospy.get_param('~z_axis_min', -0.5)
        self.z_axis_max = rospy.get_param('~z_axis_max', 1.5)
        self.cluster_size_min = rospy.get_param('~cluster_size_min', 10)
        self.cluster_size_max = rospy.get_param('~cluster_size_max', 2000)
        self.cluster_tolerance_base = rospy.get_param('~cluster_tolerance_base', 0.1)
        self.cluster_tolerance_step = rospy.get_param('~cluster_tolerance_step', 0.1)
        self.camera_topic = rospy.get_param('~camera_topic', '/camera/color/image_raw')
        self.lidar_topic = rospy.get_param('~lidar_topic', '/livox/lidar')
        self.yolo_confidence = rospy.get_param('~yolo_confidence', 0.5)
        self.yolo_model_path = rospy.get_param('~yolo_model_path', 'yolo11s.pt')
        self.publish_markers = rospy.get_param('~publish_markers', False)

        """realsense 相机"""
        # 相机参数
        self.camera_matrix = np.array([
            [909.783, 0, 650.365],
            [0, 909.004, 381.295],
            [0, 0, 1]
        ])
        self.dist_coeffs = np.zeros(5)  # 无畸变
        self.image_width = 1280  # 图像宽度
        self.image_height = 720  # 图像高度
        
        # 雷达到相机的变换矩阵
        self.lidar_to_camera = np.array([
            [0.0, -1.0, 0.0, 0.03],
            [0.0, 0.0, -1.0, -0.022],
            [1.0, 0.0, 0.0, -0.04],
            [0, 0, 0, 1]
        ])
        
        """广角相机"""
        # 相机内参
        # self.camera_matrix = np.array([
        #     [801.685, 0, 645.146],
        #     [0, 800.967, 354.575],
        #     [0, 0, 1]
        # ])
        # self.dist_coeffs = np.array([-0.399, 0.208, 0, 0, -0.070])  # 畸变系数
        # self.image_width = 1280  # 图像宽度
        # self.image_height = 720  # 图像高度
        
        # # 雷达到相机的变换矩阵
        # self.lidar_to_camera = np.array([
        #     [0.0, -1.0, 0.0, 0.0],
        #     [0.342, 0.0, -0.939, -0.1],
        #     [0.939, 0.0, 0.342, -0.15],
        #     [0, 0, 0, 1]
        # ])

        # 初始化点云聚类参数
        self.region_max = 10
        self.regions = np.zeros(100, dtype=int)
        self.init_regions()
        
        # 性能统计
        self.frames = 0
        self.start_time = 0
        self.reset = True
        
        # 用于点云/图像处理的线程安全队列
        self.lidar_queue = Queue(maxsize=1)
        self.image_queue = Queue(maxsize=1)
        self.fusion_results = Queue(maxsize=10)
        
        # 初始化CV Bridge
        self.bridge = CvBridge()
        
        # 加载YOLO模型
        rospy.loginfo("正在加载YOLO模型")
        try:
            self.model = YOLO(self.yolo_model_path)
            rospy.loginfo("YOLO模型加载完成")
        except Exception as e:
            rospy.logerr(f"加载YOLO模型失败: {e}")
            self.model = None

        perf_enabled = rospy.get_param('~perf_stats_enable', True)
        perf_window = rospy.get_param('~perf_stats_window', 300)
        perf_log_every = rospy.get_param('~perf_stats_log_every', 30)
        self.perf_stats = PerfStats(
            "fusion_detection", window_size=perf_window, log_every=perf_log_every, enabled=perf_enabled
        )
        
        # 设置点云处理线程
        self.lidar_thread = threading.Thread(target=self.process_lidar_thread)
        self.lidar_thread.daemon = True
        self.lidar_thread.start()
        
        # 设置图像处理线程
        self.image_thread = threading.Thread(target=self.process_image_thread)
        self.image_thread.daemon = True
        self.image_thread.start()
        
        # 设置融合处理线程
        self.fusion_thread = threading.Thread(target=self.fusion_thread)
        self.fusion_thread.daemon = True
        self.fusion_thread.start()
        
        # 设置订阅者
        self.lidar_sub = rospy.Subscriber(self.lidar_topic, PointCloud2, self.lidar_callback, queue_size=1)
        self.image_sub = rospy.Subscriber(self.camera_topic, Image, self.image_callback, queue_size=1)
        
        # 设置发布者
        self.cloud_filtered_pub = rospy.Publisher('~cloud_filtered', PointCloud2, queue_size=1)
        self.cluster_marker_pub = None
        if self.publish_markers:
            self.cluster_marker_pub = rospy.Publisher('~cluster_markers', MarkerArray, queue_size=1)
        self.pose_array_pub = rospy.Publisher('~cluster_poses', PoseArray, queue_size=1)
        self.detection_image_pub = rospy.Publisher('~detection_image', Image, queue_size=1)
        self.bbox3d_pub = rospy.Publisher('~bounding_boxes3d', BoundingBoxArray, queue_size=1)
        
        rospy.loginfo("多模态融合节点初始化完成")
    
    def init_regions(self):
        """初始化不同传感器的区域大小"""
        if self.sensor_model == "VLP-16":
            self.regions[0:14] = [2, 3, 3, 3, 3, 3, 3, 2, 3, 3, 3, 3, 3, 3]
        elif self.sensor_model == "HDL-32E":
            self.regions[0:14] = [4, 5, 4, 5, 4, 5, 5, 4, 5, 4, 5, 5, 4, 5]
        elif self.sensor_model == "HDL-64E":
            self.regions[0:5] = [14, 14, 14, 15, 14]
        else:
            rospy.logfatal("Unknown sensor model!")
    
    def lidar_callback(self, lidar_msg):
        """LiDAR点云回调函数"""
        # 将消息放入队列
        if not self.lidar_queue.full():
            self.lidar_queue.put(lidar_msg)
    
    def image_callback(self, image_msg):
        """图像回调函数"""
        # 将消息放入队列
        if not self.image_queue.full():
            self.image_queue.put(image_msg)
    
    def process_lidar_thread(self):
        """处理LiDAR数据的线程"""
        while not rospy.is_shutdown():
            try:
                if not self.lidar_queue.empty():
                    lidar_msg = self.lidar_queue.get()
                    
                    # 将ROS点云转换为numpy数组
                    pc_array = self.ros_to_numpy(lidar_msg)
                    
                    # 处理点云
                    filtered_points, centroids, boxes, perf = self.process_point_cloud(pc_array)
                    
                    # 发布过滤后的点云
                    bytes_pub = 0
                    if self.cloud_filtered_pub.get_num_connections() > 0:
                        filtered_cloud_msg = self.numpy_to_ros(lidar_msg.header, filtered_points)
                        self.cloud_filtered_pub.publish(filtered_cloud_msg)
                        bytes_pub += estimate_message_bytes(filtered_cloud_msg)
                    
                    # 发布未标记的聚类
                    self.publish_cluster_markers(lidar_msg.header, centroids, boxes, None)
                    perf["bytes"] = float(bytes_pub)
                    
                    # 存储处理结果，等待与图像检测结果融合
                    lidar_data = {
                        'timestamp': lidar_msg.header.stamp,
                        'frame_id': lidar_msg.header.frame_id,
                        'header': lidar_msg.header,
                        'centroids': centroids,
                        'boxes': boxes,
                        'perf': perf
                    }
                    
                    # 结果放入融合队列
                    if not self.fusion_results.full():
                        self.fusion_results.put(('lidar', lidar_data))
                
                time.sleep(0.001)  # 避免CPU占用过高
            except Exception as e:
                rospy.logerr(f"点云处理线程错误: {e}")
    
    def process_image_thread(self):
        """处理图像数据的线程"""
        while not rospy.is_shutdown():
            try:
                if not self.image_queue.empty() and self.model is not None:
                    image_msg = self.image_queue.get()
                    image_start = time.time()
                    
                    # 将ROS图像转换为OpenCV格式
                    try:
                        cv_img = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
                    except CvBridgeError as e:
                        rospy.logerr(f"CV Bridge错误: {e}")
                        continue
                    
                    # 运行YOLO目标检测
                    results = self.model(cv_img, conf=self.yolo_confidence)
                    
                    # 创建处理后的图像
                    processed_img = cv_img.copy()
                    
                    # 提取检测结果
                    yolo_detections = []
                    
                    for result in results:
                        boxes = result.boxes
                        for box in boxes:
                            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                            conf = box.conf[0].item()
                            cls = int(box.cls[0].item())
                            class_name = self.model.names[cls]
                            
                            # 转换为整数坐标
                            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                            
                            # 绘制边界框和标签
                            label = f'{class_name} {conf:.2f}'
                            cv2.rectangle(processed_img, (x1, y1), (x2, y2), (255, 0, 0), 2)
                            cv2.putText(processed_img, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                            
                            yolo_detections.append({
                                'bbox': (x1, y1, x2, y2),
                                'confidence': conf,
                                'class_id': cls,
                                'class_name': class_name
                            })
                    
                    image_time_ms = (time.time() - image_start) * 1000.0
                    
                    # 发布处理后的图像
                    try:
                        detection_image_msg = self.bridge.cv2_to_imgmsg(processed_img, "bgr8")
                        detection_image_msg.header = image_msg.header
                        self.detection_image_pub.publish(detection_image_msg)
                    except CvBridgeError as e:
                        rospy.logerr(f"CV Bridge错误: {e}")
                    
                    # 存储处理结果，等待与点云聚类结果融合
                    image_data = {
                        'timestamp': image_msg.header.stamp,
                        'frame_id': image_msg.header.frame_id,
                        'header': image_msg.header,
                        'detections': yolo_detections,
                        'image': cv_img,
                        'processed_image': processed_img,
                        'perf': {
                            't_total_ms': image_time_ms,
                            'D': float(len(yolo_detections))
                        }
                    }
                    
                    # 结果放入融合队列
                    if not self.fusion_results.full():
                        self.fusion_results.put(('image', image_data))
                
                time.sleep(0.001)  # 避免CPU占用过高
            except Exception as e:
                rospy.logerr(f"图像处理线程错误: {e}")
    
    def fusion_thread(self):
        """融合LiDAR和图像数据的线程"""
        lidar_data = None
        image_data = None
        
        while not rospy.is_shutdown():
            try:
                if not self.fusion_results.empty():
                    data_type, data = self.fusion_results.get()
                    
                    if data_type == 'lidar':
                        lidar_data = data
                    elif data_type == 'image':
                        image_data = data
                    
                    # 检查是否有足够的数据进行融合
                    if lidar_data is not None and image_data is not None:
                        # 检查时间戳是否足够接近
                        lidar_time = lidar_data['timestamp']
                        image_time = image_data['timestamp']
                        time_diff = abs((lidar_time - image_time).to_sec())
                        
                        if time_diff < 0.1:  # 100ms内的数据视为同步
                            self.fuse_lidar_and_image(lidar_data, image_data)
                            # 清除已处理的数据
                            lidar_data = None
                            image_data = None
                        elif lidar_time < image_time:
                            # LiDAR数据太旧，丢弃
                            lidar_data = None
                        else:
                            # 图像数据太旧，丢弃
                            image_data = None
                
                time.sleep(0.001)  # 避免CPU占用过高
            except Exception as e:
                rospy.logerr(f"融合线程错误: {e}")
    
    def fuse_lidar_and_image(self, lidar_data, image_data):
        """融合LiDAR聚类和图像检测结果"""
        try:
            fusion_start = time.time()
            bytes_pub = 0
            # 获取变换矩阵 (使用提供的雷达到相机的变换矩阵)
            transform_matrix = self.lidar_to_camera
            
            # 从聚类中提取边界框
            centroids = lidar_data['centroids']
            boxes = lidar_data['boxes']
            detections = image_data['detections']
            cv_image = image_data['processed_image'].copy()  # 使用已处理的图像
            
            # 创建带有类别的3D边界框消息
            bbox3d_array = BoundingBoxArray()
            bbox3d_array.header = lidar_data['header']
            
            # 用于标记的聚类
            classified_clusters = []
            
            # 处理每个3D边界框
            for cluster_idx, box in enumerate(boxes):
                min_point = box['min']
                max_point = box['max']
                
                # 创建边界框角点
                corners = np.array([
                    [min_point[0], min_point[1], min_point[2], 1],
                    [max_point[0], min_point[1], min_point[2], 1],
                    [max_point[0], max_point[1], min_point[2], 1],
                    [min_point[0], max_point[1], min_point[2], 1],
                    [min_point[0], min_point[1], max_point[2], 1],
                    [max_point[0], min_point[1], max_point[2], 1],
                    [max_point[0], max_point[1], max_point[2], 1],
                    [min_point[0], max_point[1], max_point[2], 1]
                ])
                
                # 将角点从LiDAR坐标系转换到相机坐标系
                camera_corners = []
                for corner in corners:
                    # 应用变换
                    camera_corner = np.dot(transform_matrix, corner)
                    camera_corners.append(camera_corner[:3])
                
                camera_corners = np.array(camera_corners)
                
                # 将角点从相机3D坐标投影到图像平面
                image_points, _ = cv2.projectPoints(
                    camera_corners,
                    np.zeros(3),  # 已在坐标系中，无需旋转
                    np.zeros(3),  # 已在坐标系中，无需平移
                    self.camera_matrix,
                    self.dist_coeffs
                )
                image_points = image_points.reshape(-1, 2)
                
                # 检查点是否在图像前方（Z > 0）
                if np.any(np.array([p[2] for p in camera_corners]) <= 0):
                    continue  # 边界框在相机后方，跳过
                
                # 将3D边界框投影到图像上
                # 绘制底面
                for i in range(4):
                    pt1 = (int(image_points[i][0]), int(image_points[i][1]))
                    pt2 = (int(image_points[(i+1)%4][0]), int(image_points[(i+1)%4][1]))
                    cv2.line(cv_image, pt1, pt2, (0, 0, 255), 2)
                
                # 绘制顶面
                for i in range(4):
                    pt1 = (int(image_points[i+4][0]), int(image_points[i+4][1]))
                    pt2 = (int(image_points[((i+1)%4)+4][0]), int(image_points[((i+1)%4)+4][1]))
                    cv2.line(cv_image, pt1, pt2, (0, 0, 255), 2)
                
                # 绘制侧面
                for i in range(4):
                    pt1 = (int(image_points[i][0]), int(image_points[i][1]))
                    pt2 = (int(image_points[i+4][0]), int(image_points[i+4][1]))
                    cv2.line(cv_image, pt1, pt2, (0, 0, 255), 2)
                
                # 计算边界框在图像中的2D投影包围框
                x_coords = image_points[:, 0]
                y_coords = image_points[:, 1]
                
                # 如果边界框完全超出图像，则跳过
                if np.all(x_coords < 0) or np.all(x_coords >= self.image_width) or \
                   np.all(y_coords < 0) or np.all(y_coords >= self.image_height):
                    continue
                
                # 将超出图像范围的坐标限制在图像边界内
                x_coords = np.clip(x_coords, 0, self.image_width - 1)
                y_coords = np.clip(y_coords, 0, self.image_height - 1)
                
                x_min = int(np.min(x_coords))
                y_min = int(np.min(y_coords))
                x_max = int(np.max(x_coords))
                y_max = int(np.max(y_coords))
                
                # 与2D检测框进行融合
                best_iou = 0
                best_detection = None
                
                for detection in detections:
                    det_x1, det_y1, det_x2, det_y2 = detection['bbox']
                    
                    # 计算IoU
                    x_left = max(x_min, det_x1)
                    y_top = max(y_min, det_y1)
                    x_right = min(x_max, det_x2)
                    y_bottom = min(y_max, det_y2)
                    
                    if x_right < x_left or y_bottom < y_top:
                        continue  # 没有重叠
                    
                    intersection_area = (x_right - x_left) * (y_bottom - y_top)
                    box1_area = (x_max - x_min) * (y_max - y_min)
                    box2_area = (det_x2 - det_x1) * (det_y2 - det_y1)
                    iou = intersection_area / float(box1_area + box2_area - intersection_area)
                    
                    if iou > best_iou and iou > 0.3:  # IoU阈值
                        best_iou = iou
                        best_detection = detection
                
                # 如果找到匹配的检测，标记聚类
                if best_detection is not None:
                    # 获取类别信息
                    class_name = best_detection['class_name']
                    confidence = best_detection['confidence']
                    
                    # 在3D边界框中心绘制类别标签
                    centroid = np.mean(image_points, axis=0).astype(int)
                    label = f"{class_name}: {confidence:.2f}"
                    cv2.putText(cv_image, label, (centroid[0], centroid[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
                    
                    # 添加到已分类的聚类列表
                    classified_clusters.append({
                        'cluster_idx': cluster_idx,
                        'class_name': class_name,
                        'confidence': confidence,
                        'box': box
                    })
                    
                    # 创建3D边界框消息
                    bbox3d = BoundingBox()
                    bbox3d.header = lidar_data['header']
                    bbox3d.pose.position.x = (min_point[0] + max_point[0]) / 2
                    bbox3d.pose.position.y = (min_point[1] + max_point[1]) / 2
                    bbox3d.pose.position.z = (min_point[2] + max_point[2]) / 2
                    bbox3d.pose.orientation.w = 1.0  # 单位四元数，无旋转
                    bbox3d.dimensions.x = max_point[0] - min_point[0]
                    bbox3d.dimensions.y = max_point[1] - min_point[1]
                    bbox3d.dimensions.z = max_point[2] - min_point[2]
                    bbox3d.value = best_detection['class_id']  # 使用类别ID作为值
                    bbox3d.label = best_detection['class_id']
                    
                    bbox3d_array.boxes.append(bbox3d)
            
            # 发布3D边界框消息
            self.bbox3d_pub.publish(bbox3d_array)
            bytes_pub += estimate_message_bytes(bbox3d_array)
            
            # 发布带有分类信息的聚类标记
            self.publish_cluster_markers(lidar_data['header'], centroids, lidar_data['boxes'], classified_clusters)
            
            # 发布融合后的图像
            try:
                fusion_image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
                fusion_image_msg.header = image_data['header']
                self.detection_image_pub.publish(fusion_image_msg)
                bytes_pub += estimate_message_bytes(fusion_image_msg)
            except CvBridgeError as e:
                rospy.logerr(f"CV Bridge错误: {e}")

            fusion_time_ms = (time.time() - fusion_start) * 1000.0
            lidar_perf = lidar_data.get('perf', {})
            image_perf = image_data.get('perf', {})
            sample = {
                "t_pass_ms": float(lidar_perf.get("t_pass_ms", 0.0)),
                "t_cluster_ms": float(lidar_perf.get("t_cluster_ms", 0.0)),
                "t_total_ms": float(lidar_perf.get("t_total_ms", 0.0))
                + float(image_perf.get("t_total_ms", 0.0))
                + fusion_time_ms,
                "N": float(lidar_perf.get("N", 0.0)),
                "K": float(lidar_perf.get("K", 0.0)),
                "D": float(image_perf.get("D", 0.0)),
                "bytes": float(lidar_perf.get("bytes", 0.0)) + float(bytes_pub),
            }
            self.perf_stats.add(sample)
                
        except Exception as e:
            rospy.logerr(f"融合处理错误: {e}")
    
    def _pointfield_dtype(self, datatype, is_bigendian):
        type_map = {
            PointField.INT8: np.int8,
            PointField.UINT8: np.uint8,
            PointField.INT16: np.int16,
            PointField.UINT16: np.uint16,
            PointField.INT32: np.int32,
            PointField.UINT32: np.uint32,
            PointField.FLOAT32: np.float32,
            PointField.FLOAT64: np.float64,
        }
        np_type = type_map.get(datatype, np.float32)
        dtype = np.dtype(np_type)
        return dtype.newbyteorder('>' if is_bigendian else '<')

    def ros_to_numpy(self, ros_point_cloud):
        """将ROS点云转换为numpy数组"""
        if ros_point_cloud.width == 0 or ros_point_cloud.height == 0:
            return np.zeros((0, 4), dtype=np.float32)
        field_map = {f.name: f for f in ros_point_cloud.fields}
        if not {"x", "y", "z"}.issubset(field_map):
            return np.zeros((0, 4), dtype=np.float32)

        if ros_point_cloud.row_step != ros_point_cloud.point_step * ros_point_cloud.width:
            field_names = ("x", "y", "z", "intensity") if "intensity" in field_map else ("x", "y", "z")
            points_list = []
            for point in pc2.read_points(ros_point_cloud, field_names=field_names, skip_nans=True):
                points_list.append(point)
            points = np.array(points_list, dtype=np.float32)
            if points.size == 0:
                return np.zeros((0, 4), dtype=np.float32)
            if points.shape[1] == 3:
                zeros = np.zeros((points.shape[0], 1), dtype=np.float32)
                points = np.hstack([points, zeros])
            return points

        names = []
        formats = []
        offsets = []
        for name in ("x", "y", "z", "intensity"):
            field = field_map.get(name)
            if field is None:
                continue
            names.append(name)
            formats.append(self._pointfield_dtype(field.datatype, ros_point_cloud.is_bigendian))
            offsets.append(field.offset)

        dtype = np.dtype({
            "names": names,
            "formats": formats,
            "offsets": offsets,
            "itemsize": ros_point_cloud.point_step,
        })
        data = np.frombuffer(ros_point_cloud.data, dtype=dtype)
        points = np.zeros((data.shape[0], 4), dtype=np.float32)
        points[:, 0] = data["x"]
        points[:, 1] = data["y"]
        points[:, 2] = data["z"]
        if "intensity" in data.dtype.names:
            points[:, 3] = data["intensity"]
        mask = np.isfinite(points[:, 0]) & np.isfinite(points[:, 1]) & np.isfinite(points[:, 2])
        return points[mask]
    
    def numpy_to_ros(self, header, points):
        """将numpy数组转换为ROS点云"""
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('intensity', 12, PointField.FLOAT32, 1),
        ]
        
        # 确保点云有intensity字段，如果没有则添加默认值0
        if points.shape[1] == 3:
            points_with_intensity = np.column_stack([points, np.zeros(len(points))])
        else:
            points_with_intensity = points
            
        return pc2.create_cloud(header, fields, points_with_intensity)
        
    def filter_point_cloud(self, points):
        """降采样和高度过滤"""
        if points.size == 0:
            return points, np.empty((0,), dtype=np.int32)
        stride = max(int(self.leaf), 1)
        indices = np.arange(0, points.shape[0], stride, dtype=np.int32)
        sampled = points[indices]
        z_vals = sampled[:, 2]
        mask = (z_vals >= self.z_axis_min) & (z_vals <= self.z_axis_max) & np.isfinite(z_vals)
        filtered_points = sampled[mask]
        return filtered_points, indices[mask]
    
    def divide_into_regions(self, points_xyz, indices):
        """将点云分成嵌套的环形区域"""
        indices_array = [np.empty((0,), dtype=np.int32) for _ in range(self.region_max)]
        if indices.size == 0:
            return indices_array
        points = points_xyz[indices]
        distances = np.linalg.norm(points, axis=1)
        boundaries = np.cumsum(self.regions[:self.region_max])
        region_ids = np.searchsorted(boundaries, distances, side="right")
        valid = region_ids < self.region_max
        if not np.any(valid):
            return indices_array
        region_ids = region_ids[valid]
        region_indices = indices[valid]
        for i in range(self.region_max):
            mask = region_ids == i
            if np.any(mask):
                indices_array[i] = region_indices[mask]
        return indices_array
    
    def euclidean_clustering(self, points_xyz, indices_array):
        """对每个区域执行欧几里得聚类"""
        centroids = []
        boxes = []

        for i in range(self.region_max):
            tolerance = self.cluster_tolerance_base + i * self.cluster_tolerance_step
            region_indices = indices_array[i]
            if region_indices.size < self.cluster_size_min:
                continue

            region_points = points_xyz[region_indices]
            if region_points.shape[0] == 0:
                continue

            region_pcd = o3d.geometry.PointCloud()
            region_pcd.points = o3d.utility.Vector3dVector(region_points)
            kdtree = o3d.geometry.KDTreeFlann(region_pcd)
            visited = np.zeros(region_points.shape[0], dtype=bool)

            for idx in range(region_points.shape[0]):
                if visited[idx]:
                    continue
                queue = deque([idx])
                visited[idx] = True
                cluster_indices = []
                while queue:
                    current = queue.popleft()
                    cluster_indices.append(current)
                    _, neighbors, _ = kdtree.search_radius_vector_3d(region_points[current], tolerance)
                    for nb in neighbors:
                        if not visited[nb]:
                            visited[nb] = True
                            queue.append(nb)

                if self.cluster_size_min <= len(cluster_indices) <= self.cluster_size_max:
                    cluster_points = region_points[cluster_indices]
                    min_point = np.min(cluster_points, axis=0)
                    max_point = np.max(cluster_points, axis=0)
                    centroid = np.mean(cluster_points, axis=0)
                    boxes.append({"min": min_point, "max": max_point})
                    centroids.append(centroid)

        return centroids, boxes
    
    def process_point_cloud(self, points):
        """处理点云的主函数"""
        start_time = time.time()
        # 1. 降采样和高度过滤
        pass_start = time.time()
        filtered_points, indices = self.filter_point_cloud(points)
        t_pass_ms = (time.time() - pass_start) * 1000.0
        # 2. 点云分区
        cluster_start = time.time()
        points_xyz = points[:, :3] if points.size > 0 else points.reshape((-1, 3))
        indices_array = self.divide_into_regions(points_xyz, indices)
        # 3. 欧几里得聚类
        centroids, boxes = self.euclidean_clustering(points_xyz, indices_array)
        t_cluster_ms = (time.time() - cluster_start) * 1000.0
        t_total_ms = (time.time() - start_time) * 1000.0
        perf = {
            "t_pass_ms": t_pass_ms,
            "t_cluster_ms": t_cluster_ms,
            "t_total_ms": t_total_ms,
            "N": float(points.shape[0]),
            "K": float(len(boxes)),
        }
        return filtered_points, centroids, boxes, perf
    
    def publish_cluster_markers(self, header, centroids, boxes, classified_clusters):
        if not self.publish_markers or self.cluster_marker_pub is None:
            return
        """发布聚类标记"""
        clear_marker = Marker()
        clear_marker.header = header
        clear_marker.ns = "adaptive_clustering_labels"
        clear_marker.id = 0
        clear_marker.action = Marker.DELETEALL
        
        clear_array = MarkerArray()
        clear_array.markers.append(clear_marker)
        self.cluster_marker_pub.publish(clear_array)
        # 发布位姿数组
        if self.pose_array_pub.get_num_connections() > 0:
            pose_array = PoseArray()
            pose_array.header = header
            
            for centroid in centroids:
                
                pose = Pose()
                pose.position.x = centroid[0]
                pose.position.y = centroid[1]
                pose.position.z = centroid[2]
                pose.orientation.w = 1.0
                
                pose_array.poses.append(pose)
            
            self.pose_array_pub.publish(pose_array)
        
        # 发布边界框标记
        if self.cluster_marker_pub.get_num_connections() > 0:
            marker_array = MarkerArray()
            
            for i, box in enumerate(boxes):
                min_point = box['min']
                max_point = box['max']
                
                # 查找是否有分类信息
                classification = None
                if classified_clusters is not None:
                    for cls in classified_clusters:
                        if cls['cluster_idx'] == i:
                            classification = cls
                            break
                
                # 创建边界框标记
                marker = Marker()
                marker.header = header
                marker.ns = "adaptive_clustering"
                marker.id = i
                marker.type = Marker.LINE_LIST
                marker.action = Marker.ADD
                marker.scale.x = 0.02  # 线宽
                
                # 设置颜色 - 未分类为绿色，已分类根据类别设置不同颜色
                if classification is None:
                    marker.color.a = 1.0
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.5
                else:
                    # 使用类别名称生成颜色
                    class_name = classification['class_name']
                    
                    # 为不同类别设置不同颜色
                    if class_name in ['person', 'pedestrian']:
                        marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)  # 红色
                    elif class_name in ['car', 'vehicle', 'truck', 'bus']:
                        marker.color = ColorRGBA(0.0, 0.0, 1.0, 1.0)  # 蓝色
                    elif class_name in ['bicycle', 'bike']:
                        marker.color = ColorRGBA(1.0, 0.5, 0.0, 1.0)  # 橙色
                    else:
                        marker.color = ColorRGBA(0.8, 0.2, 0.8, 1.0)  # 紫色
                
                # 创建边界框角点
                p = [None] * 24
                p[0] = Point(max_point[0], max_point[1], max_point[2])
                p[1] = Point(min_point[0], max_point[1], max_point[2])
                p[2] = Point(max_point[0], max_point[1], max_point[2])
                p[3] = Point(max_point[0], min_point[1], max_point[2])
                p[4] = Point(max_point[0], max_point[1], max_point[2])
                p[5] = Point(max_point[0], max_point[1], min_point[2])
                p[6] = Point(min_point[0], min_point[1], min_point[2])
                p[7] = Point(max_point[0], min_point[1], min_point[2])
                p[8] = Point(min_point[0], min_point[1], min_point[2])
                p[9] = Point(min_point[0], max_point[1], min_point[2])
                p[10] = Point(min_point[0], min_point[1], min_point[2])
                p[11] = Point(min_point[0], min_point[1], max_point[2])
                p[12] = Point(min_point[0], max_point[1], max_point[2])
                p[13] = Point(min_point[0], max_point[1], min_point[2])
                p[14] = Point(min_point[0], max_point[1], max_point[2])
                p[15] = Point(min_point[0], min_point[1], max_point[2])
                p[16] = Point(max_point[0], min_point[1], max_point[2])
                p[17] = Point(max_point[0], min_point[1], min_point[2])
                p[18] = Point(max_point[0], min_point[1], max_point[2])
                p[19] = Point(min_point[0], min_point[1], max_point[2])
                p[20] = Point(max_point[0], max_point[1], min_point[2])
                p[21] = Point(min_point[0], max_point[1], min_point[2])
                p[22] = Point(max_point[0], max_point[1], min_point[2])
                p[23] = Point(max_point[0], min_point[1], min_point[2])
                
                # 添加所有点到标记
                for point in p:
                    marker.points.append(point)
                    
                # 如果已分类，添加文本标记
                if classification is not None:
                    text_marker = Marker()
                    text_marker.header = header
                    text_marker.ns = "adaptive_clustering_labels"
                    text_marker.id = i
                    text_marker.type = Marker.TEXT_VIEW_FACING
                    text_marker.action = Marker.ADD
                    
                    # 文本位置（在边界框上方）
                    text_marker.pose.position.x = (min_point[0] + max_point[0]) / 2
                    text_marker.pose.position.y = (min_point[1] + max_point[1]) / 2
                    text_marker.pose.position.z = max_point[2] + 0.5
                    text_marker.pose.orientation.w = 1.0
                    
                    # 文本内容
                    text = f"{classification['class_name']}: {classification['confidence']:.2f}"
                    text_marker.text = text
                    
                    # 文本样式
                    text_marker.scale.z = 0.4  # 文本大小
                    text_marker.color = marker.color  # 与边界框相同的颜色
                    marker_array.markers.append(text_marker)
                
                marker.lifetime = rospy.Duration(0.1)
                marker_array.markers.append(marker)
                
            if marker_array.markers:
                self.cluster_marker_pub.publish(marker_array)

if __name__ == "__main__":
   try:
       fusion = MultiModalFusion()
       rospy.spin()
   except rospy.ROSInterruptException:
       pass
