#!/usr/bin/env python3
"""
OmniMow AI vision: runs the YOLO segmentation model on the Hexagon NPU (QNN context
binary via Qualcomm QAI AppBuilder) and fuses the obstacle mask with the stereo depth.

Publishes
  /vision/segmented_mask   mono8 mask of obstacle classes (255 = obstacle)
  /vision/obstacle_points  PointCloud2 in base_link for the Nav2 costmaps
  /vision/hazard           Bool, true while a hazard class (e.g. dog) is in view
  /vision/detections       JSON string with class, score and box for the app/debugging
"""
import json
import os
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Header, String

from yolo_seg import class_mask, decode, preprocess
from npu_model import NpuModel, init_qnn_runtime

CLASS_NAMES = ["dog", "grass", "pavement", "poo", "toy", "tree", "wall"]  # Same order as the training CLASS_MAP


class DepthFusionNode(Node):
    """Run image segmentation and publish obstacles for Nav2 obstacle marking."""

    def __init__(self):
        super().__init__('depth_fusion_node')
        self.bridge = CvBridge()

        self.declare_parameter('model_path', '/models/MowerAIn_seg_htp_context.bin')
        self.declare_parameter('confidence_threshold', 0.50)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('active_learning_threshold', 0.20)  # MLOps: save uncertain images
        self.declare_parameter('obstacle_classes', [0, 3, 4, 5, 6])  # dog, poo, toy, tree, wall
        self.declare_parameter('hazard_classes', [0])                # dog -> /vision/hazard
        self.declare_parameter('active_learning_classes', [0, 3, 4])  # dog, poo, toy
        self.declare_parameter('incoming_dir', '/incoming_raw')
        self.declare_parameter('reload_check_sec', 10.0)
        self.declare_parameter('max_obstacle_range', 4.0)
        self.declare_parameter('obstacle_point_height', 0.10)  # Mark AI obstacles above the costmap min height
        self.declare_parameter('point_step', 4)                 # Use every Nth mask pixel for the point cloud
        self.declare_parameter('grass_classes', [1])            # grass
        self.declare_parameter('grass_boundary_range', 2.5)     # Only nearby ground for the lawn edge

        p = lambda name: self.get_parameter(name).value
        self.model_path = p('model_path')
        self.conf_thresh = p('confidence_threshold')
        self.iou_thresh = p('iou_threshold')
        self.al_thresh = p('active_learning_threshold')
        self.obstacle_classes = set(p('obstacle_classes'))
        self.hazard_classes = set(p('hazard_classes'))
        self.al_classes = set(p('active_learning_classes'))
        self.incoming_dir = p('incoming_dir')
        self.max_range = p('max_obstacle_range')
        self.point_height = p('obstacle_point_height')
        self.point_step = int(p('point_step'))
        self.grass_classes = set(p('grass_classes'))
        self.boundary_range = p('grass_boundary_range')
        # Detected lawn edge -> /vision/boundary_points (Nav2 costmaps). Disable with OMNIMOW_AI_GRASS_BOUNDARY=false
        self.grass_boundary = os.getenv('OMNIMOW_AI_GRASS_BOUNDARY', 'true').lower() == 'true'

        # Camera intrinsics from the stereo calibration (/stereo/left/camera_info), mount pose from omnimow.env
        self.focal_length = None
        self.cx = self.cy = None
        self.info_width = None
        self.camera_height_z = float(os.getenv('OMNIMOW_CAMERA_HEIGHT_Z', '0.10'))
        self.camera_offset_x = float(os.getenv('OMNIMOW_CAMERA_OFFSET_X', '0.25'))
        self.camera_pitch_y = float(os.getenv('OMNIMOW_CAMERA_PITCH_Y', '0.0'))

        self.publisher_mask = self.create_publisher(Image, '/vision/segmented_mask', 10)
        self.publisher_points = self.create_publisher(PointCloud2, '/vision/obstacle_points', 10)
        self.publisher_boundary = self.create_publisher(PointCloud2, '/vision/boundary_points', 10)
        self.publisher_hazard = self.create_publisher(Bool, '/vision/hazard', 10)
        self.publisher_detections = self.create_publisher(String, '/vision/detections', 10)

        self.model = None
        self.depths = deque(maxlen=5)  # Newest depth images, to find the one from the image's stereo frame
        self.last_al_save = 0.0
        self.frames = 0
        self.infer_time = 0.0
        self.total_time = 0.0
        self.rx_images = 0
        self.rx_depth = 0
        self.matched = 0

        self.init_runtime()
        self.load_model()

        # Depth (16UC1 mm) and the rectified left image (JPEG) come from the same stereo frame (identical stamps)
        self.create_subscription(CameraInfo, '/stereo/left/camera_info', self.camera_info_callback, 10)
        self.create_subscription(Image, '/stereo/depth_image', self.depth_callback, qos_profile_sensor_data)
        self.create_subscription(CompressedImage, '/stereo/left/image_rect_color/compressed',
                                 self.image_callback, qos_profile_sensor_data)
        self.create_timer(p('reload_check_sec'), self.check_model_update)
        self.create_timer(10.0, self.log_stats)

    def init_runtime(self):
        if self.model_path.endswith(".onnx"):
            return
        try:
            init_qnn_runtime()
        except Exception as e:
            self.get_logger().error(f"Could not initialise QNN runtime (qai_appbuilder): {e}")

    def load_model(self):
        """Load (or reload) the model. On failure the previous model keeps running."""
        if not os.path.exists(self.model_path):
            self.get_logger().warn(f"Model not found: {self.model_path} - waiting for a deployed model...")
            return
        try:
            new_model = NpuModel(self.model_path)
        except Exception as e:
            self.get_logger().error(f"Could not load model {self.model_path}: {e}")
            return
        old_model, self.model = self.model, new_model
        if old_model is not None:
            old_model.release()
        self.get_logger().info(
            f"Model loaded on {new_model.backend}: {self.model_path} "
            f"(input {new_model.input_shape}, {'NCHW' if new_model.nchw else 'NHWC'})")

    def check_model_update(self):
        if not os.path.exists(self.model_path):
            return
        if self.model is None or os.path.getmtime(self.model_path) != self.model.mtime:
            self.get_logger().info("New model file detected - reloading...")
            self.load_model()

    def log_stats(self):
        if self.frames:
            self.get_logger().info(
                f"{self.frames / 10.0:.1f} FPS, {1000 * self.infer_time / self.frames:.0f} ms inference, "
                f"{1000 * self.total_time / self.frames:.0f} ms/frame in total; received "
                f"{self.rx_images / 10.0:.1f} images/s, {self.rx_depth / 10.0:.1f} depth/s, "
                f"{self.matched / 10.0:.1f}/s with matching depth")
        elif self.rx_images or self.rx_depth:
            self.get_logger().info(f"No model yet; received {self.rx_images / 10.0:.1f} images/s, "
                                   f"{self.rx_depth / 10.0:.1f} depth/s")
        self.frames = 0
        self.infer_time = 0.0
        self.total_time = 0.0
        self.rx_images = self.rx_depth = self.matched = 0

    def camera_info_callback(self, msg):
        self.focal_length, self.cx, self.cy = msg.p[0], msg.p[2], msg.p[6]
        self.info_width = msg.width

    def depth_callback(self, msg):
        self.rx_depth += 1
        self.depths.append(msg)

    def image_callback(self, msg):
        self.rx_images += 1
        if self.model is None:
            return
        cv_image = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if cv_image is None:
            self.get_logger().warn("Could not decode the JPEG camera image", throttle_duration_sec=5.0)
            return

        start = time.time()
        tensor, meta = preprocess(cv_image, self.model.size, self.model.nchw)
        try:
            outputs = self.model.infer(tensor)
            if getattr(self, 'shapes_logged_for', None) is not self.model:
                # Once per loaded model: what the export produced, to check against CLASS_NAMES
                self.shapes_logged_for = self.model
                self.get_logger().info(
                    f"Model outputs: {[tuple(np.asarray(o).shape) for o in outputs]} "
                    f"({len(CLASS_NAMES)} classes expected: {', '.join(CLASS_NAMES)})")
            detections, protos = decode(outputs, meta, min(self.al_thresh, self.conf_thresh), self.iou_thresh)
        except Exception as e:
            self.get_logger().error(f"Inference failed: {e}", throttle_duration_sec=5.0)
            return
        self.infer_time += time.time() - start
        self.frames += 1

        confident = [d for d in detections if d["score"] >= self.conf_thresh]
        mask = class_mask(confident, protos, meta, self.obstacle_classes)

        mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
        mask_msg.header = msg.header
        self.publisher_mask.publish(mask_msg)

        grass_mask = (class_mask(detections, protos, meta, self.grass_classes)
                      if self.grass_boundary else None)
        self.publish_obstacle_points(mask, grass_mask, msg.header)
        self.publisher_hazard.publish(Bool(data=any(d["cls"] in self.hazard_classes for d in confident)))
        self.publisher_detections.publish(String(data=json.dumps([
            {"class": CLASS_NAMES[d["cls"]] if d["cls"] < len(CLASS_NAMES) else d["cls"],
             "score": round(d["score"], 3), "box": [round(float(v), 1) for v in d["box"]]}
            for d in confident
        ])))
        self.save_uncertain_image(cv_image, detections)
        self.total_time += time.time() - start

    def matching_depth(self, image_header):
        """Depth image (metres) from the same stereo frame as the processed image, or None.
        It has the size of the camera_info intrinsics, which may be smaller than the AI image."""
        stamp_ns = lambda s: s.sec * 1_000_000_000 + s.nanosec
        target = stamp_ns(image_header.stamp)
        diff = lambda m: abs(stamp_ns(m.header.stamp) - target)
        depth_msg = min(self.depths, key=diff, default=None)
        if depth_msg is None or not self.focal_length or diff(depth_msg) > 150_000_000:
            return None
        self.matched += 1
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        if depth.dtype == np.uint16:  # 16UC1 millimetres (stereo_node), 0 = no depth
            depth = depth.astype(np.float32) * 0.001
        return depth

    def pixels_to_base_link(self, mask, depth, max_range):
        """Project mask pixels with valid stereo depth into base_link (same math as stereo_node).
        mask and depth have the same size; the depth may be a subsampled camera image (pixel
        (u, v) = (u * sub, v * sub) of the camera_info image). Returns X, Y, Z arrays (metres)."""
        sub = self.info_width / depth.shape[1] if self.info_width else 1.0
        step = max(1, int(round(self.point_step / sub)))
        sub_depth = depth[::step, ::step]
        valid = (mask[::step, ::step] > 0) & (sub_depth > 0.15) & (sub_depth < max_range)
        v, u = np.nonzero(valid)
        z_c = sub_depth[v, u]
        x_c = (u * step * sub - self.cx) * z_c / self.focal_length
        y_c = (v * step * sub - self.cy) * z_c / self.focal_length
        cos_t, sin_t = np.cos(self.camera_pitch_y), np.sin(self.camera_pitch_y)
        X_r = z_c * cos_t - y_c * sin_t + self.camera_offset_x
        Y_r = -x_c
        Z_r = -y_c * cos_t - z_c * sin_t + self.camera_height_z
        return X_r, Y_r, Z_r

    def publish_points(self, publisher, X, Y, header):
        # Flat things (poo, toys, the edge of the lawn) lie below the costmap's min obstacle height,
        # so every AI point is marked at a fixed height at its ground position.
        points = np.column_stack([X, Y, np.full_like(X, self.point_height)]).astype(np.float32)
        # An empty cloud is published too, so the costmap can clear obstacles that are gone
        publisher.publish(point_cloud2.create_cloud_xyz32(header, points))

    def publish_obstacle_points(self, obstacle_mask, grass_mask, image_header):
        header = Header(stamp=image_header.stamp, frame_id='base_link')
        empty = np.zeros(0, dtype=np.float32)
        depth = self.matching_depth(image_header)
        if depth is None:
            self.publish_points(self.publisher_points, empty, empty, header)
            if self.grass_boundary:
                self.publish_points(self.publisher_boundary, empty, empty, header)
            return

        # The AI image is larger than the depth (OMNIMOW_AI_IMAGE_WIDTH, and the depth is subsampled):
        # bring the masks to the depth size
        if obstacle_mask.shape[:2] != depth.shape[:2]:
            size = (depth.shape[1], depth.shape[0])
            obstacle_mask = cv2.resize(obstacle_mask, size, interpolation=cv2.INTER_NEAREST)
            if grass_mask is not None:
                grass_mask = cv2.resize(grass_mask, size, interpolation=cv2.INTER_NEAREST)

        X, Y, _ = self.pixels_to_base_link(obstacle_mask, depth, self.max_range)
        self.publish_points(self.publisher_points, X, Y, header)

        if self.grass_boundary:
            # Lawn edge: ground (not raised) pixels that are neither grass nor an obstacle,
            # e.g. pavement, soil in flower beds, gravel. Keeps the mower on the grass even if
            # its position estimate drifts. Only nearby ground, where stereo depth is reliable.
            not_grass = ((grass_mask == 0) & (obstacle_mask == 0)).astype(np.uint8)
            X, Y, Z = self.pixels_to_base_link(not_grass, depth, self.boundary_range)
            ground = np.abs(Z) < 0.05
            self.publish_points(self.publisher_boundary, X[ground], Y[ground], header)

    def save_uncertain_image(self, cv_image, detections):
        """Active learning: save frames with uncertain detections for the next training round."""
        uncertain = any(self.al_thresh <= d["score"] < self.conf_thresh and d["cls"] in self.al_classes
                        for d in detections)
        now = time.time()
        if not uncertain or now - self.last_al_save < 10.0 or not os.path.isdir(self.incoming_dir):
            return
        self.last_al_save = now
        filepath = os.path.join(self.incoming_dir, f"img_{int(now * 1000)}.jpg")
        cv2.imwrite(filepath, cv_image)
        self.get_logger().info(f"Saved uncertain object for active learning: {filepath}")

    def destroy_node(self):
        if self.model is not None:
            self.model.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DepthFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
