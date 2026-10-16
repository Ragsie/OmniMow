#!/usr/bin/env python3
"""
OmniMow stereo node: rectified stereo images for visual SLAM, a depth image for the AI fusion
and a virtual LaserScan for Nav2.

Uses the calibration from calibrate_stereo.py (ROS camera_info YAML files). Publishes
  /stereo/left/image_rect, /stereo/right/image_rect   mono8, rectified (RTAB-Map)
  /stereo/left/camera_info, /stereo/right/camera_info  rectified camera_info
  /stereo/left/image_rect_color/compressed              JPEG (AI segmentation input), OMNIMOW_AI_IMAGE_WIDTH wide
  /stereo/left/image_rect_color                         bgr8 like the JPEG, only while something subscribes (debug)
  /stereo/depth_image                                   16UC1 millimetres (0 = no depth), aligned with the left image,
                                                        half its size (camera_info intrinsics scaled by 0.5)
  /scan                                                 LaserScan in base_link
"""
import os
import time
from concurrent.futures import ThreadPoolExecutor

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, LaserScan

from camera import StereoCapture

OPTICAL_FRAME = 'camera_optical_frame'

SGBM_MODES = {
    'sgbm': cv2.STEREO_SGBM_MODE_SGBM,
    '3way': cv2.STEREO_SGBM_MODE_SGBM_3WAY,  # Multithreaded, several times faster on the board's cores
    'hh4': cv2.STEREO_SGBM_MODE_HH4,
}


def load_camera_info(path):
    with open(path) as f:
        c = yaml.safe_load(f)
    size = (c['image_width'], c['image_height'])
    K = np.array(c['camera_matrix']['data'], dtype=np.float64).reshape(3, 3)
    D = np.array(c['distortion_coefficients']['data'], dtype=np.float64)
    R = np.array(c['rectification_matrix']['data'], dtype=np.float64).reshape(3, 3)
    P = np.array(c['projection_matrix']['data'], dtype=np.float64).reshape(3, 4)
    return size, K, D, R, P


def scale_intrinsics(M, s):
    """3x3 camera matrix for an image scaled by s (pixel centres stay pixel centres)."""
    M = M.copy()
    M[:2] *= s
    M[0, 2] += 0.5 * s - 0.5
    M[1, 2] += 0.5 * s - 0.5
    return M


def ai_image_size(size):
    """
    Size of the colour image for the AI (16:9 like the camera), or None to use the stereo size.
    The model's input is 800x800: an 800 px wide image fills it without upscaling (letterboxed
    to 800x450 + padding), which is sharper than upscaling the 640x360 stereo image.
    """
    width = int(os.getenv('OMNIMOW_AI_IMAGE_WIDTH', '800') or 0)
    if width <= size[0]:
        return None
    return width, int(round(width * size[1] / size[0] / 2)) * 2


def rectified_camera_info(size, P):
    """camera_info of the rectified image: no distortion, identity rectification."""
    msg = CameraInfo()
    msg.width, msg.height = size
    msg.distortion_model = 'plumb_bob'
    msg.d = [0.0] * 5
    msg.k = P[:3, :3].ravel().tolist()
    msg.r = np.eye(3).ravel().tolist()
    msg.p = P.ravel().tolist()
    return msg


class StereoNode(Node):
    """Convert synchronized stereo frames into rectified images, a depth image and a virtual LaserScan."""

    def __init__(self):
        super().__init__('stereo_node')
        self.bridge = CvBridge()

        # ROS 2 parameters (inherited from omnimow.env via the launch file)
        # Empty = camera from omnimow.env (/dev/videoN or libcamera ID, see camera.py)
        self.declare_parameter('video_device_left', '')
        self.declare_parameter('video_device_right', '')
        self.declare_parameter('calibration_dir', '/calibration')
        self.declare_parameter('camera_height_z', 0.10)    # Mounting height above ground (metres)
        self.declare_parameter('camera_offset_x', 0.25)    # Forward distance from base_link (metres)
        self.declare_parameter('camera_pitch_y', 0.0)      # Pitch in radians (positive = downward)
        self.declare_parameter('jpeg_quality', 85)         # Colour image for the AI (JPEG)

        p = lambda name: self.get_parameter(name).value
        self.calibration_dir = p('calibration_dir')
        self.camera_height_z = p('camera_height_z')
        self.camera_offset_x = p('camera_offset_x')
        self.camera_pitch_y = p('camera_pitch_y')
        self.jpeg_params = [cv2.IMWRITE_JPEG_QUALITY, int(p('jpeg_quality'))]

        # Calibration -> rectification maps and rectified intrinsics
        size_l, K1, D1, R1, P1 = load_camera_info(os.path.join(self.calibration_dir, 'left.yaml'))
        size_r, K2, D2, R2, P2 = load_camera_info(os.path.join(self.calibration_dir, 'right.yaml'))
        self.size = size_l
        self.map_l = cv2.initUndistortRectifyMap(K1, D1, R1, P1[:3, :3], size_l, cv2.CV_16SC2)
        self.map_r = cv2.initUndistortRectifyMap(K2, D2, R2, P2[:3, :3], size_r, cv2.CV_16SC2)
        self.focal_length = P1[0, 0]
        self.cx, self.cy = P1[0, 2], P1[1, 2]
        self.baseline = -P2[0, 3] / P2[0, 0]
        self.info_l = rectified_camera_info(size_l, P1)
        self.info_r = rectified_camera_info(size_r, P2)

        # Colour image for the AI: the left camera delivers it larger, it is rectified at that size
        # and scaled down to the calibrated size for the stereo matching
        self.ai_size = ai_image_size(size_l)
        self.map_ai = None
        if self.ai_size:
            s = self.ai_size[0] / size_l[0]
            self.map_ai = cv2.initUndistortRectifyMap(
                scale_intrinsics(K1, s), D1, R1, scale_intrinsics(P1[:3, :3], s), self.ai_size, cv2.CV_16SC2)

        self.publisher_scan = self.create_publisher(LaserScan, '/scan', 10)
        self.publisher_depth = self.create_publisher(Image, '/stereo/depth_image', 10)
        # Large raw images (> ~512 KB) mostly get lost between the containers, so the colour image
        # goes out as JPEG and the depth in 16 bit (millimetres); the raw colour image only for debugging
        self.publisher_left_jpeg = self.create_publisher(
            CompressedImage, '/stereo/left/image_rect_color/compressed', 10)
        self.publisher_left_color = self.create_publisher(Image, '/stereo/left/image_rect_color', 10)
        self.publisher_left = self.create_publisher(Image, '/stereo/left/image_rect', 10)
        self.publisher_right = self.create_publisher(Image, '/stereo/right/image_rect', 10)
        self.publisher_info_l = self.create_publisher(CameraInfo, '/stereo/left/camera_info', 10)
        self.publisher_info_r = self.create_publisher(CameraInfo, '/stereo/right/camera_info', 10)

        # Open both MIPI CSI cameras (GStreamer/NV12, see camera.py) at the calibrated resolution
        try:
            self.cameras = StereoCapture(p('video_device_left') or None, p('video_device_right') or None,
                                         self.size[0], self.size[1], left_size=self.ai_size)
        except RuntimeError as e:
            self.get_logger().error(f"Could not open the dual MIPI CSI cameras: {e}")
            return

        # StereoSGBM estimates horizontal pixel displacement between the rectified images (grayscale).
        # The largest disparity sets the closest depth: focal * baseline / numDisparities. More
        # disparities see closer, but cost time and leave that many columns at the left edge without depth.
        block = 9
        num_disparities = max(16, int(os.getenv('OMNIMOW_STEREO_NUM_DISPARITIES', '96')) // 16 * 16)
        mode = os.getenv('OMNIMOW_STEREO_MODE', '3way').lower()
        self.stereo = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=num_disparities,
            blockSize=block,
            P1=8 * block * block,
            P2=32 * block * block,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=32,
            mode=SGBM_MODES.get(mode, cv2.STEREO_SGBM_MODE_SGBM_3WAY)
        )
        # OpenCV's thread pool: all 8 cores (incl. the slow ones) mostly add spinning, not speed
        cv2.setNumThreads(int(os.getenv('OMNIMOW_STEREO_THREADS', '4') or 4))
        # The disparity runs in a worker thread (OpenCV releases the GIL) while the images are published
        self.worker = ThreadPoolExecutor(max_workers=1)
        min_depth = self.focal_length * self.baseline / num_disparities
        # Every frame costs stereo matching here and work in depth_fusion and SLAM; the mower moves
        # slowly, so the default 10 FPS of the cameras' 15 is plenty
        fps = float(os.getenv('OMNIMOW_STEREO_FPS', '10') or 10)

        self.get_logger().info(
            f"Stereo node started: {self.size[0]}x{self.size[1]}, baseline={self.baseline * 1000:.1f} mm, "
            f"focal={self.focal_length:.1f} px (from {self.calibration_dir}), AI image "
            f"{'%dx%d' % (self.ai_size or self.size)}, SGBM {mode} with {num_disparities} disparities "
            f"(depth from {min_depth:.2f} m), max {fps:g} FPS, {cv2.getNumThreads()} OpenCV threads")
        self.timer = self.create_timer(1.0 / fps, self.process_frame)
        self.stat_frames, self.stat_time = 0, 0.0
        self.stat_stages = {}
        self.create_timer(10.0, self.log_stats)
        self.create_timer(5.0, self.check_calibration)

    def check_calibration(self):
        """Recalibration requested (calibration deleted): exit so the container restarts into calibration."""
        if not os.path.exists(os.path.join(self.calibration_dir, 'left.yaml')):
            self.get_logger().warn("Calibration removed - restarting into stereo calibration")
            self.cameras.release()
            os._exit(3)

    def log_stats(self):
        """Every 10 s: stereo output rate and processing time, and what the cameras deliver."""
        fps_l, fps_r = self.cameras.fps()
        n = self.stat_frames or 1
        ms = 1000 * self.stat_time / n
        stages = ', '.join(f"{k} {1000 * v / n:.0f}" for k, v in self.stat_stages.items())
        self.get_logger().info(
            f"Stereo {self.stat_frames / 10.0:.1f} FPS ({ms:.0f} ms/frame: {stages}), "
            f"cameras left {fps_l:.1f} FPS, right {fps_r:.1f} FPS")
        self.stat_frames, self.stat_time = 0, 0.0
        self.stat_stages = {}

    def stage(self, name, since):
        """Add the time since `since` to stage `name` of the statistics; returns now."""
        now = time.monotonic()
        self.stat_stages[name] = self.stat_stages.get(name, 0.0) + now - since
        return now

    def process_frame(self):
        # Newest left/right pair taken close together (the cameras free-run in threads, see
        # StereoCapture.auto_skew)
        left_raw, right_raw = self.cameras.read(timeout=0.5)
        if left_raw is None:
            fps_l, fps_r = self.cameras.fps()
            self.get_logger().warn(
                f"No synchronized frames from the MIPI CSI cameras (left {fps_l:.1f} FPS, "
                f"right {fps_r:.1f} FPS, pairing within {self.cameras.auto_skew() * 1000:.0f} ms)",
                throttle_duration_sec=5.0)
            return
        expected_left = tuple(self.ai_size or self.size)
        if ((left_raw.shape[1], left_raw.shape[0]) != expected_left
                or (right_raw.shape[1], right_raw.shape[0]) != tuple(self.size)):
            self.get_logger().error(
                f"Cameras deliver {left_raw.shape[1]}x{left_raw.shape[0]} (left) and "
                f"{right_raw.shape[1]}x{right_raw.shape[0]} (right) but expected "
                f"{expected_left[0]}x{expected_left[1]} and {self.size[0]}x{self.size[1]} - set "
                f"OMNIMOW_CAMERA_WIDTH/HEIGHT and OMNIMOW_AI_IMAGE_WIDTH or recalibrate",
                throttle_duration_sec=10.0)
            return

        start = time.monotonic()
        if self.map_ai is not None:
            left_ai = cv2.remap(left_raw, *self.map_ai, cv2.INTER_LINEAR)
            left = cv2.resize(left_ai, tuple(self.size), interpolation=cv2.INTER_AREA)
        else:
            left = left_ai = cv2.remap(left_raw, *self.map_l, cv2.INTER_LINEAR)
        right = cv2.remap(right_raw, *self.map_r, cv2.INTER_LINEAR)
        gray_left = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
        h, w = gray_left.shape
        t = self.stage('rectify', start)
        disparity_job = self.worker.submit(self.stereo.compute, gray_left, gray_right)

        stamp = self.get_clock().now().to_msg()
        self.publish_jpeg(left_ai, stamp)
        if self.publisher_left_color.get_subscription_count() > 0:
            self.publish_image(self.publisher_left_color, left_ai, 'bgr8', stamp)
        self.publish_image(self.publisher_left, gray_left, 'mono8', stamp)
        self.publish_image(self.publisher_right, gray_right, 'mono8', stamp)
        for pub, info in ((self.publisher_info_l, self.info_l), (self.publisher_info_r, self.info_r)):
            info.header.stamp = stamp
            info.header.frame_id = OPTICAL_FRAME
            pub.publish(info)
        t = self.stage('publish', t)

        disparity = disparity_job.result().astype(np.float32) / 16.0
        t = self.stage('disparity wait', t)

        # Depth map Z = (focal_length * baseline) / disparity; invalid disparity -> 0 (no depth)
        depth_map = np.zeros_like(disparity)
        valid = disparity > 0.5
        depth_map[valid] = (self.focal_length * self.baseline) / disparity[valid]
        depth_mm = np.clip(depth_map * 1000.0, 0, 65535).astype(np.uint16)
        # Half size: 640x480 in 16 bit is 614 KB, and messages over ~512 KB mostly get lost between
        # the containers. Every second pixel of every second row (pixel (2u, 2v) of the full image).
        self.publish_image(self.publisher_depth, np.ascontiguousarray(depth_mm[::2, ::2]), '16UC1', stamp)

        self.publish_scan(depth_map, w, h, stamp)
        self.stage('depth+scan', t)
        self.stat_frames += 1
        self.stat_time += time.monotonic() - start

    def destroy_node(self):
        if hasattr(self, 'worker'):
            self.worker.shutdown(wait=False)
        super().destroy_node()

    def publish_image(self, publisher, image, encoding, stamp):
        msg = self.bridge.cv2_to_imgmsg(image, encoding=encoding)
        msg.header.stamp = stamp
        msg.header.frame_id = OPTICAL_FRAME
        publisher.publish(msg)

    def publish_jpeg(self, image, stamp):
        ok, data = cv2.imencode('.jpg', image, self.jpeg_params)
        if not ok:
            return
        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.header.frame_id = OPTICAL_FRAME
        msg.format = 'bgr8; jpeg compressed bgr8'
        msg.data.frombytes(data.tobytes())
        self.publisher_left_jpeg.publish(msg)

    # ==========================================
    #  3D VIRTUAL LIDAR ALGORITHM WITH PITCH COMPENSATION
    # ==========================================
    def publish_scan(self, depth_map, w, h, stamp):
        step = 4
        rows = np.arange(h // 4, h - 10, step)  # Exclude sky and very close bumper
        cols = np.arange(0, w, step)
        v_grid, u_grid = np.meshgrid(rows, cols, indexing='ij')
        z_c = depth_map[v_grid, u_grid]

        # Validity mask for depth (working area: 0.15m to 4.0m)
        valid_mask = (z_c > 0.15) & (z_c < 4.0)
        z_c = z_c[valid_mask]
        v_coords = v_grid[valid_mask]
        u_coords = u_grid[valid_mask]

        # 3D points in the optical frame (Z forward, X right, Y down), using the calibrated centre
        x_c = (u_coords - self.cx) * z_c / self.focal_length
        y_c = (v_coords - self.cy) * z_c / self.focal_length

        # Pitch compensation and translation to base_link (robot's centre of rotation on the ground)
        cos_t, sin_t = np.cos(self.camera_pitch_y), np.sin(self.camera_pitch_y)
        X_r = z_c * cos_t - y_c * sin_t + self.camera_offset_x
        Y_r = -x_c
        Z_r = -y_c * cos_t - z_c * sin_t + self.camera_height_z

        # Only points above grass height (>= 5 cm) and below the robot body (<= 45 cm)
        obstacle_mask = (Z_r >= 0.05) & (Z_r <= 0.45) & (X_r > 0.1) & (X_r < 4.0)
        X_obs = X_r[obstacle_mask]
        Y_obs = Y_r[obstacle_mask]
        R_obs = np.sqrt(X_obs ** 2 + Y_obs ** 2)
        alpha_obs = np.arctan2(Y_obs, X_obs)

        # Horizontal field of view of the rectified image
        fov_rad = 2.0 * np.arctan(w / (2.0 * self.focal_length))
        scan = LaserScan()
        scan.header.stamp = stamp
        # The points are already transformed to base_link above (offset + pitch)
        scan.header.frame_id = 'base_link'
        scan.angle_min = -fov_rad / 2.0
        scan.angle_max = fov_rad / 2.0
        scan.angle_increment = fov_rad / w
        scan.time_increment = 0.0
        scan.scan_time = self.timer.timer_period_ns * 1e-9
        scan.range_min = 0.15
        scan.range_max = 4.0

        # Keep the closest obstacle for each angle so Nav2 receives conservative ranges
        scan_ranges = np.full(w, np.inf)
        bins = ((alpha_obs - scan.angle_min) / scan.angle_increment).astype(int)
        ok = (bins >= 0) & (bins < w)
        np.minimum.at(scan_ranges, bins[ok], R_obs[ok])
        scan.ranges = np.where(np.isinf(scan_ranges), np.nan, scan_ranges).tolist()
        self.publisher_scan.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = StereoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
