#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge

class StereoNode(Node):
    """Convert synchronized stereo frames into a depth image and virtual LaserScan."""

    def __init__(self):
        super().__init__('stereo_node')
        self.publisher_scan = self.create_publisher(LaserScan, '/scan', 10)
        self.publisher_depth = self.create_publisher(Image, '/stereo/depth_image', 10)
        self.bridge = CvBridge()

        # ROS 2 Parameters with default values (Inherited dynamically from omnimow.env via launch) [UPDATED FOR DUAL-CSI]
        self.declare_parameter('video_device_left', '/dev/video0')
        self.declare_parameter('video_device_right', '/dev/video1')
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('baseline', 0.06)          # IMX219 Binocular standard baseline is 60mm
        self.declare_parameter('focal_length', 350.0)      # Focal length in pixels
        self.declare_parameter('camera_height_z', 0.10)    # Mounting height above ground (meters)
        self.declare_parameter('camera_offset_x', 0.25)    # Forward distance from base_link rotation center (meters)
        self.declare_parameter('camera_pitch_y', 0.0)      # Pitch angle in radians (0.0 = horizontal, positive = downward)

        device_l = self.get_parameter('video_device_left').get_parameter_value().string_value
        device_r = self.get_parameter('video_device_right').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.baseline = self.get_parameter('baseline').get_parameter_value().double_value
        self.focal_length = self.get_parameter('focal_length').get_parameter_value().double_value
        self.camera_height_z = self.get_parameter('camera_height_z').get_parameter_value().double_value
        self.camera_offset_x = self.get_parameter('camera_offset_x').get_parameter_value().double_value
        self.camera_pitch_y = self.get_parameter('camera_pitch_y').get_parameter_value().double_value

        # Open both MIPI CSI video devices in V4L2 mode
        self.cap_left = cv2.VideoCapture(device_l, cv2.CAP_V4L2)
        self.cap_right = cv2.VideoCapture(device_r, cv2.CAP_V4L2)

        if not self.cap_left.isOpened() or not self.cap_right.isOpened():
            self.get_logger().error(f"Could not open dual MIPI CSI cameras: {device_l} & {device_r}")
            return

        self.get_logger().info(f"MIPI CSI Stereo Node started. Baseline={self.baseline}m, FocalLength={self.focal_length}px")
        self.timer = self.create_timer(0.05, self.process_frame) # 20 FPS (50ms)

    def process_frame(self):
        # IMPROVED SYNCHRONIZATION: Use grab() and then retrieve() to fetch frames lightning-fast in hardware
        if not (self.cap_left.grab() and self.cap_right.grab()):
            self.get_logger().warn("Could not synchronize/grab frames from MIPI CSI.")
            return

        ret_l, left_img = self.cap_left.retrieve()
        ret_r, right_img = self.cap_right.retrieve()

        if not (ret_l and ret_r):
            self.get_logger().warn("Could not decode image frames from MIPI CSI.")
            return

        h, w, _ = left_img.shape

        # Convert to grayscale (Grayscale)
        gray_left = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)

        # StereoSGBM estimates horizontal pixel displacement between the two cameras.
        stereo = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=64,
            blockSize=9,
            P1=8 * 3 * 9 * 9,
            P2=32 * 3 * 9 * 9,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=32
        )
        disparity = stereo.compute(gray_left, gray_right).astype(np.float32) / 16.0

        # Avoid division by zero and calculate depth map: Z = (focal_length * baseline) / disparity
        disparity[disparity <= 0] = 0.1
        depth_map = (self.focal_length * self.baseline) / disparity

        # Publish the depth image for visualization and downstream debugging.
        depth_msg = self.bridge.cv2_to_imgmsg(depth_map, encoding="32FC1")
        depth_msg.header.stamp = self.get_clock().now().to_msg()
        depth_msg.header.frame_id = self.frame_id
        self.publisher_depth.publish(depth_msg)

        # ==========================================
        #  3D VIRTUAL LIDAR ALGORITHM WITH PITCH COMPENSATION
        # ==========================================
        # Downsample grid to maintain 20 FPS (Kryo architecture handles this effortlessly)
        step = 4
        rows = np.arange(h // 4, h - 10, step) # Exclude sky and very close bumper
        cols = np.arange(0, w, step)           # No splitting of w now, since left_img is the full left image!
        v_grid, u_grid = np.meshgrid(rows, cols, indexing='ij')

        # Retrieve depth values
        z_c = depth_map[v_grid, u_grid]

        # Validity mask for depth (working area: 0.15m to 4.0m)
        valid_mask = (z_c > 0.15) & (z_c < 4.0)

        if not np.any(valid_mask):
            self.publish_empty_scan(w)
            return

        z_c = z_c[valid_mask]
        v_coords = v_grid[valid_mask]
        u_coords = u_grid[valid_mask]

        # Camera's optical center
        cx = w / 2.0
        cy = h / 2.0

        # Calculate 3D points in the camera's own coordinate system (Standard ROS Camera frame: Z=forward, X=right, Y=down)
        x_c = (u_coords - cx) * z_c / self.focal_length
        y_c = (v_coords - cy) * z_c / self.focal_length

        # Trigonometric rotation (pitch compensation) and translation to base_link (robot's center of rotation on the ground)
        theta = self.camera_pitch_y
        cos_t = np.cos(theta)
        sin_t = np.sin(theta)

        # 3D transformation to robot coordinates:
        X_r = z_c * cos_t - y_c * sin_t + self.camera_offset_x
        Y_r = -x_c
        Z_r = -y_c * cos_t - z_c * sin_t + self.camera_height_z

        # OBSTACLE FILTERING:
        # We filter grass and tiles by only considering points that rise above grass height (e.g., >= 5 cm),
        # but are lower than the robot's physical body (e.g., <= 45 cm) to ignore branches above the robot.
        obstacle_mask = (Z_r >= 0.05) & (Z_r <= 0.45) & (X_r > 0.1) & (X_r < 4.0)

        X_obs = X_r[obstacle_mask]
        Y_obs = Y_r[obstacle_mask]

        # Convert recorded obstacle points to 2D polar coordinates (Distance R and Angle alpha)
        R_obs = np.sqrt(X_obs**2 + Y_obs**2)
        alpha_obs = np.arctan2(Y_obs, X_obs)

        # Create standardized LaserScan message for Nav2
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.frame_id

        # Camera's horizontal FOV is approximately 83 degrees (1.4486 radians) for IMX219-83
        fov_rad = 1.4486
        num_readings = w
        scan.angle_min = -fov_rad / 2.0
        scan.angle_max = fov_rad / 2.0
        scan.angle_increment = fov_rad / num_readings
        scan.time_increment = 0.0
        scan.scan_time = 0.05
        scan.range_min = 0.15
        scan.range_max = 4.0

        # Initialize all measurements with infinite distance
        scan_ranges = np.full(num_readings, float('inf'))

        # Map each obstacle point to the LaserScan angle bin that contains it.
        bin_indices = ((alpha_obs - scan.angle_min) / scan.angle_increment).astype(int)

        # Ensure that indices stay within array bounds
        valid_bins = (bin_indices >= 0) & (bin_indices < num_readings)
        bin_indices = bin_indices[valid_bins]
        R_obs = R_obs[valid_bins]

        # Keep the closest obstacle for each angle so Nav2 receives conservative ranges.
        for idx, r in zip(bin_indices, R_obs):
            if r < scan_ranges[idx]:
                scan_ranges[idx] = r

        # Replace any remaining infinite values with NaN (standard in ROS for "no object found")
        scan.ranges = np.where(np.isinf(scan_ranges), float('nan'), scan_ranges).tolist()

        self.publisher_scan.publish(scan)

    def publish_empty_scan(self, num_readings):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.frame_id
        fov_rad = 1.4486
        scan.angle_min = -fov_rad / 2.0
        scan.angle_max = fov_rad / 2.0
        scan.angle_increment = fov_rad / num_readings
        scan.time_increment = 0.0
        scan.scan_time = 0.05
        scan.range_min = 0.15
        scan.range_max = 4.0
        scan.ranges = [float('nan')] * num_readings
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