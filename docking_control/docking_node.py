#!/usr/bin/env python3
"""
OmniMow docking: Nav2 to a staging pose in front of the charging station, then slowly straight
ahead until the BMS reports charging current. Undocking backs out and turns clear of the dock.

The staging pose is found by the mower itself: whenever it leaves the dock (e.g. when the lawn is
mapped or a mission starts) and whenever it docks, the docked pose on the SLAM map is saved in
/opt/omnimow/dock.json. The staging pose is OMNIMOW_DOCK_STAGING_DISTANCE in front of it. Until a
pose is saved, the map origin is used (RTAB-Map's map starts where the mower stood in the dock).
OMNIMOW_DOCK_STAGING_X/_Y/_YAW, if set, override all of this.

Docking and undocking can be stopped at any time: /omnimow/dock_cancel (mowing_control calls it on
stop), an emergency stop or a stuck mower (/mower/state). While a hazard (dog) is in view, the slow
approach and the undocking pause.

Services (std_srvs/Trigger): /omnimow/dock, /omnimow/undock, /omnimow/dock_cancel
"""
import json
import math
import os
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformException, TransformListener

DOCK_FILE = os.getenv('OMNIMOW_DOCK_FILE', '/opt/omnimow/dock.json')
MOWER_STUCK, MOWER_ESTOP = 4, 5  # /mower/state from the drive ESP32
CHARGING_AMPS = 0.08             # Daly BMS: positive current while charging


class DockingCancelled(Exception):
    """Docking or undocking was stopped (cancel request, emergency stop)."""


def env_optional_float(name):
    value = os.getenv(name, '').strip()
    return float(value) if value else None


class OmniMowDockingController(Node):
    """Navigate to a staging pose, then use slow motion and charging current to dock."""

    def __init__(self):
        super().__init__('omnimow_docking_controller')
        self.get_logger().info('OmniMow docking and charging control started.')

        # From omnimow.env / robot.env
        self.staging_override = [env_optional_float(f'OMNIMOW_DOCK_STAGING_{k}') for k in ('X', 'Y', 'YAW')]
        self.staging_distance = float(os.getenv('OMNIMOW_DOCK_STAGING_DISTANCE', '1.0'))
        self.dock_speed = float(os.getenv('OMNIMOW_DOCK_SPEED', '0.05'))  # Very slow and safe driving
        self.max_distance = float(os.getenv('OMNIMOW_DOCK_MAX_DISTANCE', '2.0'))  # Safety limit (meters)

        # The service callbacks block while docking, so everything runs in a reentrant group on a
        # MultiThreadedExecutor - otherwise Nav2 results, battery messages and a cancel request
        # could never arrive.
        self.cb_group = ReentrantCallbackGroup()
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=self.cb_group)
        self.create_subscription(BatteryState, '/battery_state', self.battery_callback, 10,
                                 callback_group=self.cb_group)
        self.create_subscription(Int32, '/mower/state', self.mower_state_callback, 10,
                                 callback_group=self.cb_group)
        self.create_subscription(Bool, '/vision/hazard', self.hazard_callback, 10, callback_group=self.cb_group)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.is_charging = False
        self.current_voltage = 0.0
        self.current_charge_amps = 0.0
        self.mower_state = 0
        self.hazard_seen = 0.0  # Time of the last /vision/hazard = true
        self.cancel_requested = False
        self.cancel_reason = ''
        self.goal_handle = None

        # Called by mowing_control
        self.dock_srv = self.create_service(Trigger, '/omnimow/dock', self.handle_dock_request, callback_group=self.cb_group)
        self.undock_srv = self.create_service(Trigger, '/omnimow/undock', self.handle_undock_request, callback_group=self.cb_group)
        self.cancel_srv = self.create_service(Trigger, '/omnimow/dock_cancel', self.handle_cancel_request,
                                              callback_group=self.cb_group)

    # ------------------------------------------------------------------ callbacks
    def battery_callback(self, msg: BatteryState):
        """Update charging state from the battery data published by the ESP32."""
        self.current_voltage = msg.voltage
        self.current_charge_amps = msg.current  # Positive value indicates charging current
        self.is_charging = self.current_charge_amps > CHARGING_AMPS

    def mower_state_callback(self, msg):
        self.mower_state = msg.data
        if msg.data in (MOWER_STUCK, MOWER_ESTOP) and not self.cancel_requested:
            self.cancel('emergency stop' if msg.data == MOWER_ESTOP else 'mower stuck')

    def hazard_callback(self, msg):
        if msg.data:
            self.hazard_seen = time.time()

    @property
    def hazard(self):
        # Only while depth_fusion keeps reporting it: a stopped depth_fusion must not block docking
        return time.time() - self.hazard_seen < 1.0

    def handle_cancel_request(self, request, response):
        self.cancel('stop requested')
        response.success, response.message = True, "Docking stopped"
        return response

    def cancel(self, reason):
        """Stop docking/undocking now: the waiting loops notice the flag within 0.1 s."""
        self.cancel_requested = True
        self.cancel_reason = reason
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.stop_wheels()
        self.get_logger().warn(f"Docking cancelled: {reason}")

    # ------------------------------------------------------------------ helpers
    def stop_wheels(self):
        self.cmd_vel_pub.publish(Twist())

    def check_cancel(self):
        if self.cancel_requested:
            self.stop_wheels()
            raise DockingCancelled(self.cancel_reason)

    def wait_for_future(self, future, timeout_sec=None):
        """Wait for a future that the executor completes in another thread. Stops on cancel."""
        start = time.time()
        while not future.done():
            self.check_cancel()
            if timeout_sec is not None and time.time() - start > timeout_sec:
                return False
            time.sleep(0.05)
        return True

    def drive_for(self, linear, angular, seconds):
        """Drive at a fixed speed for a time; pauses (and stops the wheels) while a hazard is in view."""
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = linear, angular
        remaining = seconds
        while remaining > 0:
            self.check_cancel()
            if self.hazard:
                self.stop_wheels()
            else:
                self.cmd_vel_pub.publish(cmd)
                remaining -= 0.1
            time.sleep(0.1)
        self.stop_wheels()

    def map_pose(self):
        """(x, y, yaw) of base_link on the SLAM map, or None without localization."""
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', Time())
        except TransformException:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.x, t.y, yaw

    def save_dock_pose(self):
        """Remember where the mower stands while it is in the dock (map frame)."""
        pose = self.map_pose()
        if pose is None:
            self.get_logger().warn("No map position - the dock pose is not saved")
            return
        try:
            os.makedirs(os.path.dirname(DOCK_FILE), exist_ok=True)
            with open(DOCK_FILE + '.tmp', 'w') as f:
                json.dump({"frame": "map", "x": round(pose[0], 3), "y": round(pose[1], 3),
                           "yaw": round(pose[2], 4), "saved_at": int(time.time())}, f, indent=2)
            os.replace(DOCK_FILE + '.tmp', DOCK_FILE)
            self.get_logger().info(f"Dock pose saved: x={pose[0]:.2f} y={pose[1]:.2f} yaw={pose[2]:.2f}")
        except OSError as e:
            self.get_logger().error(f"Could not save the dock pose: {e}")

    def staging_pose(self):
        """Staging pose (x, y, yaw) in front of the dock, facing it, and where it came from."""
        if all(v is not None for v in self.staging_override):
            return tuple(self.staging_override), "OMNIMOW_DOCK_STAGING_X/_Y/_YAW"
        try:
            with open(DOCK_FILE) as f:
                dock = json.load(f)
            x, y, yaw, source = float(dock['x']), float(dock['y']), float(dock['yaw']), DOCK_FILE
        except (OSError, ValueError, KeyError):
            # RTAB-Map's map origin is where the first session started: in the dock, facing it
            x, y, yaw, source = 0.0, 0.0, 0.0, "the map origin (no dock pose saved yet)"
        d = self.staging_distance
        return (x - d * math.cos(yaw), y - d * math.sin(yaw), yaw), source

    # ------------------------------------------------------------------ docking
    def handle_dock_request(self, request, response):
        """Navigate home and advance slowly until charging current is detected."""
        self.cancel_requested = False
        try:
            response.success, response.message = self.dock()
        except DockingCancelled as e:
            response.success, response.message = False, f"Docking stopped: {e}"
        finally:
            self.goal_handle = None
            self.stop_wheels()
        return response

    def dock(self):
        self.get_logger().info('Docking request received. Starting 2-step docking algorithm...')

        # Step 1: use Nav2 and the visual SLAM pose to reach a safe staging position.
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            return False, "Nav2 action server is not available. Cannot find the staging pose."

        (sx, sy, syaw), source = self.staging_pose()
        self.get_logger().info(f"Navigating to staging pose: X={sx:.2f}, Y={sy:.2f}, yaw={syaw:.2f} (map frame, from {source})...")
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = sx
        goal_msg.pose.pose.position.y = sy
        goal_msg.pose.pose.orientation.z = math.sin(syaw / 2)
        goal_msg.pose.pose.orientation.w = math.cos(syaw / 2)

        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        if not self.wait_for_future(send_goal_future, 10.0):
            return False, "Nav2 did not answer the staging pose goal."
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            return False, "Nav2 rejected the staging pose target."
        self.goal_handle = goal_handle
        self.check_cancel()  # A cancel that came while the goal was sent

        self.get_logger().info("Staging pose accepted. Waiting for arrival...")
        result_future = goal_handle.get_result_async()
        self.wait_for_future(result_future)
        self.goal_handle = None
        status = result_future.result().status
        if status != 4:  # 4 = SUCCEEDED in action status codes
            return False, f"Nav2 could not navigate to the staging pose. Status: {status}"

        self.get_logger().info("Reached the staging pose successfully! Starting step 2: controlled side-docking...")

        # Step 2: enter the dock slowly while monitoring the Daly BMS charge current. The time
        # limit is the maximum permitted travel distance (time spent paused for a hazard not counted).
        self.is_charging = False
        cmd = Twist()
        cmd.linear.x = self.dock_speed
        driven = 0.0
        while driven < self.max_distance:
            self.check_cancel()
            if self.is_charging:
                self.get_logger().info(f"Charging detected! Voltage: {self.current_voltage:.2f}V, Current: {self.current_charge_amps:.2f}A.")
                self.stop_wheels()
                self.save_dock_pose()
                return True, "Charging contact established! OmniMow is now docked and charging."
            if self.hazard:
                self.stop_wheels()  # Something (a dog) in front of the dock: wait
            else:
                self.cmd_vel_pub.publish(cmd)
                driven += self.dock_speed * 0.1
            time.sleep(0.1)

        self.stop_wheels()
        return False, f"Docking failed: drove {self.max_distance} meters without detecting voltage/current (timeout)."

    def handle_undock_request(self, request, response):
        """Back out of the charging station and rotate clear of the dock tower."""
        self.get_logger().info('Undocking request received. Backing out of the charging station...')
        self.cancel_requested = False
        # Still in the dock: this is where docking must lead back to
        if self.is_charging:
            self.save_dock_pose()
        try:
            # 1. Drive slowly backward for 8 seconds (~64 cm) to back out of the charging contacts
            self.drive_for(-0.08, 0.0, 8.0)
            # 2. Turn left, away from the dock tower, for 3 seconds
            self.drive_for(0.0, 0.3, 3.0)
        except DockingCancelled as e:
            response.success, response.message = False, f"Undocking stopped: {e}"
            return response

        self.get_logger().info("Undocking complete! OmniMow is free from the charger and ready to work.")
        response.success = True
        response.message = "Undocking completed successfully."
        return response


def main(args=None):
    rclpy.init(args=args)
    node = OmniMowDockingController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_wheels()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
