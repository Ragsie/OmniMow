#!/usr/bin/env python3
import os
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import BatteryState
from nav2_msgs.action import NavigateToPose

class OmniMowDockingController(Node):
    """Navigate to a staging pose, then use slow motion and charging current to dock."""

    def __init__(self):
        super().__init__('omnimow_docking_controller')
        self.get_logger().info('OmniMow docking and charging control started.')

        # 1. Load configuration parameters from omnimow.env
        self.staging_x = float(os.getenv('OMNIMOW_DOCK_STAGING_X', '5.20'))
        self.staging_y = float(os.getenv('OMNIMOW_DOCK_STAGING_Y', '2.45'))
        self.staging_yaw = float(os.getenv('OMNIMOW_DOCK_STAGING_YAW', '1.57'))
        self.dock_speed = float(os.getenv('OMNIMOW_DOCK_SPEED', '0.05')) # Very slow and safe driving
        self.max_distance = float(os.getenv('OMNIMOW_DOCK_MAX_DISTANCE', '2.0')) # Safety timer (meters)

        # 2. Create pub/sub and action clients
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.battery_sub = None # Dynamically created during local docking to save resources
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 3. System status flags
        self.is_charging = False
        self.current_voltage = 0.0
        self.current_charge_amps = 0.0

        # 4. Create services for external control (app and backend)
        self.dock_srv = self.create_service(Trigger, '/omnimow/dock', self.handle_dock_request)
        self.undock_srv = self.create_service(Trigger, '/omnimow/undock', self.handle_undock_request)

    def battery_callback(self, msg: BatteryState):
        """Update charging state from the battery data published by the ESP32."""
        self.current_voltage = msg.voltage
        self.current_charge_amps = msg.current # Positive value indicates charging current [cite: 5]

        # If charging voltage is active (Daly BMS reports positive current, typically >50mA) [cite: 5]
        if self.current_charge_amps > 0.08:
            self.is_charging = True

    def handle_dock_request(self, request, response):
        """Navigate home and advance slowly until charging current is detected."""
        self.get_logger().info('Docking request received. Starting 2-step docking algorithm...')
        self.is_charging = False

        # Step 1: use Nav2 and the fused RTK pose to reach a safe staging position.
        if not self.nav_client.wait_for_action_server(timeout_sec=5.0):
            response.success = False
            response.message = "Nav2 action server is not available. Cannot find the staging pose."
            return response

        self.get_logger().info(f"Navigating to staging pose: X={self.staging_x}, Y={self.staging_y} using RTK-GPS...")
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = self.staging_x
        goal_msg.pose.pose.position.y = self.staging_y

        # Convert yaw (Euler angle) to quaternion rotation
        from math import sin, cos
        half_yaw = self.staging_yaw * 0.5
        goal_msg.pose.pose.orientation.z = sin(half_yaw)
        goal_msg.pose.pose.orientation.w = cos(half_yaw)

        # Send target to Nav2
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            response.success = False
            response.message = "Nav2 rejected the staging pose target."
            return response

        self.get_logger().info("Staging pose accepted. Waiting for arrival...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        status = result_future.result().status
        if status != 4: # 4 = SUCCEEDED in action status codes
            response.success = False
            response.message = f"Nav2 could not navigate to the staging pose. Status: {status}"
            return response

        self.get_logger().info("Reached the staging pose successfully! Starting step 2: controlled side-docking...")

        # Step 2: enter the dock slowly while monitoring the Daly BMS charge current.
        self.battery_sub = self.create_subscription(
            BatteryState,
            '/battery_state',
            self.battery_callback,
            10
        )

        # The timeout converts the maximum permitted travel distance into a time limit.
        rate = self.create_rate(10) # 10 Hz
        start_time = time.time()
        timeout_duration = self.max_distance / self.dock_speed # e.g. 2.0m / 0.05m/s = 40 seconds

        twist_cmd = Twist()
        twist_cmd.linear.x = self.dock_speed
        twist_cmd.angular.z = 0.0

        success = False
        while (time.time() - start_time) < timeout_duration:
            # Ensure rclpy callbacks run so the battery measurements are refreshed
            rclpy.spin_once(self, timeout_sec=0.01)

            if self.is_charging:
                self.get_logger().info(f"Charging detected! Voltage: {self.current_voltage:.2f}V, Current: {self.current_charge_amps:.2f}A.")
                success = True
                break

            # Send drive command
            self.cmd_vel_pub.publish(twist_cmd)
            rate.sleep()

        # Stop the wheels immediately regardless of outcome
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)

        # Unsubscribe from battery updates to clean up
        self.destroy_subscription(self.battery_sub)
        self.battery_sub = None

        if success:
            response.success = True
            response.message = "Charging contact established! OmniMow is now docked and charging."
        else:
            response.success = False
            response.message = f"Docking failed: drove {self.max_distance} meters without detecting voltage/current (timeout)."

        return response

    def handle_undock_request(self, request, response):
        """Back out of the charging station and rotate clear of the dock tower."""
        self.get_logger().info('Undocking request received. Backing out of the charging station...')

        rate = self.create_rate(10)

        # 1. Drive slowly backward for 8 seconds to back out of the charging contacts
        back_cmd = Twist()
        back_cmd.linear.x = -0.08 # m/s
        back_cmd.angular.z = 0.0

        for _ in range(80): # 80 loops * 0.1s = 8 seconds (~64 cm)
            self.cmd_vel_pub.publish(back_cmd)
            rate.sleep()

        # 2. Rotate 0.3 rad/s away for 3 seconds so the dock tower is fully cleared
        turn_cmd = Twist()
        turn_cmd.linear.x = 0.0
        turn_cmd.angular.z = 0.3 # Turns gently to the left (opposite the tower)

        for _ in range(30): # 30 loops * 0.1s = 3 seconds
            self.cmd_vel_pub.publish(turn_cmd)
            rate.sleep()

        # 3. Stop completely
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)

        self.get_logger().info("Undocking complete! OmniMow is free from the charger and ready to work.")
        response.success = True
        response.message = "Undocking completed successfully."
        return response

def main(args=None):
    rclpy.init(args=args)
    node = OmniMowDockingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()