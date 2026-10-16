#!/usr/bin/env python3
"""
OmniMow mowing controller.

Mission: undock -> mow every zone (perimeter passes, then stripes) -> dock. Along the way it
  * runs the blade only while following a pass, never while driving between passes,
  * stops the blade and waits when the AI sees a hazard (dog) in front of the mower,
  * drives home to charge when the battery is low and resumes where it stopped,
  * keeps Nav2 inside the lawn and out of flower beds with a keepout costmap filter,
  * saves progress, so a stopped or interrupted mission continues instead of starting over.

Zones are stored in /opt/omnimow/zones.json in the SLAM map frame (metres, RTAB-Map's map of the
garden), so they stay valid across reboots as long as the same map is used.

Automatic lawn mapping (/omnimow/zones/auto_map): the mower drives along the lawn edge by itself
(edge on the right, see edge_follow.py) until it is back where it started, and saves the driven
loop as a new zone.

Services (std_srvs/Trigger)
  /omnimow/start_mowing, /omnimow/stop_mowing, /omnimow/return_home, /omnimow/zones/auto_map
  /omnimow/zones/record_start, /omnimow/zones/record_save_zone,
  /omnimow/zones/record_save_exclusion, /omnimow/zones/record_cancel
Topics
  /cutter/speed (Int32 ERPM), /mower/mission (String JSON status)
"""
import hashlib
import json
import math
import os
import threading
import time

import numpy as np
import rclpy
import shapely
import shapely.geometry
import shapely.ops
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import FollowPath, NavigateToPose
from nav2_msgs.msg import CostmapFilterInfo
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import BatteryState, LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Int32, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

import coverage
import edge_follow

ZONES_FILE = os.getenv('OMNIMOW_ZONES_FILE', '/opt/omnimow/zones.json')
PROGRESS_FILE = os.getenv('OMNIMOW_PROGRESS_FILE', '/opt/omnimow/mowing_progress.json')
GOAL_SUCCEEDED = 4  # action_msgs/GoalStatus.STATUS_SUCCEEDED
MOWER_STUCK, MOWER_ESTOP = 4, 5  # /mower/state from the drive ESP32
MAP_FRAME = 'map'                # RTAB-Map's garden map (map -> odom -> base_link)


def env_float(name, default):
    return float(os.getenv(name, str(default)))


class MissionInterrupted(Exception):
    """Raised inside the mission thread when it has to stop (stop request, e-stop)."""


class MowingController(Node):
    def __init__(self):
        super().__init__('mowing_controller')
        self.cb = ReentrantCallbackGroup()

        # Settings from omnimow.env
        self.cut_width = env_float('OMNIMOW_CUT_WIDTH', 0.18)
        self.overlap = env_float('OMNIMOW_MOW_OVERLAP', 0.03)
        self.edge_margin = env_float('OMNIMOW_MOW_EDGE_MARGIN', 0.0)
        self.perimeter_passes = int(env_float('OMNIMOW_MOW_PERIMETER_PASSES', 1))
        self.cutter_erpm = int(env_float('OMNIMOW_CUTTER_ERPM', 25000))
        self.battery_return = env_float('OMNIMOW_MOW_BATTERY_RETURN', 0.25)
        self.battery_resume = env_float('OMNIMOW_MOW_BATTERY_RESUME', 0.95)
        self.keepout_clearance = env_float('OMNIMOW_ROBOT_RADIUS', 0.28) + 0.05
        self.hazard_clear_sec = env_float('OMNIMOW_MOW_HAZARD_CLEAR_SEC', 5.0)

        # Live robot state
        self.pose = None            # (x, y, yaw) in the map frame
        self.battery_pct = None
        self.battery_current = 0.0
        self.hazard = False
        self.hazard_last_seen = 0.0
        self.mower_state = 0
        self.blade_erpm = 0

        # Mission state
        self.state = 'IDLE'
        self.message = ''
        self.mission_thread = None
        self.home_thread = None
        self.stop_requested = False
        self.dock_after_stop = False
        self.plan = []
        self.plan_id = None
        self.pass_index = 0
        self.skipped = 0

        # Zone recording
        self.recording = None  # list of [x, y] (map frame) while recording

        # Publishers
        self.cutter_pub = self.create_publisher(Int32, '/cutter/speed', 10)
        # Edge following drives directly, but through Nav2's collision monitor (stop/slow zones)
        self.drive_pub = self.create_publisher(Twist, 'cmd_vel_smoothed', 10)
        self.mission_pub = self.create_publisher(String, '/mower/mission', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.filter_info_pub = self.create_publisher(CostmapFilterInfo, '/costmap_filter_info', latched)
        self.mask_pub = self.create_publisher(OccupancyGrid, '/keepout_mask', latched)
        self.keepout = None  # (allowed polygon, forbidden polygons) in the map frame
        self.keepout_active = False

        # Subscriptions
        self.create_subscription(BatteryState, '/battery_state', self.battery_cb, 10, callback_group=self.cb)
        self.create_subscription(Bool, '/vision/hazard', self.hazard_cb, 10, callback_group=self.cb)
        self.create_subscription(Int32, '/mower/state', self.mower_state_cb, 10, callback_group=self.cb)
        # Everything that is not mowable lawn, in base_link, for edge following
        self.edge_sources = {}  # topic -> (time, (N, 2) points)
        for topic in ('/vision/boundary_points', '/vision/obstacle_points'):
            self.create_subscription(PointCloud2, topic, lambda msg, t=topic: self.cloud_cb(t, msg), 10,
                                     callback_group=self.cb)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10, callback_group=self.cb)
        self.edge_distance = env_float('OMNIMOW_EDGE_DISTANCE', 0.28)
        self.auto_map_max_min = env_float('OMNIMOW_AUTO_MAP_MAX_MIN', 60.0)
        self.mapper = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.1, self.update_pose, callback_group=self.cb)

        # Nav2 and other nodes
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=self.cb)
        self.follow_client = ActionClient(self, FollowPath, 'follow_path', callback_group=self.cb)
        self.dock_cli = self.create_client(Trigger, '/omnimow/dock', callback_group=self.cb)
        self.undock_cli = self.create_client(Trigger, '/omnimow/undock', callback_group=self.cb)

        # Services
        for name, handler in [
            ('/omnimow/start_mowing', self.handle_start),
            ('/omnimow/stop_mowing', self.handle_stop),
            ('/omnimow/return_home', self.handle_return_home),
            ('/omnimow/zones/record_start', self.handle_record_start),
            ('/omnimow/zones/record_save_zone', self.handle_record_save_zone),
            ('/omnimow/zones/record_save_exclusion', self.handle_record_save_exclusion),
            ('/omnimow/zones/record_cancel', self.handle_record_cancel),
            ('/omnimow/zones/auto_map', self.handle_auto_map),
        ]:
            self.create_service(Trigger, name, handler, callback_group=self.cb)

        self.create_timer(1.0, self.publish_status, callback_group=self.cb)
        self.set_keepout(False)
        self.get_logger().info(f"Mowing controller ready. Zones: {ZONES_FILE}")

    # ------------------------------------------------------------------ callbacks
    def update_pose(self):
        """Robot pose in the map frame from TF (RTAB-Map map -> odom, FusionCore odom -> base_link)."""
        try:
            tf = self.tf_buffer.lookup_transform(MAP_FRAME, 'base_link', Time())
        except TransformException:
            return
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (t.x, t.y, yaw)
        # Zone recording: a point every 25 cm while the mower is driven along the edge
        if self.recording is not None:
            if not self.recording or math.dist(self.recording[-1], self.pose[:2]) >= 0.25:
                self.recording.append([round(t.x, 3), round(t.y, 3)])

    def battery_cb(self, msg):
        if msg.voltage > 0:
            self.battery_pct = msg.percentage
            self.battery_current = msg.current

    def hazard_cb(self, msg):
        if msg.data:
            self.hazard_last_seen = time.time()
            if not self.hazard and self.blade_erpm:
                self.get_logger().warn("Hazard in view - blade off")
            self.hazard = True
            self.set_blade(False)  # Immediately, not only when the mission loop notices
        elif time.time() - self.hazard_last_seen > self.hazard_clear_sec:
            self.hazard = False

    def mower_state_cb(self, msg):
        self.mower_state = msg.data
        if msg.data in (MOWER_STUCK, MOWER_ESTOP):
            self.set_blade(False)

    # ------------------------------------------------------------------ blade / status
    def set_blade(self, on):
        erpm = self.cutter_erpm if on else 0
        if erpm != self.blade_erpm:
            self.get_logger().info(f"Blade {'ON' if on else 'OFF'}")
        self.blade_erpm = erpm
        self.cutter_pub.publish(Int32(data=erpm))

    def set_state(self, state, message=''):
        if state != self.state or message != self.message:
            self.get_logger().info(f"State {state}{': ' + message if message else ''}")
        self.state, self.message = state, message

    def publish_status(self):
        # Re-send the blade command every second (the cutter ESP32 keeps the last value)
        self.cutter_pub.publish(Int32(data=self.blade_erpm))
        total = len(self.plan)
        self.mission_pub.publish(String(data=json.dumps({
            "state": self.state,
            "active": bool(self.mission_thread and self.mission_thread.is_alive()),
            "message": self.message,
            "pass": self.pass_index,
            "total_passes": total,
            "progress_pct": round(100.0 * self.pass_index / total, 1) if total else 0.0,
            "skipped_passes": self.skipped,
            "blade_on": self.blade_erpm != 0,
            "hazard": self.hazard,
            "recording_points": len(self.recording) if self.recording is not None else None,
            # Position on the SLAM map (metres, map origin = charging station) for the app's map
            "pose": [round(self.pose[0], 2), round(self.pose[1], 2), round(self.pose[2], 3)] if self.pose else None,
            "edge_mapping": {"state": self.mapper.state, "loop_length_m": round(self.mapper.length, 1)}
            if self.mapper else None,
        })))

    # ------------------------------------------------------------------ services
    def handle_start(self, request, response):
        if self.mission_thread and self.mission_thread.is_alive():
            response.success, response.message = False, f"Mission already running ({self.state})"
            return response
        self.stop_requested = False
        self.dock_after_stop = False
        self.mission_thread = threading.Thread(target=self.run_mission, daemon=True)
        self.mission_thread.start()
        response.success, response.message = True, "Mowing started"
        return response

    def handle_stop(self, request, response):
        self.request_stop(dock=False)
        response.success, response.message = True, "Mowing stopped - progress is kept"
        return response

    def handle_return_home(self, request, response):
        if self.mission_thread and self.mission_thread.is_alive():
            self.request_stop(dock=True)
        elif not (self.home_thread and self.home_thread.is_alive()):
            self.home_thread = threading.Thread(target=self.go_home, daemon=True)
            self.home_thread.start()
        response.success, response.message = True, "Returning to the charging station"
        return response

    def request_stop(self, dock):
        self.stop_requested = True
        self.dock_after_stop = dock
        self.set_blade(False)

    def handle_record_start(self, request, response):
        self.recording = []
        if self.pose is None:
            self.recording = None
            response.success, response.message = False, "No map position yet - is slam/localization running?"
            return response
        response.success, response.message = True, "Recording: drive the mower along the edge of the lawn"
        return response

    def cloud_cb(self, topic, msg):
        pts = point_cloud2.read_points_numpy(msg, field_names=['x', 'y'], skip_nans=True)
        self.edge_sources[topic] = (time.time(), np.asarray(pts, dtype=float).reshape(-1, 2))

    def scan_cb(self, msg):
        r = np.asarray(msg.ranges, dtype=float)
        a = msg.angle_min + np.arange(len(r)) * msg.angle_increment
        ok = np.isfinite(r)
        # The stereo node publishes the scan in base_link already
        self.edge_sources['/scan'] = (time.time(), np.column_stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])]))

    def edge_points(self, max_age=0.5):
        fresh = [pts for t, pts in self.edge_sources.values() if time.time() - t < max_age]
        return np.vstack(fresh) if fresh else np.zeros((0, 2))

    def handle_auto_map(self, request, response):
        if self.mission_thread and self.mission_thread.is_alive():
            response.success, response.message = False, f"Mission already running ({self.state})"
            return response
        if '/vision/boundary_points' not in self.edge_sources:
            response.success, response.message = False, "No lawn edge data - is depth_fusion running with OMNIMOW_AI_GRASS_BOUNDARY=true?"
            return response
        self.stop_requested = False
        self.dock_after_stop = True
        self.mission_thread = threading.Thread(target=self.run_auto_map, daemon=True)
        self.mission_thread.start()
        response.success, response.message = True, "Edge mapping started - the mower drives along the lawn edge"
        return response

    def handle_record_cancel(self, request, response):
        self.recording = None
        response.success, response.message = True, "Recording cancelled"
        return response

    def handle_record_save_zone(self, request, response):
        points, error = self.take_recording()
        if error:
            response.success, response.message = False, error
            return response
        zones = self.load_zones()
        name = f"zone_{len(zones['zones']) + 1}"
        zones['zones'].append({"name": name, "enabled": True, "boundary": points, "exclusions": []})
        self.save_zones(zones)
        response.success, response.message = True, f"Saved {name} with {len(points)} points"
        return response

    def handle_record_save_exclusion(self, request, response):
        points, error = self.take_recording()
        if error:
            response.success, response.message = False, error
            return response
        zones = self.load_zones()
        centre = shapely.geometry.Polygon(points).centroid
        for zone in zones['zones']:
            if shapely.geometry.Polygon(zone['boundary']).contains(centre):
                zone.setdefault('exclusions', []).append(points)
                self.save_zones(zones)
                response.success, response.message = True, f"Saved exclusion in {zone['name']}"
                return response
        response.success, response.message = False, "The exclusion is not inside any saved zone"
        return response

    def take_recording(self):
        points, self.recording = self.recording, None
        if points is None:
            return None, "Recording was not started"
        if len(points) < 3:
            return None, f"Only {len(points)} points recorded - drive a longer loop"
        return points, None

    # ------------------------------------------------------------------ zones & plan
    @staticmethod
    def load_zones():
        try:
            with open(ZONES_FILE) as f:
                zones = json.load(f)
            zones.setdefault('zones', [])
            zones.setdefault('frame', MAP_FRAME)
            return zones
        except FileNotFoundError:
            return {"frame": MAP_FRAME, "zones": []}

    @staticmethod
    def save_zones(zones):
        os.makedirs(os.path.dirname(ZONES_FILE), exist_ok=True)
        tmp = ZONES_FILE + '.tmp'
        with open(tmp, 'w') as f:
            json.dump(zones, f, indent=2)
        os.replace(tmp, ZONES_FILE)

    def build_plan(self):
        zones = [z for z in self.load_zones()['zones'] if z.get('enabled', True)]
        if not zones:
            raise RuntimeError(f"No zones defined in {ZONES_FILE}")
        settings = [self.cut_width, self.overlap, self.edge_margin, self.perimeter_passes]
        plan_id = hashlib.sha1(json.dumps([zones, settings], sort_keys=True).encode()).hexdigest()[:12]

        plan, allowed, forbidden = [], [], []
        start = self.pose[:2] if self.pose else (0.0, 0.0)
        for zone in zones:
            boundary = [tuple(p) for p in zone['boundary']]
            exclusions = [[tuple(p) for p in e] for e in zone.get('exclusions', [])]
            passes = coverage.plan_zone(boundary, exclusions, self.cut_width, self.overlap,
                                        self.edge_margin, self.perimeter_passes,
                                        angle=zone.get('stripe_angle_rad'), start=start)
            for p in passes:
                p['zone'] = zone['name']
            plan.extend(passes)
            if passes:
                start = passes[-1]['points'][-1]
            a, f = coverage.keepout_area(boundary, exclusions, self.keepout_clearance)
            allowed.append(a)
            forbidden.extend(f)
            self.get_logger().info(f"Zone {zone['name']}: {len(passes)} passes")
        self.keepout = (shapely.ops.unary_union(allowed), forbidden)
        return plan, plan_id

    def load_progress(self, plan_id):
        try:
            with open(PROGRESS_FILE) as f:
                progress = json.load(f)
            if progress.get('plan_id') == plan_id:
                return int(progress.get('next_pass', 0)), int(progress.get('skipped', 0))
        except (FileNotFoundError, ValueError):
            pass
        return 0, 0

    def save_progress(self, next_pass):
        try:
            with open(PROGRESS_FILE, 'w') as f:
                json.dump({"plan_id": self.plan_id, "next_pass": next_pass, "skipped": self.skipped}, f)
        except OSError as e:
            self.get_logger().error(f"Could not save progress: {e}")

    # ------------------------------------------------------------------ keepout filter
    def set_keepout(self, active):
        """Publish the keepout mask (or an empty one). Only active while the mower is on the lawn,
        otherwise the mower could not leave or reach a dock that stands outside the lawn."""
        info = CostmapFilterInfo(type=0, filter_mask_topic='/keepout_mask', base=0.0, multiplier=1.0)
        info.header.frame_id = MAP_FRAME
        info.header.stamp = self.get_clock().now().to_msg()
        self.filter_info_pub.publish(info)

        grid = OccupancyGrid()
        grid.header = info.header
        grid.info.resolution = 0.1
        if active and self.keepout:
            allowed, forbidden = self.keepout
            minx, miny, maxx, maxy = allowed.bounds
            minx, miny = minx - 3.0, miny - 3.0
            w = int(math.ceil((maxx + 3.0 - minx) / 0.1))
            h = int(math.ceil((maxy + 3.0 - miny) / 0.1))
            xs = minx + (np.arange(w) + 0.5) * 0.1
            ys = miny + (np.arange(h) + 0.5) * 0.1
            gx, gy = np.meshgrid(xs, ys)
            blocked = ~shapely.contains_xy(allowed, gx, gy)
            for poly in forbidden:
                blocked |= shapely.contains_xy(poly, gx, gy)
            grid.info.width, grid.info.height = w, h
            grid.info.origin.position.x, grid.info.origin.position.y = minx, miny
            grid.data = (blocked.astype(np.int8) * 100).ravel().tolist()
        else:
            grid.info.width = grid.info.height = 1
            grid.data = [0]
        grid.info.origin.orientation.w = 1.0
        self.mask_pub.publish(grid)
        self.keepout_active = active and self.keepout is not None

    def update_keepout(self):
        if self.keepout and self.pose and not self.keepout_active:
            if shapely.contains_xy(self.keepout[0], self.pose[0], self.pose[1]):
                self.set_keepout(True)

    # ------------------------------------------------------------------ helpers
    def wait(self, future, timeout=None):
        """Wait for a future completed by the executor thread. Returns None on timeout."""
        start = time.time()
        while not future.done():
            if timeout is not None and time.time() - start > timeout:
                return None
            time.sleep(0.05)
        return future.result()

    def check_abort(self):
        if self.stop_requested:
            raise MissionInterrupted("stop requested")
        if self.mower_state in (MOWER_STUCK, MOWER_ESTOP):
            raise MissionInterrupted("mower stuck" if self.mower_state == MOWER_STUCK else "emergency stop")

    def battery_low(self):
        return self.battery_pct is not None and self.battery_pct < self.battery_return

    def is_docked(self):
        return self.battery_current > 0.08  # Same charging detection as docking_node

    def call_trigger(self, client, name, timeout=600.0):
        if not client.wait_for_service(timeout_sec=5.0):
            return False, f"{name} not available"
        res = self.wait(client.call_async(Trigger.Request()), timeout)
        if res is None:
            return False, f"{name} timed out"
        return res.success, res.message

    def make_pose(self, x, y, yaw):
        ps = PoseStamped()
        ps.header.frame_id = MAP_FRAME
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y = x, y
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return ps

    def run_action(self, client, goal, interrupt_on_hazard):
        """Send a Nav2 goal and watch it. Returns 'ok', 'failed', 'hazard' or 'battery'."""
        if not client.wait_for_server(timeout_sec=10.0):
            return 'failed'
        handle = self.wait(client.send_goal_async(goal), 10.0)
        if handle is None or not handle.accepted:
            return 'failed'
        result_future = handle.get_result_async()
        reason = None
        while not result_future.done():
            if self.stop_requested or self.mower_state in (MOWER_STUCK, MOWER_ESTOP):
                reason = 'stop'
            elif interrupt_on_hazard and self.hazard:
                reason = 'hazard'
            elif self.battery_low():
                reason = 'battery'
            if reason:
                self.wait(handle.cancel_goal_async(), 5.0)
                self.wait(result_future, 10.0)
                break
            self.update_keepout()
            time.sleep(0.1)
        if reason == 'stop':
            self.check_abort()
        if reason:
            return reason
        return 'ok' if result_future.result().status == GOAL_SUCCEEDED else 'failed'

    # ------------------------------------------------------------------ mission
    def run_mission(self):
        try:
            self.set_state('PLANNING')
            self.plan, self.plan_id = self.build_plan()
            self.pass_index, self.skipped = self.load_progress(self.plan_id)
            if self.pass_index >= len(self.plan):
                self.pass_index, self.skipped = 0, 0
            self.get_logger().info(f"Plan {self.plan_id}: {len(self.plan)} passes, starting at {self.pass_index}")

            if self.is_docked():
                self.leave_dock()

            consecutive_failures = 0
            while self.pass_index < len(self.plan):
                self.check_abort()
                if self.mow_pass(self.plan[self.pass_index]):
                    consecutive_failures = 0
                else:
                    self.skipped += 1
                    consecutive_failures += 1
                    if consecutive_failures >= 5:
                        # Probably not an obstacle but a problem (localization, Nav2): retry these next time
                        self.skipped -= consecutive_failures
                        self.pass_index -= consecutive_failures - 1
                        self.save_progress(self.pass_index)
                        raise RuntimeError("5 passes in a row could not be mowed")
                self.pass_index += 1
                self.save_progress(self.pass_index)

            self.set_blade(False)
            self.save_progress(0)
            self.set_state('RETURNING', 'Lawn finished')
            self.go_home(final_state='DONE')
        except MissionInterrupted as e:
            self.set_blade(False)
            self.set_keepout(False)
            self.set_state('STOPPED', str(e))
            if self.dock_after_stop:
                self.go_home()
        except Exception as e:
            self.set_blade(False)
            self.set_keepout(False)
            self.set_state('ERROR', str(e))
            self.get_logger().error(f"Mission failed: {e}")
            if self.mower_state not in (MOWER_STUCK, MOWER_ESTOP):
                self.go_home(final_state='ERROR', message=str(e))

    # ------------------------------------------------------------------ automatic edge mapping
    def drive(self, v, w):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(v), float(w)
        self.drive_pub.publish(msg)

    def run_auto_map(self):
        try:
            self.set_blade(False)  # Mapping only - the blade stays off
            self.set_keepout(False)
            if self.is_docked():
                self.leave_dock()
            if self.pose is None:
                raise RuntimeError("No map position - is slam/localization running?")
            self.mapper = edge_follow.EdgeMapper(edge_follow.EdgeParams(target=self.edge_distance))
            started = time.time()
            seek_start = self.pose[:2]
            self.set_state('MAPPING', 'Looking for the lawn edge')
            while True:
                self.check_abort()
                if self.hazard:
                    self.drive(0.0, 0.0)
                    self.wait_for_hazard_clear()
                    self.set_state('MAPPING', 'Following the lawn edge')
                if self.battery_low():
                    raise RuntimeError("Battery low - edge mapping aborted, start it again when charged")
                if time.time() - started > self.auto_map_max_min * 60:
                    raise RuntimeError(f"Edge mapping did not close the loop within {self.auto_map_max_min:.0f} min")
                if not self.mapper.edge_found and math.dist(seek_start, self.pose[:2]) > 30.0:
                    raise RuntimeError("No lawn edge found within 30 m")

                v, w, closed = self.mapper.step(self.edge_points(), self.pose)
                if closed:
                    break
                if self.mapper.edge_found:
                    self.set_state('MAPPING', f"Following the lawn edge ({self.mapper.length:.0f} m)")
                self.drive(v, w)
                time.sleep(0.1)

            self.drive(0.0, 0.0)
            zones = self.load_zones()
            name = f"lawn_{len(zones['zones']) + 1}"
            zones['zones'].append({"name": name, "enabled": True, "boundary": self.mapper.boundary(),
                                   "exclusions": [], "source": "edge_mapping"})
            self.save_zones(zones)
            message = f"Saved {name}: {self.mapper.length:.0f} m edge, area {shapely.geometry.Polygon(self.mapper.boundary()).area:.0f} m2"
            self.get_logger().info(message)
            self.mapper = None
            self.set_state('RETURNING', message)
            self.go_home(final_state='DONE', message=message)
        except MissionInterrupted as e:
            self.drive(0.0, 0.0)
            self.mapper = None
            self.set_state('STOPPED', f"Edge mapping stopped: {e}")
            if self.dock_after_stop:
                self.go_home()
        except Exception as e:
            self.drive(0.0, 0.0)
            self.mapper = None
            self.set_state('ERROR', str(e))
            self.get_logger().error(f"Edge mapping failed: {e}")
            if self.mower_state not in (MOWER_STUCK, MOWER_ESTOP):
                self.go_home(final_state='ERROR', message=str(e))

    def leave_dock(self):
        self.set_state('UNDOCKING')
        ok, msg = self.call_trigger(self.undock_cli, 'undock', 60.0)
        if not ok:
            raise RuntimeError(f"Undock failed: {msg}")

    def go_home(self, final_state='CHARGING', message=''):
        self.set_blade(False)
        self.set_keepout(False)
        self.set_state('RETURNING', message)
        ok, msg = self.call_trigger(self.dock_cli, 'dock')
        self.set_state(final_state if ok else 'ERROR', message if ok else f"Docking failed: {msg}")
        return ok

    def charge_and_resume(self):
        """Low battery: dock, wait until charged, undock and carry on."""
        self.set_blade(False)
        self.save_progress(self.pass_index)
        self.set_state('RETURNING', 'Battery low')
        self.set_keepout(False)
        ok, msg = self.call_trigger(self.dock_cli, 'dock')
        if not ok:
            raise RuntimeError(f"Docking for charging failed: {msg}")
        self.set_state('CHARGING', f"Resuming at {int(self.battery_resume * 100)}%")
        while self.battery_pct is None or self.battery_pct < self.battery_resume:
            self.check_abort()
            time.sleep(5.0)
        self.leave_dock()

    def wait_for_hazard_clear(self):
        self.set_blade(False)
        self.set_state('PAUSED', 'Hazard in view (dog) - waiting')
        while self.hazard:
            self.check_abort()
            time.sleep(0.2)

    def mow_pass(self, mow_pass):
        """Drive to the start (blade off), then follow the pass with the blade on.
        Returns True when the pass is done, False when it had to be skipped."""
        remaining = coverage.densify(mow_pass['points'])
        while True:
            self.check_abort()
            if self.hazard:
                self.wait_for_hazard_clear()
            if self.battery_low():
                self.charge_and_resume()
            label = f"{mow_pass['zone']} {mow_pass['type']} {self.pass_index + 1}/{len(self.plan)}"

            # 1. Transition to the start of the (remaining) pass with Nav2's planner, blade off
            self.set_blade(False)
            self.set_state('MOWING', f"{label}: driving to start")
            (x0, y0), (x1, y1) = remaining[0], remaining[min(1, len(remaining) - 1)]
            goal = NavigateToPose.Goal(pose=self.make_pose(x0, y0, math.atan2(y1 - y0, x1 - x0)))
            result = self.run_action(self.nav_client, goal, interrupt_on_hazard=False)
            if result in ('hazard', 'battery'):
                continue
            if result != 'ok':
                self.get_logger().warn(f"{label}: start not reachable - skipping")
                return False

            # 2. Blade on, let it spin up, then follow the pass with the controller
            if self.hazard:
                continue
            self.set_blade(True)
            time.sleep(2.0)
            self.set_state('MOWING', label)
            path = Path()
            path.header.frame_id = MAP_FRAME
            path.header.stamp = self.get_clock().now().to_msg()
            yaw = 0.0
            for i, (xa, ya) in enumerate(remaining):
                if i + 1 < len(remaining):  # Heading towards the next point; the last point keeps the previous one
                    xb, yb = remaining[i + 1]
                    yaw = math.atan2(yb - ya, xb - xa)
                path.poses.append(self.make_pose(xa, ya, yaw))
            goal = FollowPath.Goal(path=path, controller_id='FollowPath',
                                   goal_checker_id='general_goal_checker')
            result = self.run_action(self.follow_client, goal, interrupt_on_hazard=True)
            self.set_blade(False)
            if result == 'ok':
                return True
            if result in ('hazard', 'battery') and self.pose:
                # Continue the same pass from the point closest to where the mower stopped
                pts = np.array(remaining)
                nearest = int(np.argmin(np.hypot(pts[:, 0] - self.pose[0], pts[:, 1] - self.pose[1])))
                remaining = remaining[nearest:]
                if len(remaining) < 3:
                    return True
                continue
            self.get_logger().warn(f"{label}: blocked (obstacle?) - skipping the rest of this pass")
            return False


def main(args=None):
    rclpy.init(args=args)
    node = MowingController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.set_blade(False)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
