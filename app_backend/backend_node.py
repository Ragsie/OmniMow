#!/usr/bin/env python3
from fastapi import FastAPI, WebSocket
import uvicorn
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, BatteryState
from std_msgs.msg import Int32, Float32, Empty, Bool  # Rain sensor message type
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
import threading
import json
import asyncio
import os
import math
import time
import shutil # Added: for SSD guard

# ROS 2 to HTTP/WebSocket bridge for mower telemetry, schedules, and control.
app = FastAPI(title="OmniMow API Gateway")
clients = []

class BackendROSNode(Node):
    """Collect mower telemetry and expose a live status stream to the app."""

    def __init__(self):
        super().__init__('app_backend_node')
        self.gps_sub = self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.battery_sub = self.create_subscription(BatteryState, '/battery_state', self.battery_callback, 10)
        self.status_sub = self.create_subscription(Int32, '/mower/state', self.state_callback, 10)
        self.cutter_status_sub = self.create_subscription(Int32, '/cutter/status', self.cutter_status_callback, 10)
        self.cutter_current_sub = self.create_subscription(Float32, '/cutter/current', self.cutter_current_callback, 10)
        self.drive_current_sub = self.create_subscription(Float32, '/drive/current', self.drive_current_callback, 10)
        self.cutter_rpm_sub = self.create_subscription(Int32, '/cutter/rpm', self.cutter_rpm_callback, 10)

        # Supporting telemetry used by the app dashboard and persistent statistics.
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.satellites_sub = self.create_subscription(Int32, '/gps/satellites', self.satellites_callback, 10)
        self.charge_cycles_sub = self.create_subscription(Int32, '/battery/charge_cycles', self.charge_cycles_callback, 10)
        self.rain_sub = self.create_subscription(Bool, '/mower/rain', self.rain_callback, 10) # Added: rain sensor subscriber

        # Heartbeat publisher for ESP32 hardware watchdog
        self.heartbeat_pub = self.create_publisher(Empty, '/mower/heartbeat', 10)
        self.heartbeat_timer = self.create_timer(0.2, self.publish_heartbeat) # Send a ping every 200 ms

        # Docking services are called asynchronously so the ROS executor remains responsive.
        self.dock_cli = self.create_client(Trigger, '/omnimow/dock')
        self.undock_cli = self.create_client(Trigger, '/omnimow/undock')

        # 10-second timer for schedule and rain delay logic
        self.schedule_timer = self.create_timer(10.0, self.check_schedule_timer)

        self.gps_data = {"lat": 0.0, "lon": 0.0, "status": "No GPS signal", "rtk_code": 0, "rtk_text": "No GPS signal", "satellites": 0}
        self.battery_v = 24.0
        self.battery_pct = 100.0
        self.battery_current = 0.0
        self.battery_temp = 25.0
        self.state = 0 # 0=STOP, 1=MOWING, 2=RETURNING_TO_DOCK, 3=CHARGING, 4=STUCK, 5=EMERGENCY_STOP, 6=CUTTER_BLOCKED, 7=SEARCHING_EDGE, 8=RAIN, 9=DRYING
        self.cutter_status = 0 # 0=OFF, 1=OK, 2=BLOCKED
        self.cutter_current = 0.0
        self.drive_current = 0.0
        self.cutter_rpm = 0
        self.satellites_count = 0
        self.charge_cycles = 0
        self.is_raining = False # Added: rain sensor status

        # Schedule configuration and rain drying timer
        self.rain_resume_time = 0.0  # Unix timestamp for when mowing may resume after rain
        self.rain_delay_duration = 7200  # Default drying time is 2 hours (in seconds)
        self.schedule = {
            "enabled": False,
            "days": {
                "0": [], "1": [], "2": [], "3": [], "4": [], "5": [], "6": []
            }
        }

        # Persistent statistics loaded from the Orange Pi NVMe SSD
        self.stats_file = "/opt/omnimow/stats.json"
        self.total_distance_km = 0.0
        self.total_runtime_hours = 0.0
        self.load_stats()

        self.last_x = None
        self.last_y = None
        self.last_stats_save_time = time.time()
        self.last_runtime_tick = time.time()

        # CPU monitoring
        self.last_cpu_idle = 0
        self.last_cpu_total = 0

    def load_stats(self):
        # Restore counters and user settings so a container restart does not reset them.
        if os.path.exists(self.stats_file):
            try:
                with open(self.stats_file, 'r') as f:
                    data = json.load(f)
                    self.total_distance_km = data.get("total_distance_km", 0.0)
                    self.total_runtime_hours = data.get("total_runtime_hours", 0.0)
                    self.schedule = data.get("schedule", {
                        "enabled": False,
                        "days": {"0": [], "1": [], "2": [], "3": [], "4": [], "5": [], "6": []}
                    })
                    self.rain_delay_duration = data.get("rain_delay_duration", 7200)
            except Exception as e:
                self.get_logger().error(f"Could not read stats file: {e}")
        else:
            self.save_stats()

    def save_stats(self):
        # Write all user-visible counters and scheduling settings in one JSON document.
        try:
            os.makedirs(os.path.dirname(self.stats_file), exist_ok=True)
            with open(self.stats_file, 'w') as f:
                json.dump({
                    "total_distance_km": round(self.total_distance_km, 3),
                    "total_runtime_hours": round(self.total_runtime_hours, 4),
                    "schedule": self.schedule,
                    "rain_delay_duration": self.rain_delay_duration
                }, f)
        except Exception as e:
            self.get_logger().error(f"Could not save stats file: {e}")

    def odom_callback(self, msg):
        # Added: calculate Euclidean distance driven from wheel encoders and odometry tree
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.last_x is not None and self.last_y is not None:
            dist_m = math.sqrt((x - self.last_x)**2 + (y - self.last_y)**2)
            if dist_m < 5.0: # Safety threshold to avoid jumps during odom reset
                self.total_distance_km += dist_m / 1000.0

        self.last_x = x
        self.last_y = y
        self.update_runtime_and_save()

    def update_runtime_and_save(self):
        # Added: update runtime if the machine is moving and save to disk every 10 seconds
        now = time.time()
        dt = now - self.last_runtime_tick
        self.last_runtime_tick = now

        if self.state in [1, 2, 7]: # Active operating states
            self.total_runtime_hours += dt / 3600.0

        if now - self.last_stats_save_time >= 10.0:
            self.last_stats_save_time = now
            self.save_stats()
            self.broadcast_status()

    def satellites_callback(self, msg):
        # Odometry is integrated between callbacks; large jumps are treated as localization resets.
        self.satellites_count = msg.data
        self.gps_data["satellites"] = self.satellites_count
        self.broadcast_status()

    def charge_cycles_callback(self, msg):
        # These values are forwarded immediately because the app displays them live.
        self.charge_cycles = msg.data
        self.broadcast_status()

    def gps_callback(self, msg):
        status_code = msg.status.status
        # Convert RTK/GPS status to English text and code for the app
        rtk_text = "No GPS signal"
        if status_code == 0:
            rtk_text = "Standard GPS fix (rough)"
        elif status_code == 1:
            rtk_text = "RTK float (seeking precision)"
        elif status_code == 2:
            rtk_text = "RTK centimeter fix (perfect)"

        self.gps_data = {
            "lat": msg.latitude,
            "lon": msg.longitude,
            "status": rtk_text,
            "rtk_code": status_code + 1 if status_code >= 0 else 0, # Convert to 0=No fix, 1=GPS, 2=Float, 3=Fix
            "rtk_text": rtk_text,
            "satellites": self.satellites_count
        }
        self.broadcast_status()

    def battery_callback(self, msg):
        self.battery_v = msg.voltage
        self.battery_pct = msg.percentage * 100.0
        self.battery_current = msg.current
        self.battery_temp = msg.temperature
        self.broadcast_status()

    def rain_callback(self, msg):
        # Rain has priority over scheduling: return to the dock, then wait for the configured drying time.
        was_raining = self.is_raining
        self.is_raining = msg.data

        if self.is_raining:
            self.state = 8  # RAIN (8) - drive back to the charging station
            self.rain_resume_time = 0.0  # Reset drying time while it is raining
            self.trigger_go_to_dock()
        elif was_raining and not self.is_raining:
            # Rain has just stopped! Set the drying timer.
            self.rain_resume_time = time.time() + self.rain_delay_duration
            self.state = 9  # DRYING (9) - drying in the dock
            self.get_logger().info(f"Rain stopped. Activating a {self.rain_delay_duration/3600:.1f}-hour drying timer.")

        self.broadcast_status()

    def state_callback(self, msg):
        self.state = msg.data
        self.broadcast_status()

    def cutter_status_callback(self, msg):
        self.cutter_status = msg.data
        if self.cutter_status == 2:
            self.state = 6 # CUTTER_BLOCKED system state
        self.broadcast_status()

    def cutter_current_callback(self, msg):
        self.cutter_current = msg.data
        self.broadcast_status()

    def drive_current_callback(self, msg):
        self.drive_current = msg.data
        self.broadcast_status()

    def cutter_rpm_callback(self, msg):
        self.cutter_rpm = msg.data
        self.broadcast_status()

    def check_schedule_timer(self):
        # Evaluate the drying timer and configured mowing windows periodically.
        current_time = time.time()

        # 1. Rain timer check
        if self.state == 9:
            if current_time >= self.rain_resume_time:
                self.get_logger().info("Drying timer expired! The mower is ready to operate again.")
                self.state = 0  # Set to STOP/Ready
                self.broadcast_status()

        # 2. Schedule check
        if not self.schedule.get("enabled", False):
            return

        now = time.localtime()
        weekday_str = str(now.tm_wday) # 0=Monday, 6=Sunday
        current_time_str = f"{now.tm_hour:02d}:{now.tm_min:02d}"

        active_slots = self.schedule.get("days", {}).get(weekday_str, [])
        is_mow_window = False
        for slot in active_slots:
            start = slot.get("start", "")
            end = slot.get("end", "")
            if start <= current_time_str < end:
                is_mow_window = True
                break

        if is_mow_window:
            # If we are inside the mowing window, the mower is ready (state 0), it is not raining, and we are not drying (state 9)
            if self.state == 0 and not self.is_raining and current_time >= self.rain_resume_time:
                self.get_logger().info("Mowing schedule reached! Resuming autonomous mowing.")
                self.state = 1  # Set state to MOWING (1)
                self.trigger_autonomous_mow()
        else:
            # If we are outside the mowing window and the mower is active (state 1), send it home
            if self.state == 1:
                self.get_logger().info("Mowing window expired! Automatically sending the robot back to the dock.")
                self.state = 2  # Set state to RETURNING_TO_DOCK (2)
                self.trigger_go_to_dock()

    def trigger_autonomous_mow(self):
        # The undock service starts the physical transition from the charging station to mowing.
        self.get_logger().info("Sending autonomous start command...")
        if self.undock_cli.wait_for_service(timeout_sec=1.0):
            req = Trigger.Request()
            self.undock_cli.call_async(req)

    def trigger_go_to_dock(self):
        # The docking service handles navigation to the staging pose and final contact detection.
        self.get_logger().info("Sending autonomous docking command...")
        if self.dock_cli.wait_for_service(timeout_sec=1.0):
            req = Trigger.Request()
            self.dock_cli.call_async(req)

    def get_system_temperature(self):
        try:
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                temp_raw = int(f.read().strip())
            return round(temp_raw / 1000.0, 1)
        except Exception:
            return 0.0

    def get_cpu_load(self):
        try:
            with open("/proc/stat", "r") as f:
                line = f.readline()
            parts = line.split()
            # CPU user, nice, system, idle, iowait, irq, softirq
            cpu_times = [int(x) for x in parts[1:5]]
            idle = cpu_times[3]
            total = sum(cpu_times)

            diff_idle = idle - self.last_cpu_idle
            diff_total = total - self.last_cpu_total

            self.last_cpu_idle = idle
            self.last_cpu_total = total

            if diff_total == 0:
                return 0.0
            return round((1.0 - (diff_idle / diff_total)) * 100.0, 1)
        except Exception:
            return 0.0

    def get_wifi_rssi_dbm(self):
        # Added: robust parsing of Linux Wi-Fi RSSI signal strength in dBm and %
        try:
            if not os.path.exists("/proc/net/wireless"):
                return {"dbm": -99, "percentage": 0, "interface": "none"}
            with open("/proc/net/wireless", "r") as f:
                lines = f.readlines()
            for line in lines:
                if "wlan" in line or "wl" in line:
                    parts = line.split()
                    # Typical format: wlan0: 0000   85.  -65.  -256
                    # Link quality is parts[2], signal level (RSSI) is parts[3]
                    rssi_str = parts[3].replace(".", "")
                    rssi = int(rssi_str)

                    # Convert to percentage (0% to 100%)
                    if rssi <= -100:
                        pct = 0
                    elif rssi >= -50:
                        pct = 100
                    else:
                        pct = int(2 * (rssi + 100))

                    # Extract interface name (e.g. wlan0)
                    interface = parts[0].replace(":", "")
                    return {"dbm": rssi, "percentage": pct, "interface": interface}
            return {"dbm": -99, "percentage": 0, "interface": "unknown"}
        except Exception as e:
            return {"dbm": -99, "percentage": 0, "error": str(e)}

    def publish_heartbeat(self):
        # Added: send heartbeat ping to confirm the Radxa is running for the ESP32
        try:
            msg = Empty()
            self.heartbeat_pub.publish(msg)
        except Exception:
            pass

    def get_disk_space_pct(self):
        # SSD Guard - monitors free NVMe space on the Radxa Dragon Q6A
        try:
            total, used, free = shutil.disk_usage("/")
            free_pct = (free / total) * 100.0
            return round(free_pct, 1)
        except Exception:
            return 100.0

    def broadcast_status(self):
        payload = {
            "gps": self.gps_data,
            "battery": {
                "voltage": round(self.battery_v, 2),
                "percentage": round(self.battery_pct, 1),
                "current": round(self.battery_current, 2),
                "temperature_celsius": round(self.battery_temp, 1),
                "charge_cycles": self.charge_cycles #
            },
            "state": self.state,
            "cutter_status": self.cutter_status,
            "cutter_rpm": self.cutter_rpm,
            "power_consumption": {
                "total_bms_current_ampere": round(self.battery_current, 2),
                "drive_motors_current_ampere": round(self.drive_current, 2),
                "cutter_motor_current_ampere": round(self.cutter_current, 2),
                "cutter_motor_power_watts": round(self.battery_v * self.cutter_current, 1) #
            },
            "statistics": { #
                "total_distance_km": round(self.total_distance_km, 2),
                "total_runtime_hours": round(self.total_runtime_hours, 1)
            },
            "system": {
                "cpu_temp_celsius": self.get_system_temperature(),
                "cpu_load_pct": self.get_cpu_load(),
                "wifi": self.get_wifi_rssi_dbm(), # Added: send Wi-Fi RSSI to the app
                "disk_free_pct": self.get_disk_space_pct(), # SSD guard monitoring
                "rain_detected": self.is_raining, # Added: rain sensor status for the app
                "rain_delay": { # Live status of the drying timer
                    "is_delayed": time.time() < self.rain_resume_time,
                    "remaining_seconds": max(0, int(self.rain_resume_time - time.time())),
                    "resume_timestamp": self.rain_resume_time
                },
                "schedule": self.schedule # Added: send the schedule configuration to the app
            }
        }

        for client in clients:
            try:
                asyncio.run_coroutine_threadsafe(client.send_text(json.dumps(payload)), loop)
            except Exception:
                clients.remove(client)

def start_ros():
    global ros_node
    rclpy.init()
    ros_node = BackendROSNode()
    app.state.ros_node = ros_node # Store node reference in FastAPI app state for REST endpoints
    rclpy.spin(ros_node)
    rclpy.shutdown()

@app.get("/api/schedule")
async def get_schedule():
    # Added: REST endpoint to fetch the current mowing schedule and rain delay
    if hasattr(app.state, "ros_node"):
        return {
            "schedule": app.state.ros_node.schedule,
            "rain_delay_duration": app.state.ros_node.rain_delay_duration,
            "rain_resume_time": app.state.ros_node.rain_resume_time,
            "remaining_drying_seconds": max(0, int(app.state.ros_node.rain_resume_time - time.time()))
        }
    return {"error": "ROS 2 node not started yet"}

@app.post("/api/schedule")
async def post_schedule(new_schedule: dict):
    # Added: REST endpoint to save a new schedule and update stats.json
    if hasattr(app.state, "ros_node"):
        app.state.ros_node.schedule = new_schedule
        app.state.ros_node.save_stats()
        app.state.ros_node.broadcast_status()
        return {"status": "success", "message": "Schedule updated persistently!"}
    return {"error": "ROS 2 node not started yet"}

@app.post("/api/schedule/rain_delay")
async def post_rain_delay(payload: dict):
    # Added: REST endpoint to update the drying time
    if hasattr(app.state, "ros_node"):
        app.state.ros_node.rain_delay_duration = int(payload.get("duration_seconds", 7200))
        app.state.ros_node.save_stats()
        app.state.ros_node.broadcast_status()
        return {"status": "success", "message": "Drying duration updated!"}
    return {"error": "ROS 2 node not started yet"}

@app.post("/api/schedule/skip_delay")
async def skip_delay():
    # Added: REST endpoint to skip the drying timer and force operation
    if hasattr(app.state, "ros_node"):
        app.state.ros_node.rain_resume_time = 0.0
        if app.state.ros_node.state == 9:
            app.state.ros_node.state = 0
        app.state.ros_node.broadcast_status()
        return {"status": "success", "message": "Rain delay skipped. The mower is ready!"}
    return {"error": "ROS 2 node not started yet"}

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    clients.append(websocket)
    try:
        while True:
            await websocket.receive_text()
    except Exception:
        clients.remove(websocket)

if __name__ == "__main__":
    global loop
    loop = asyncio.get_event_loop()
    ros_thread = threading.Thread(target=start_ros, daemon=True)
    ros_thread.start()
    uvicorn.run(app, host="0.0.0.0", port=8000)