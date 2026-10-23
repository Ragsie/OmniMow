#!/usr/bin/env python3
"""
Camera capture for the dual IMX219 on the Radxa Dragon Q6A (QCS6490).

The CSI cameras deliver NV12 through Qualcomm's camera pipeline and are opened with a GStreamer
pipeline. By default via libcamera with the cameras' fixed IDs (device-tree paths, see `cam -l`),
which - unlike /dev/videoN - never swap between boots. v4l2src on /dev/video2 and /dev/video3 is the
alternative. The frame is scaled down while it is still NV12 and only then converted to BGR,
which keeps the CPU load low.

Each camera is read in its own thread that always keeps the newest frame; StereoCapture
pairs the newest left and right frames whose timestamps are close together.

Settings (omnimow.env):
  OMNIMOW_CAMERA_BACKEND             libcamera (default), gstreamer (v4l2src on /dev/videoN) or
                                     v4l2 (plain OpenCV capture, e.g. USB cameras); host_setup.sh
                                     replaces auto with what it found (/etc/omnimow/hardware.env)
  OMNIMOW_CAMERA_ID_LEFT/RIGHT       libcamera IDs, e.g. /base/soc@0/cci@ac4b000/i2c-bus@1/camera@10
  OMNIMOW_CAMERA_DEVICE_LEFT/RIGHT   /dev/video2, /dev/video3 or /dev/v4l/by-path/... (gstreamer /
                                     v4l2 backends)
  OMNIMOW_CAMERA_SENSOR_WIDTH/HEIGHT format requested from the camera (1920 x 1080), not used with
                                     OMNIMOW_CAMERA_GPU_SCALE
  OMNIMOW_CAMERA_GPU_SCALE           true: libcamera's GPU debayer (LIBCAMERA_SOFTISP_MODE=gpu) delivers
                                     the final size itself, scaled from the whole sensor frame. libcamera
                                     then picks the IMX219's 1640x1232 binned mode (full field of view),
                                     so the size must be 4:3 (640 x 480). No 1080p conversion on the CPU.
  OMNIMOW_CAMERA_WIDTH/HEIGHT        size delivered to the software (640 x 360, same 16:9 aspect)
  OMNIMOW_CAMERA_FPS                 frame rate of the cameras, e.g. 30; empty = the camera's own.
                                     libcamera's simple pipeline cannot set it, so the IMX219s' vertical
                                     blanking is set directly once they stream (set_sensor_fps). Fewer
                                     frames = less work for the ISP, but left/right pairs up to half a
                                     frame interval apart
  OMNIMOW_CAMERA_UPSIDE_DOWN         true: the camera pair is mounted upside down. Both images are turned
                                     180 degrees, and left/right swap (the IDs/devices above name the
                                     cameras as mounted upright)
  OMNIMOW_AI_IMAGE_WIDTH             the left camera delivers this width instead (800 = the AI model's
                                     input width), the stereo node scales it down for the stereo matching
"""
import glob
import os
import re
import subprocess
import threading
import time

import cv2


# Dual IMX219 on the Radxa Dragon Q6A (CSI ports on CCI I2C bus 1 and 0)
DEFAULT_CAMERA_ID_LEFT = "/base/soc@0/cci@ac4b000/i2c-bus@1/camera@10"
DEFAULT_CAMERA_ID_RIGHT = "/base/soc@0/cci@ac4b000/i2c-bus@0/camera@10"


def env_int(name, default):
    return int(os.getenv(name, str(default)))


def upside_down():
    return os.getenv('OMNIMOW_CAMERA_UPSIDE_DOWN', 'false').lower() == 'true'


def backend():
    value = os.getenv('OMNIMOW_CAMERA_BACKEND', 'libcamera').lower()
    return 'libcamera' if value in ('', 'auto') else value


def default_cameras():
    """(left, right) camera for the configured backend: libcamera IDs or /dev/video nodes."""
    if backend() == 'libcamera':
        return (os.getenv('OMNIMOW_CAMERA_ID_LEFT', DEFAULT_CAMERA_ID_LEFT),
                os.getenv('OMNIMOW_CAMERA_ID_RIGHT', DEFAULT_CAMERA_ID_RIGHT))
    # /dev/v4l/by-path links (USB port) point to the /dev/videoN the camera has on this boot
    return (os.path.realpath(os.getenv('OMNIMOW_CAMERA_DEVICE_LEFT') or '/dev/video2'),
            os.path.realpath(os.getenv('OMNIMOW_CAMERA_DEVICE_RIGHT') or '/dev/video3'))


def gpu_scale():
    """libcamera's GPU debayer scales to the requested size (OMNIMOW_CAMERA_GPU_SCALE)."""
    return (backend() == 'libcamera' and os.getenv('LIBCAMERA_SOFTISP_MODE', '').lower() == 'gpu'
            and os.getenv('OMNIMOW_CAMERA_GPU_SCALE', 'false').lower() == 'true')


def gstreamer_pipeline(device, width, height, sensor_width, sensor_height, fmt="NV12", source="v4l2", fps=0,
                       scaled_by_camera=False):
    """device: /dev/videoN for v4l2src, or a libcamera camera ID for libcamerasrc."""
    src = f'libcamerasrc camera-name="{device}"' if source == "libcamera" else f"v4l2src device={device}"
    # An empty format lets the camera pick one (mainline kernels' software ISP has no NV12)
    caps = f"format={fmt}," if fmt else ""
    # libcamerasrc turns the frame rate into the camera's frame duration, so the frames are never made
    rate = f",framerate={fps}/1" if fps else ""
    if scaled_by_camera:
        # The GPU debayer renders the whole sensor frame at width x height (RGBA); only the
        # small RGBA -> BGR conversion is left for the CPU
        return (
            f"{src} ! video/x-raw,{caps}width={width},height={height}{rate} ! "
            f"videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false"
        )
    return (
        f"{src} ! "
        f"video/x-raw,{caps}width={sensor_width},height={sensor_height}{rate} ! "
        # Scale and convert in one pass on several threads (videoconvertscale, GStreamer >= 1.22):
        # with the software ISP every frame arrives as 1920x1080 RGBA in memory
        f"videoconvertscale n-threads=2 ! video/x-raw,format=BGR,width={width},height={height} ! "
        f"appsink drop=true max-buffers=1 sync=false"
    )


def open_capture(device, width=None, height=None):
    """Open one camera (libcamera ID or /dev/videoN) with the configured backend at width x height."""
    width = width or env_int('OMNIMOW_CAMERA_WIDTH', 640)
    height = height or env_int('OMNIMOW_CAMERA_HEIGHT', 360)
    if backend() == 'v4l2':
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        return cap
    fps = int(os.getenv('OMNIMOW_CAMERA_FPS', '') or 0) if backend() == 'libcamera' else 0
    args = (device, width, height,
            env_int('OMNIMOW_CAMERA_SENSOR_WIDTH', 1920), env_int('OMNIMOW_CAMERA_SENSOR_HEIGHT', 1080),
            os.getenv('OMNIMOW_CAMERA_FORMAT', 'NV12'),
            'libcamera' if backend() == 'libcamera' else 'v4l2')
    scaled = gpu_scale()
    cap = cv2.VideoCapture(gstreamer_pipeline(*args, fps=fps, scaled_by_camera=scaled), cv2.CAP_GSTREAMER)
    if fps and not cap.isOpened():
        print(f"Camera {device} does not accept {fps} FPS - opening it at its own frame rate", flush=True)
        cap = cv2.VideoCapture(gstreamer_pipeline(*args, scaled_by_camera=scaled), cv2.CAP_GSTREAMER)
    return cap


def _v4l2_ctl(device, *args):
    return subprocess.run(['v4l2-ctl', '-d', device, *args], capture_output=True, text=True,
                          timeout=5).stdout


def set_sensor_fps(fps, sensor='imx219'):
    """
    Set the frame rate of all streaming `sensor` subdevices through their vertical blanking:
    fps = pixel_rate / ((width + hblank) * (height + vblank)). The driver resets the blanking
    when libcamera configures the format, so this runs once the cameras stream. Returns
    [(subdevice, fps reached)].
    """
    result = []
    for name_file in sorted(glob.glob('/sys/class/video4linux/v4l-subdev*/name')):
        with open(name_file) as f:
            if sensor not in f.read():
                continue
        dev = '/dev/' + name_file.split('/')[-2]
        try:
            ctrls = _v4l2_ctl(dev, '--list-ctrls')
            fmt = _v4l2_ctl(dev, '--get-subdev-fmt')
            value = lambda c: int(re.search(rf'{c} .*?value=(-?\d+)', ctrls).group(1))
            vmin, vmax = map(int, re.search(r'vertical_blanking .*?min=(\d+) max=(\d+)', ctrls).groups())
            width, height = map(int, re.search(r'Width/Height\s*:\s*(\d+)/(\d+)', fmt).groups())
            line = width + value('horizontal_blanking')
            pixel_rate = value('pixel_rate')
            vblank = min(vmax, max(vmin, round(pixel_rate / (fps * line)) - height))
            _v4l2_ctl(dev, '-c', f'vertical_blanking={vblank}')
            result.append((dev, pixel_rate / (line * (height + vblank))))
        except (OSError, subprocess.SubprocessError, AttributeError, ValueError) as e:
            print(f"Could not set the frame rate of {dev}: {e}", flush=True)
    return result


class CameraThread:
    """Reads one camera continuously in the background and keeps only the newest frame."""

    def __init__(self, device, width=None, height=None):
        self.device = device
        self.rotate = upside_down()
        self.cap = open_capture(device, width, height)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open camera {device} (backend {backend()}; "
                               f"list the camera IDs with 'cam -l', or set OMNIMOW_CAMERA_BACKEND=gstreamer "
                               f"to use /dev/video2 and /dev/video3)")
        self.frame = None
        self.stamp = 0.0       # time.monotonic() when the frame arrived
        self.count = 0
        self.interval = 0.0    # Average time between frames (s), 0 until known
        self.lock = threading.Lock()
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        while self.running:
            ok, frame = self.cap.read()
            if not ok:
                time.sleep(0.01)
                continue
            if self.rotate:
                frame = cv2.rotate(frame, cv2.ROTATE_180)
            now = time.monotonic()
            with self.lock:
                if self.count:
                    dt = now - self.stamp
                    self.interval = dt if not self.interval else 0.9 * self.interval + 0.1 * dt
                self.frame, self.stamp = frame, now
                self.count += 1

    def latest(self):
        """(frame, timestamp, frame number) of the newest frame; frame is None until the first one."""
        with self.lock:
            return self.frame, self.stamp, self.count

    def release(self):
        self.running = False
        self.thread.join(timeout=2.0)
        self.cap.release()


class StereoCapture:
    """Left + right camera, returning pairs of new frames taken close together in time."""

    def __init__(self, device_left=None, device_right=None, width=None, height=None, left_size=None):
        """left_size: (width, height) of the left camera if it differs from the right (AI image)."""
        default_left, default_right = default_cameras()
        device_left = device_left or default_left
        device_right = device_right or default_right
        if upside_down():
            # Turned over, the camera mounted on the left is on the right
            device_left, device_right = device_right, device_left
        self.left = CameraThread(device_left, *(left_size or (width, height)))
        time.sleep(1.0)  # Give the ISP time to allocate the first stream before starting the second
        try:
            self.right = CameraThread(device_right, width, height)
        except Exception:
            self.left.release()
            raise
        self.last = (0, 0)

        fps = int(os.getenv('OMNIMOW_CAMERA_FPS', '') or 0)
        if fps and backend() == 'libcamera':
            # Once both stream (the driver resets the blanking when the format is set)
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline and not (self.left.count and self.right.count):
                time.sleep(0.05)
            for dev, reached in set_sensor_fps(fps):
                print(f"Camera {dev}: {reached:.1f} FPS", flush=True)

    def fps(self):
        """Measured frame rates (left, right)."""
        return tuple(1.0 / c.interval if c.interval else 0.0 for c in (self.left, self.right))

    def auto_skew(self):
        """
        Largest time difference that still pairs a frame of each camera every frame. The cameras
        free-run, so their frames are offset by a fixed phase of up to one frame interval: there
        is always a pair within half an interval, but not necessarily within a fixed 20 ms.
        """
        interval = max(self.left.interval, self.right.interval)
        return min(0.05, max(0.01, 0.55 * interval)) if interval else 0.02

    def read(self, timeout=2.0, max_skew=None):
        """
        Wait for a new left/right pair whose timestamps differ less than max_skew seconds
        (default: auto_skew()). Returns (left, right) or (None, None) on timeout.
        """
        if max_skew is None:
            max_skew = self.auto_skew()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            fl, tl, nl = self.left.latest()
            fr, tr, nr = self.right.latest()
            if fl is not None and fr is not None and (nl, nr) != self.last and abs(tl - tr) <= max_skew:
                self.last = (nl, nr)
                return fl.copy(), fr.copy()
            time.sleep(0.002)
        return None, None

    def release(self):
        self.left.release()
        self.right.release()
