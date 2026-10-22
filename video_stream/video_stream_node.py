#!/usr/bin/env python3
"""
OmniMow live video for the app: the rectified left camera image with the AI detections drawn
on top, sent to the app over WebRTC.

The app opens a WebSocket on ws://<mower-ip>:8889/webrtc and exchanges JSON messages:
  app -> mower   {"type": "offer", "sdp": ...}
  mower -> app   {"type": "answer", "sdp": ...}   (contains the mower's ICE candidates)
  app -> mower   {"candidate": ..., "sdpMid": ..., "sdpMLineIndex": ...}
The offer must ask to receive video (a recvonly video transceiver), otherwise there is no
video section to answer with.

Subscribes
  /stereo/left/image_rect_color/compressed   JPEG from the stereo node (OMNIMOW_AI_IMAGE_WIDTH wide)
  /vision/detections                         JSON from depth_fusion: class, score, box in that image
The JPEGs are only decoded and encoded while the app watches.
"""
import asyncio
import json
import logging
import os
import threading
import time
from fractions import Fraction

import cv2
import numpy as np
import rclpy
from aiohttp import WSMsgType, web
from aiortc import MediaStreamTrack, RTCConfiguration, RTCPeerConnection, RTCSessionDescription
from aiortc.sdp import candidate_from_sdp
from av import VideoFrame
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

PORT = int(os.getenv('OMNIMOW_STREAM_PORT', '8889') or 8889)
OVERLAY = os.getenv('OMNIMOW_STREAM_OVERLAY', 'true').lower() == 'true'
VIDEO_CLOCK = 90000          # RTP clock rate for video
FRAME_TIMEOUT = 1.0          # Without a camera image for this long the app gets a "no image" frame
DETECTIONS_MAX_AGE = 1.0     # Older detections are not drawn (depth_fusion stopped or lags behind)
HAZARD_CLASSES = {'dog'}     # Drawn in red, like the hazard pause in mowing_control
OBSTACLE_CLASSES = {'poo', 'toy', 'tree', 'wall'}

log = logging.getLogger('video_stream')


class FrameSource:
    """Newest camera JPEG and detections from ROS, handed to the asyncio side."""

    def __init__(self, loop):
        self.loop = loop
        self.jpeg = None
        self.seq = 0
        self.size = (640, 480)
        self.detections = []
        self.detections_time = 0.0
        self.new_frame = asyncio.Condition()

    def put_jpeg(self, data):
        """Called from the ROS thread."""
        self.loop.call_soon_threadsafe(self._store, data)

    def _store(self, data):
        self.jpeg = data
        self.seq += 1
        self.loop.create_task(self._notify())

    async def _notify(self):
        async with self.new_frame:
            self.new_frame.notify_all()

    def put_detections(self, detections):
        self.detections, self.detections_time = detections, time.monotonic()

    def current_detections(self):
        if time.monotonic() - self.detections_time > DETECTIONS_MAX_AGE:
            return []
        return self.detections


def draw_detections(image, detections):
    for d in detections:
        try:
            x1, y1, x2, y2 = (int(round(v)) for v in d['box'])
            name, score = str(d['class']), float(d['score'])
        except (KeyError, TypeError, ValueError):
            continue
        if name in HAZARD_CLASSES:
            color = (0, 0, 255)
        elif name in OBSTACLE_CLASSES:
            color = (0, 165, 255)
        else:
            color = (0, 200, 0)
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
        label = f"{name} {score:.2f}"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        ty = max(y1, th + 4)
        cv2.rectangle(image, (x1, ty - th - 4), (x1 + tw + 4, ty), color, -1)
        cv2.putText(image, label, (x1 + 2, ty - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)


def placeholder(size, text):
    image = np.zeros((size[1], size[0], 3), dtype=np.uint8)
    cv2.putText(image, text, (20, size[1] // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
    return image


class CameraTrack(MediaStreamTrack):
    """Video track for one app connection: sends every new camera frame once."""

    kind = 'video'

    def __init__(self, source):
        super().__init__()
        self.source = source
        self.seq = -1
        self.start = None

    async def recv(self):
        source = self.source
        try:
            async with source.new_frame:
                await asyncio.wait_for(
                    source.new_frame.wait_for(lambda: source.seq != self.seq and source.jpeg is not None),
                    FRAME_TIMEOUT)
        except asyncio.TimeoutError:
            pass

        image = None
        if source.seq != self.seq and source.jpeg is not None:
            self.seq = source.seq
            image = cv2.imdecode(np.frombuffer(source.jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            image = placeholder(source.size, "No camera image")
        else:
            source.size = (image.shape[1], image.shape[0])
            if OVERLAY:
                draw_detections(image, source.current_detections())

        now = time.monotonic()
        if self.start is None:
            self.start = now
        frame = VideoFrame.from_ndarray(image, format='bgr24')
        frame.pts = int((now - self.start) * VIDEO_CLOCK)
        frame.time_base = Fraction(1, VIDEO_CLOCK)
        return frame


class VideoStreamNode(Node):
    """Feeds the camera JPEGs and AI detections into the FrameSource."""

    def __init__(self, source):
        super().__init__('video_stream_node')
        self.source = source
        self.create_subscription(CompressedImage, '/stereo/left/image_rect_color/compressed',
                                 self.image_callback, qos_profile_sensor_data)
        self.create_subscription(String, '/vision/detections', self.detections_callback, 10)

    def image_callback(self, msg):
        self.source.put_jpeg(bytes(msg.data))

    def detections_callback(self, msg):
        try:
            detections = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(detections, list):
            self.source.put_detections(detections)


def parse_candidate(data):
    """ICE candidate from the app's JSON, or None for the end-of-candidates marker."""
    text = data.get('candidate') or ''
    if text.startswith('candidate:'):
        text = text[len('candidate:'):]
    if not text:
        return None
    candidate = candidate_from_sdp(text)
    candidate.sdpMid = data.get('sdpMid')
    candidate.sdpMLineIndex = data.get('sdpMLineIndex')
    return candidate


async def handle_offer(ws, pc, source, data, pending_candidates, peer):
    """Answer the app's offer with the camera track. False if the offer cannot carry video."""
    if 'm=video' not in data.get('sdp', ''):
        log.warning("Offer from %s has no video section - the app must add a recvonly "
                    "video transceiver before createOffer()", peer)
        await ws.send_json({'type': 'error', 'message': 'offer has no video section'})
        return False
    pc.addTrack(CameraTrack(source))
    await pc.setRemoteDescription(RTCSessionDescription(sdp=data['sdp'], type='offer'))
    for candidate in pending_candidates:
        await pc.addIceCandidate(candidate)
    pending_candidates.clear()
    # aiortc gathers all of its candidates in setLocalDescription: they are in the answer
    await pc.setLocalDescription(await pc.createAnswer())
    await ws.send_json({'type': pc.localDescription.type, 'sdp': pc.localDescription.sdp})
    return True


async def handle_candidate(pc, data, pending_candidates):
    try:
        candidate = parse_candidate(data)
    except ValueError:
        return
    if candidate is None:
        return
    if pc.remoteDescription is None:
        pending_candidates.append(candidate)  # Arrived before the offer was handled
    else:
        await pc.addIceCandidate(candidate)


async def webrtc_handler(request):
    """One app connection: WebSocket signaling and its peer connection."""
    source = request.app['source']
    ws = web.WebSocketResponse(heartbeat=20)
    await ws.prepare(request)
    peer = request.remote
    # No STUN server: the app connects over the local network (or a VPN), where the mower's
    # host candidates are reachable, and the answer is not held up by an unreachable STUN server
    pc = RTCPeerConnection(RTCConfiguration(iceServers=[]))
    request.app['pcs'].add(pc)
    pending_candidates = []
    log.info("App connected from %s (%d viewers)", peer, len(request.app['pcs']))

    @pc.on('connectionstatechange')
    async def on_state():
        log.info("WebRTC connection to %s: %s", peer, pc.connectionState)
        if pc.connectionState in ('failed', 'closed'):
            await ws.close()

    try:
        async for msg in ws:
            if msg.type != WSMsgType.TEXT:
                continue
            try:
                data = json.loads(msg.data)
            except ValueError:
                continue
            if not isinstance(data, dict):
                continue

            if data.get('type') == 'offer':
                if not await handle_offer(ws, pc, source, data, pending_candidates, peer):
                    break
            elif 'candidate' in data:
                await handle_candidate(pc, data, pending_candidates)
    except Exception as e:
        log.error("WebRTC session with %s failed: %s", peer, e)
    finally:
        # The app closes the WebSocket when the live view is left: stop encoding right away
        request.app['pcs'].discard(pc)
        await pc.close()
        log.info("App %s disconnected (%d viewers)", peer, len(request.app['pcs']))
    return ws


async def on_shutdown(app):
    await asyncio.gather(*(pc.close() for pc in list(app['pcs'])))
    app['pcs'].clear()


def spin_ros(node):
    try:
        rclpy.spin(node)
    except Exception:
        pass


async def main():
    logging.basicConfig(level=logging.INFO, format='[video_stream] %(levelname)s %(message)s')
    rclpy.init()
    source = FrameSource(asyncio.get_running_loop())
    node = VideoStreamNode(source)
    threading.Thread(target=spin_ros, args=(node,), daemon=True).start()

    app = web.Application()
    app['source'] = source
    app['pcs'] = set()
    app.router.add_get('/webrtc', webrtc_handler)
    app.on_shutdown.append(on_shutdown)
    runner = web.AppRunner(app)
    await runner.setup()
    await web.TCPSite(runner, '0.0.0.0', PORT).start()
    log.info("WebRTC live video on ws://0.0.0.0:%d/webrtc (detections overlay %s)", PORT, 'on' if OVERLAY else 'off')
    try:
        await asyncio.Event().wait()
    finally:
        await runner.cleanup()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
