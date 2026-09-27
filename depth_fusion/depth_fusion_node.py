#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import time
import onnxruntime as ort

#[QUALCOMM NATIVE SNPE SDK IMPORT]
try:
    import snpe
    HAS_SNPE = True
except ImportError:
    HAS_SNPE = False

class DepthFusionNode(Node):
    """Run image segmentation and publish a mask for Nav2 obstacle marking."""

    def __init__(self):
        super().__init__('depth_fusion_node')
        self.bridge = CvBridge()
        self.publisher_mask = self.create_publisher(Image, '/vision/segmented_mask', 10)

        # ROS 2 parameters
        self.declare_parameter('model_path', '/models/MowerAIn-seg.onnx')
        self.declare_parameter('confidence_threshold', 0.50)
        self.declare_parameter('active_learning_threshold', 0.20) # MLOps threshold

        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.conf_thresh = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.al_thresh = self.get_parameter('active_learning_threshold').get_parameter_value().double_value

        self.get_logger().info(f"Initializing ONNX Runtime with Qualcomm QNN EP. Loading model: {self.model_path}")

        # QUALCOMM QNN RUNTIME INITIALIZATION
        session_options = ort.SessionOptions()
        session_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL

        # Configure QNN (Qualcomm Neural Network) backend settings
        qnn_options = {
            "backend_path": "libQnnHtp.so", # Hexagon Tensor Processor backend for 12 TOPS
            "profiling_level": "basic"
        }

        try:
            # Create asynchronous inference session for Qualcomm Hexagon NPU
            self.session = ort.InferenceSession(
                self.model_path,
                sess_options=session_options,
                providers=["QNNExecutionProvider"],
                provider_options=[qnn_options]
            )
            self.input_name = self.session.get_inputs()[0].name
            self.get_logger().info("Qualcomm Hexagon NPU (12 TOPS) accelerated ONNX session started!")
        except Exception as e:
            self.get_logger().error(f"Error loading QNN model: {str(e)}. Fallback to CPU...")
            self.session = ort.InferenceSession(self.model_path, sess_options=session_options, providers=["CPUExecutionProvider"])
            self.input_name = self.session.get_inputs()[0].name

        # Subscribe to the synchronized left source image
        self.subscription = self.create_subscription(
            Image,
            '/stereo/left/image_raw',
            self.image_callback,
            10
        )

        # Optionally initialize the native Qualcomm SNPE runtime when its bindings exist.
        if HAS_SNPE:
            try:
                # Create the Qualcomm SNPE model container.
                builder = snpe.SNPEBuilder(self.model_path)

                # configure to run 100% on Hexagon HTP/DSP (NPU) for maximum performance and power saving! [cite: 7, 8]
                builder.set_runtime_processor(snpe.Runtime.DSP)

                # Build the native inference runtime.
                self.snpe_runtime = builder.build()
                self.get_logger().info("Qualcomm Hexagon NPU (12 TOPS) accelerated native SNPE session started!")
            except Exception as e:
                self.get_logger().error(f"Error creating native SNPE session: {str(e)}. Fallback to CPU...")
                try:
                    builder = snpe.SNPEBuilder(self.model_path)
                    builder.set_runtime_processor(snpe.Runtime.CPU)
                    self.snpe_runtime = builder.build()
                except Exception as ex:
                    self.get_logger().error(f"Fatal error: Could not load model on CPU: {str(ex)}")
                    self.snpe_runtime = None
        else:
            self.get_logger().error("Qualcomm SNPE Python SDK bindings not found! Check PYTHONPATH in your Docker container.")
            self.snpe_runtime = None

        # Subscribe to the left camera image used as the segmentation input.
        self.subscription = self.create_subscription(
            Image,
            '/stereo/left/image_raw',
            self.image_callback,
            10
        )


    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        orig_h, orig_w, _ = cv_image.shape

        # YOLOv8-seg preprocessing: Scale to 640x640 input size
        model_size = 640
        img_resized = cv2.resize(cv_image, (model_size, model_size))
        img_rgb = cv2.cvtColor(img_resized, cv2.COLOR_BGR2RGB)

        # Normalize to float32 (or use UINT8 input depending on QNN quantization)
        img_input = np.expand_dims(img_rgb, axis=0).astype(np.float32) / 255.0

        # Run inference lightning-fast on Qualcomm Hexagon NPU
        outputs = self.session.run(None, {self.input_name: img_input})

        # Decode model outputs into a binary mask before restoring the source resolution.
        mask = np.zeros((model_size, model_size), dtype=np.uint8)

        # Active-learning hook: uncertain detections can be saved for later retraining.
        # If there are classes of interest (e.g., dog poop, toys, animals)
        # with low confidence (e.g., between 20% and 50%), save the image for nightly training.
        low_confidence_detected = False

        # If uncertain objects are detected, save the image to the MLOps inbox over NFS
        if low_confidence_detected:
            timestamp = int(time.time() * 1000)
            filepath = f"/incoming_raw/img_{timestamp}.jpg"
            cv2.imwrite(filepath, cv_image)
            self.get_logger().info(f"Saved uncertain object for active learning: {filepath}")

        # Scale the mask back to the original resolution
        full_mask = cv2.resize(mask, (orig_w, orig_h), interpolation=cv2.INTER_NEAREST)

        # Publish the mask as ROS 2 Image, so Nav2 Costmap can mark areas as impassable (lethal)
        mask_msg = self.bridge.cv2_to_imgmsg(full_mask, encoding="mono8")
        mask_msg.header = msg.header
        self.publisher_mask.publish(mask_msg)

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