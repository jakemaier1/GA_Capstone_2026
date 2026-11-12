#!/usr/bin/env python3

import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
)
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO


MODEL_PATH = (
    "/root/share/airport.onnx"
)

CAMERA_TOPIC = "/ascamera_hp60c/camera_publisher/rgb0/image"

# Increase to reject more weak detections.
# Decrease if real signs are being missed.
CONFIDENCE = 0.60


class SignDetection(Node):
    def __init__(self):
        super().__init__("sign_detection")

        self.model = YOLO(MODEL_PATH, task="detect")
        self.bridge = CvBridge()
        self.prev_time = None
        self.printed_classes = False
        self.quit_requested = False

        # Keep only the newest pending image to reduce delay.
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.subscription = self.create_subscription(
            Image,
            CAMERA_TOPIC,
            self.image_callback,
            qos,
        )

        print(f"Model: {MODEL_PATH}")
        print(f"Camera topic: {CAMERA_TOPIC}")
        print(f"Confidence threshold: {CONFIDENCE}")
        print("Waiting for camera images.")
        print("Press q in the preview window, or Ctrl+C here, to quit.")

    def image_callback(self, message):
        try:
            frame = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding="bgr8",
            )
        except Exception as error:
            self.get_logger().error(
                f"Image conversion failed: {error}"
            )
            return

        self.process_frame(frame)

    def process_frame(self, frame):
        start = time.perf_counter()

        # Use the ONNX model's existing input size.
        results = self.model.predict(
            source=frame,
            conf=CONFIDENCE,
            iou=0.45,
            verbose=False,
        )

        prediction_ms = (time.perf_counter() - start) * 1000

        # Confirm which classes the loaded model contains.
        if not self.printed_classes:
            print("Loaded classes:", results[0].names)
            self.printed_classes = True

        annotated_frame = results[0].plot()

        # Approximate processed/displayed FPS, not camera FPS.
        now = time.perf_counter()

        if self.prev_time is None:
            fps = 0.0
        else:
            fps = 1.0 / max(now - self.prev_time, 0.000001)

        self.prev_time = now

        text = (
            f"FPS: {fps:.1f} | "
            f"Prediction: {prediction_ms:.0f} ms"
        )

        cv2.putText(
            annotated_frame,
            text,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
        )

        # No video recording, to avoid encoding and disk overhead.
        cv2.imshow(
            "Airport Sign Detection",
            cv2.resize(annotated_frame, (640, 480)),
        )

        if cv2.waitKey(1) & 0xFF == ord("q"):
            self.quit_requested = True

    def close(self):
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = SignDetection()

        while rclpy.ok() and not node.quit_requested:
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.close()
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
