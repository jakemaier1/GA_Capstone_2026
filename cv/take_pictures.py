from pathlib import Path
from datetime import datetime
import select
import sys
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


class PictureSaver(Node):
    def __init__(self):
        super().__init__("picture_saver")

        self.bridge = CvBridge()
        self.latest_image = None
        self.last_received = None

        # Save directly into the folder shared with the Jetson host.
        session = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        self.folder = Path("/root/share/camera_pictures") / session
        self.folder.mkdir(parents=True, exist_ok=True)

        # Keep only the newest pending camera frame.
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.subscription = self.create_subscription(
            Image,
            "/ascamera_hp60c/camera_publisher/rgb0/image",
            self.image_callback,
            qos,
        )

    def image_callback(self, message):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(
                message, desired_encoding="bgr8"
            )
            self.last_received = time.monotonic()
        except Exception as error:
            self.get_logger().error(f"Image conversion failed: {error}")

    def save_picture(self):
        if self.latest_image is None:
            print("No image received. Check the camera driver.")
            return

        # Refuse to save if frames have stopped arriving.
        if time.monotonic() - self.last_received > 1.0:
            print("Camera frames stopped arriving. Picture not saved.")
            return

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        filename = self.folder / f"picture_{timestamp}.jpg"

        try:
            saved = cv2.imwrite(
                str(filename),
                self.latest_image,
                [cv2.IMWRITE_JPEG_QUALITY, 95],
            )

            if saved:
                print(f"Saved: {filename}")
            else:
                print(f"Could not save: {filename}")
        except cv2.error as error:
            print(f"Could not save picture: {error}")


def main():
    rclpy.init()
    node = PictureSaver()

    print(f"Saving pictures in: {node.folder}")
    print("Press Enter to save. Type q and Enter to quit.")

    try:
        while rclpy.ok():
            # Receive camera frames while checking for keyboard input.
            rclpy.spin_once(node, timeout_sec=0.05)

            if select.select([sys.stdin], [], [], 0)[0]:
                command = sys.stdin.readline()

                if not command or command.strip().lower() == "q":
                    break

                node.save_picture()

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
