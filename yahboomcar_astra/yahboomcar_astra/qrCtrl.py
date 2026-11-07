import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, Bool,UInt16
from yahboomcar_msgs.msg import ServoControl
from geometry_msgs.msg import Twist

import cv2
import time
import numpy as np

import pyzbar.pyzbar as pyzbar
from sensor_msgs.msg import Image
import os
from cv_bridge import CvBridge

class QR_Tracker(Node):
    def __init__(self,name):
        super().__init__(name)
        self.pubCmdVel = self.create_publisher(Twist, 'cmd_vel', 1)
        self.servo_angle = ServoControl()
        self.servos1 = int(os.environ.get("INIT_SERVO_S1"))
        self.servos2 = int(os.environ.get("INIT_SERVO_S2"))
        self.pub_Servo = self.create_publisher(ServoControl, 'Servo', 10)
        self.servo_angle.s1 = self.servos1
        self.servo_angle.s2 = 35

        self.bridge = CvBridge()
        camera_type = os.getenv('CAMERA_TYPE', 'usb')
        topic_name = '/ascamera_hp60c/camera_publisher/rgb0/image' if camera_type == 'nuwa' else '/usb_cam/image_raw'
        
        self.subscription = self.create_subscription(
            Image,
            topic_name, 
            self.image_callback,
            10)

        for i in range(3):
        	self.pub_Servo.publish(self.servo_angle)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        payload, (x, y, w, h) = self.detect_qrcode(frame)
        if payload != None:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 225, 255), 2)
            cv2.putText(frame, payload, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 225, 255), 2)
            self.robot_action(payload)
        else:
            self.pub_vel(0.0,0.0,0.0)

        cv2.imshow('frame', frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("Exiting program")
            cv2.destroyAllWindows()
            rclpy.shutdown()
            exit(0)

    def detect_qrcode(self,image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        barcodes = pyzbar.decode(gray)
        for barcode in barcodes:
            (x, y, w, h) = barcode.rect
            barcodeData = barcode.data.decode("utf-8")
            barcodeType = barcode.type
            return barcodeData, (x, y, w, h)
        return None, (0, 0, 0, 0)

    def pub_vel(self, x, y, z):
        twist = Twist()
        twist.linear.x = x
        twist.linear.y = y
        twist.angular.z = z
        self.pubCmdVel.publish(twist)
    

    def robot_action(self,data):
        if data == "forward":
            self.pub_vel(0.1,0.0,0.0)

        elif data == "back":
            self.pub_vel(-0.1,0.0,0.0)
            
        elif data == "left":
            self.pub_vel(0.1,0.0,1.0)
            
        elif data == "right":
            self.pub_vel(0.1,0.0,-1.0)

        elif data == "stop":
            self.pub_vel(0.0,0.0,0.0)

    def cancel(self):
        cmd1 = "ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "
        cmd2 = '''"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"'''
        cmd = cmd1 +cmd2
        os.system(cmd)

        cmd1 = "ros2 topic pub -1 /Servo yahboomcar_msgs/msg/ServoControl "
        cmd2 = f'''"{{'s1': {self.servos1}, 's2': {self.servos2}}}"'''
        cmd = cmd1 +cmd2
        os.system(cmd)
        print("Shutting down this node.")
        cv2.destroyAllWindows()


def main():
    rclpy.init()
    QRdetect = QR_Tracker('qrtracker')
    print("start it")
    try:
        rclpy.spin(QRdetect)
    except KeyboardInterrupt:
        pass
    finally:
        QRdetect.cancel()
        QRdetect.destroy_node()
