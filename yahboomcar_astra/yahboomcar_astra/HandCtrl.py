#!/usr/bin/env python3
# encoding: utf-8
import threading
import cv2 as cv
import numpy as np
from yahboomcar_mediapipe.media_library import *
from yahboomcar_msgs.msg import ServoControl
from time import sleep, time
import rclpy
from rclpy.node import Node
import os
from cv_bridge import CvBridge
class PoseCtrlArm(Node):
    def __init__(self,name):
        super().__init__(name)
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

        self.car_status = True
        self.stop_status = 0
        self.locking = False
        self.pose_detector = Holistic()
        self.hand_detector = HandDetector()
        self.pTime = self.index = 0
        for i in range(3):
        	self.pub_Servo.publish(self.servo_angle)
        self.media_ros = MediaROS()
        self.event = threading.Event()
        self.event.set()

    def process(self, frame):
        frame = cv.flip(frame, 1)

        frame, lmList, _ = self.hand_detector.findHands(frame,False)
        if len(lmList) != 0:
            threading.Thread(target=self.hand_threading, args=(lmList,)).start()
        else:self.media_ros.pub_vel(0.0, 0.0,0.0)
            
        self.cTime = time()  
        fps = 1 / (self.cTime - self.pTime)
        self.pTime = self.cTime 
        text = "FPS : " + str(int(fps))
        cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
        
        return frame

    def hand_threading(self, lmList):
        if self.event.is_set():
            self.event.clear()
            self.stop_status = 0
            self.index = 0
            fingers = self.hand_detector.fingersUp(lmList)
            print("fingers: ", fingers)
            if sum(fingers) == 5: 
                self.media_ros.pub_vel(0.09, 0.0,0.0)
                sleep(0.5)
                
            elif sum(fingers) == 3: 
                self.media_ros.pub_vel(-0.09, 0.0,0.0)
                sleep(0.5)
                
            elif sum(fingers) == 1 and fingers[1] == 1: 
                self.media_ros.pub_vel(0.09, 0.0, -0.7)
                sleep(0.5)
                
            elif sum(fingers) == 2 and fingers[1] == 1 and fingers[2] == 1:
                self.media_ros.pub_vel(0.0, 0.0, 0.0)
                sleep(0.5)
                
            elif sum(fingers) == 4:
                self.media_ros.pub_vel(0.09, 0.0, 0.7)
                sleep(0.5)
            else:
                self.media_ros.pub_vel(0.0, 0.0, 0.0)
            self.event.set()

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame = self.process(frame)
        cv.imshow('HandDetector', frame)
        
        if cv.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("Exiting program")
            cv.destroyAllWindows()
            self.destroy_node()
            rclpy.shutdown()
            exit(0)

def main():
    rclpy.init()
    pose_ctrl_arm = PoseCtrlArm('posectrlarm')
    try:
        rclpy.spin(pose_ctrl_arm)
    except KeyboardInterrupt:
        pass
    finally:
        cv.destroyAllWindows()
        pose_ctrl_arm.destroy_node()
        rclpy.shutdown()
