#ros lib
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Bool,UInt16
from sensor_msgs.msg import CompressedImage, LaserScan, Image
from message_filters import ApproximateTimeSynchronizer, Subscriber
#common lib
import os
import threading
import math
from yahboomcar_depth.astra_common import *
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge

print("import done")


class faceFollow(Node):
    def __init__(self,name):
        super().__init__(name)
        #create the publisher
        self.pub_cmdVel = self.create_publisher(Twist,'/cmd_vel',10)
        self.declare_param()
        self.get_param()
        #create the subscriber
        self.depth_image_sub = Subscriber(self, Image, '/ascamera_hp60c/camera_publisher/depth0/image_raw')
        self.rgb_image_sub = Subscriber(self, Image, '/ascamera_hp60c/camera_publisher/rgb0/image')
        self.ts = ApproximateTimeSynchronizer([self.rgb_image_sub, self.depth_image_sub],queue_size=10,slop=0.5)
        self.ts.registerCallback(self.depth_img_Callback)

        self.sub_JoyState = self.create_subscription(Bool,'/JoyState',  self.JoyStateCallback,1)
        self.start = self.end = 0

        #跟随
        self.rgb_bridge = CvBridge()
        self.depth_bridge = CvBridge()
        self.prev_dist = 0
        self.prev_angular = 0
        self.Center_prevx = 0
        self.Center_prevr = 0

        self.dist = 0
        self.Joy_active = False
        self.Robot_Run = False
        self.img_flip = False
        self.twist = Twist()

        self.encoding = ['16UC1', '32FC1']
        xml_path = '/root/yahboomcar_ros2_ws/yahboomcar_ws/src/yahboomcar_depth/yahboomcar_depth/Advanced/haarcascade_frontalface_default.xml'
        self.face_patterns = cv.CascadeClassifier(xml_path)
        self.end = 0
        self.Center_x = self.Center_y = self.Center_r = 0
        self.prev_time = 0
        # pid init
        
        self.linear_PID = (3.0, 0.0, 1.0)
        self.angular_PID = (0.5, 0.0, 2.0)
        self.PID_init()

        self.prev_main_PID = None
        self.prev_angular_PID = None
        
        
        
        print("init done")


    def declare_param(self):
        self.declare_parameter("linear_Kp",3.0)
        self.linear_Kp = self.get_parameter('linear_Kp').get_parameter_value().double_value
        self.declare_parameter("linear_Ki",0.0)
        self.linear_Ki = self.get_parameter('linear_Ki').get_parameter_value().double_value
        self.declare_parameter("linear_Kd",1.0)
        self.linear_Kd = self.get_parameter('linear_Kd').get_parameter_value().double_value
        self.declare_parameter("angular_Kp",0.35)
        self.angular_Kp = self.get_parameter('angular_Kp').get_parameter_value().double_value
        self.declare_parameter("angular_Ki",0.0)
        self.angular_Ki = self.get_parameter('angular_Ki').get_parameter_value().double_value
        self.declare_parameter("angular_Kd",2.0)
        self.angular_Kd = self.get_parameter('angular_Kd').get_parameter_value().double_value
        self.declare_parameter("scale",1000)
        self.scale = self.get_parameter('scale').get_parameter_value().integer_value
        self.declare_parameter("minDistance",0.4)
        self.minDistance = self.get_parameter('minDistance').get_parameter_value().double_value

    def get_param(self):
        self.linear_Kp = self.get_parameter('linear_Kp').get_parameter_value().double_value
        self.linear_Ki = self.get_parameter('linear_Ki').get_parameter_value().double_value
        self.linear_Kd = self.get_parameter('linear_Kd').get_parameter_value().double_value
        self.angular_Kp = self.get_parameter('angular_Kp').get_parameter_value().double_value
        self.angular_Ki = self.get_parameter('angular_Ki').get_parameter_value().double_value
        self.angular_Kd = self.get_parameter('angular_Kd').get_parameter_value().double_value
        self.scale = self.get_parameter('scale').get_parameter_value().integer_value
        self.minDistance = self.get_parameter('minDistance').get_parameter_value().double_value
        
        self.linear_PID = (self.linear_Kp, self.linear_Ki, self.linear_Kd)
        self.angular_PID = (self.angular_Kp, self.angular_Ki, self.angular_Kd)
        self.minDist = self.minDistance * 1000

    def PID_init(self):
        self.linear_pid = simplePID(self.linear_PID[0] / 1000.0, self.linear_PID[1] / 1000.0, self.linear_PID[2] / 1000.0)
        self.angular_pid = simplePID(self.angular_PID[0] / 100.0, self.angular_PID[1] / 100.0, self.angular_PID[2] / 100.0)

    def depth_img_Callback(self,color_frame,depth_frame):
        rgb_image = self.rgb_bridge.imgmsg_to_cv2(color_frame,'bgr8')
        result_image = np.copy(rgb_image)

        depth_image = self.depth_bridge.imgmsg_to_cv2(depth_frame, self.encoding[1])
        frame = cv.resize(depth_image, (640, 480))
        depth_image_info = frame.astype(np.float32)
        
        action = cv.waitKey(10) & 0xFF

        end = time.time()
        fps = 1 / (end - self.start)
        self.start = end
        text = "FPS : " + str(int(fps))
        cv.putText(result_image,text, (30, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1)
       
        faces = self.face_patterns.detectMultiScale(result_image , scaleFactor=1.2, minNeighbors=10, minSize=(50, 50))
        if len(faces) == 1:
            now_time = time.time()
            if now_time - self.prev_time > 5:
                if self.Center_prevx == self.Center_x and self.Center_prevr == self.Center_r: self.Center_r = 0
                self.prev_time = now_time
            for (x, y, w, h) in faces:
                self.Center_x = x + w //2
                self.Center_y = y + h //2
                self.Center_r = w * h //100
                cv.rectangle(result_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

            points = [
                    (int(self.Center_y), int(self.Center_x)),         # 中心点
                    (int(self.Center_y -1), int(self.Center_x -1)),  # 右下偏移点
                    (int(self.Center_y - 3), int(self.Center_x - 3))   # 左上偏移点
                ]
            valid_depths = []
            for y, x in points:
                depth_val = depth_image_info[y][x]
                if depth_val != 0:
                    valid_depths.append(depth_val)
            if valid_depths:
                dist = int(sum(valid_depths) / len(valid_depths))
            else:
                dist = 0
            self.dist = dist
            text = "x:" + str(self.Center_x) + " y:" + str(self.Center_y)+ " dist:" + str(dist)
            cv.putText(result_image, text, (20, 55), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)
            if dist > 0.2:
                self.execute(self.Center_x,dist)
            self.Center_prevx = self.Center_x
            self.Center_prevr = self.Center_r
        else:
            now_time = time.time()
            if now_time - self.prev_time > 0.75:
                self.prev_time = now_time
                self.execute(320,self.minDist)
        if self.dist == 0:
            self.pub_cmdVel.publish(Twist())
            
        cv.imshow('frame', result_image)
        if action == ord('q') or action == 113:
            self.capture.release()
            cv.destroyAllWindows()

            
    def JoyStateCallback(self, msg):
        if not isinstance(msg, Bool):
            return
        self.Joy_active = msg.data
        self.pub_cmdVel.publish(Twist())

    def cancel(self):
        cmd1 = "ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "
        cmd2 = '''"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"'''
        cmd = cmd1 +cmd2
        os.system(cmd)
        print("Shutting down this node.")



    def execute(self, point_x, dist):
        self.get_param()

        if self.Joy_active == True: return

        linear_x = self.linear_pid.compute(dist, self.minDist)
        angular_z = self.angular_pid.compute(point_x,320)
        if abs(dist - self.minDist) < 30: linear_x = 0
        if abs(point_x - 320) < 30: angular_z = 0

        twist = Twist()
        if angular_z>3.0:
            angular_z = 3.0
        if angular_z<-3.0:
            angular_z = -3.0
        if linear_x > 0.3:
            linear_x = 0.3
        if linear_x <-0.3:
            linear_x = -0.3

 
        if linear_x == 0 and abs(point_x - 320.0) > 50:
            twist.angular.z = -angular_z * 1.0
            twist.linear.x = -0.15
        else:
            twist.angular.z = -angular_z * 1.0
            twist.linear.x = linear_x * 1.0
        self.pub_cmdVel.publish(twist)
        self.Robot_Run = True


def main():
    rclpy.init()
    face_Follow = faceFollow("FaceFollow")
    try:
        rclpy.spin(face_Follow)
    except KeyboardInterrupt:
        pass
    finally:
        face_Follow.cancel()