import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
encoding = ['16UC1', '32FC1']

class Edge_DetectionNode(Node):
	def __init__(self, name):
		super().__init__(name)
		self.pub_vel = self.create_publisher(Twist,'/cmd_vel',1)
		self.sub_depth = self.create_subscription(Image,"/ascamera_hp60c/camera_publisher/depth0/image_raw",self.get_DepthImageCallBack,100)
		
		self.lin_x = 0.1
		self.Joy_active = False
		self.depth_bridge = CvBridge()
		self.move_flag = False



	def compute_dist(self,result_frame):
		frame = cv2.resize(result_frame, (640, 480))
		depth_image_info = frame.astype(np.float32)
		if  self.move_flag == True:
			if depth_image_info[240, 360]/1000>0.3:
				self.pubVel(0,0,0)
				self.move_flag = False
				print("Stop!!!")
			else:
				self.pubVel(self.lin_x,0,0)		
				print("Moving....")
		else:
			self.pubVel(0,0,0)
			print("Stop status now!Press the SPACEBAR to change the state.")
			
	


	def get_DepthImageCallBack(self,msg):
		depth_image = self.depth_bridge.imgmsg_to_cv2(msg, encoding[1])
		compute_ = threading.Thread(target=self.compute_dist, args=(depth_image,))
		compute_.start()
		compute_.join()
		key = cv2.waitKey(10)
		if key == 32:
			self.move_flag = True
		cv2.imshow("frame", depth_image)		
		

	def pubVel(self,vx,vy,vz):
		vel = Twist()
		vel.linear.x = float(vx)
		vel.linear.y = float(vy)
		vel.angular.z = float(vz)
		self.pub_vel.publish(vel)


		
		
def main():
	print('----------------------')
	rclpy.init()
	edge_detect = Edge_DetectionNode('Edge_Detection_node')
	rclpy.spin(edge_detect)