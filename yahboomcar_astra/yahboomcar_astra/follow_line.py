#ros lib
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
#common lib
import os
from yahboomcar_astra.common.follow_common import *

LINEAR_SPEED = 0.38

KP = 90
KI = 0
KD = 12

class LineDetect(Node):
	def __init__(self,name):
		super().__init__(name)
		#create a publisher
		self.pub_cmdVel = self.create_publisher(Twist,"/cmd_vel",1)
		self.pub_turnOptions = self.create_publisher(String,"/turn_options", 10)
		self.circle = ()
		self.hsv_range = ()
		self.Roi_init = ()
		self.end = 0
		self.new_color_selected = False
		self.is_dragging = False
		self.Track_state = 'init'
		self.windows_name = 'frame'
		self.cols, self.rows = 0, 0
		self.Mouse_XY = (0, 0)
		self.color = color_follow()

		self.declare_param()
		self.PID_init()
		
		self.capture = cv.VideoCapture(0)
		self.capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'MJPG'))
		self.timer = self.create_timer(0.011, self.on_timer)

		cv.namedWindow(self.windows_name, cv.WINDOW_AUTOSIZE)
		cv.setMouseCallback(self.windows_name, self.onMouse, 0)	

	def declare_param(self):
		#HSV
		self.declare_parameter("Hmin",0)
		self.Hmin = self.get_parameter('Hmin').get_parameter_value().integer_value
		self.declare_parameter("Smin",85)
		self.Smin = self.get_parameter('Smin').get_parameter_value().integer_value
		self.declare_parameter("Vmin",126)
		self.Vmin = self.get_parameter('Vmin').get_parameter_value().integer_value
		self.declare_parameter("Hmax",9)
		self.Hmax = self.get_parameter('Hmax').get_parameter_value().integer_value
		self.declare_parameter("Smax",253)
		self.Smax = self.get_parameter('Smax').get_parameter_value().integer_value
		self.declare_parameter("Vmax",253)
		self.Vmax = self.get_parameter('Vmax').get_parameter_value().integer_value
        #PID
		self.declare_parameter("Kp",KP)
		self.Kp = self.get_parameter('Kp').get_parameter_value().integer_value
		self.declare_parameter("Ki",KI)
		self.Ki = self.get_parameter('Ki').get_parameter_value().integer_value
		self.declare_parameter("Kd",KD)
		self.Kd = self.get_parameter('Kd').get_parameter_value().integer_value
		self.FollowLinePID = (self.Kp,self.Ki,self.Kd)
		#other
		self.declare_parameter("scale",1000)
		self.scale = self.get_parameter('scale').get_parameter_value().integer_value
		self.declare_parameter("linear",LINEAR_SPEED)
		self.linear = self.get_parameter('linear').get_parameter_value().double_value
		
		

	def PID_init(self):
		self.PID_controller = simplePID(
            [0, 0],
            [self.FollowLinePID[0] / 1.0 / (self.scale), 0],
            [self.FollowLinePID[1] / 1.0 / (self.scale), 0],
            [self.FollowLinePID[2] / 1.0 / (self.scale), 0])
		
	def onMouse(self, event, x, y, flags, param):
		if event == 1:
			self.Track_state = 'init'
			self.is_dragging = True
			self.Mouse_XY = (x,y)
		if event == 4:
			self.is_dragging = False
			self.Track_state = 'mouse'
		if self.is_dragging == True:
			self.cols = min(self.Mouse_XY[0], x), min(self.Mouse_XY[1], y)
			self.rows = max(self.Mouse_XY[0], x), max(self.Mouse_XY[1], y)
			self.Roi_init = (self.cols[0], self.cols[1], self.rows[0], self.rows[1])

	def select_color_roi(self, rgb_img):
		cv.line(rgb_img, self.cols, self.rows, (255, 0, 0), 2)
		cv.rectangle(rgb_img, self.cols, self.rows, (0, 255, 0), 2)
		if self.Roi_init[0]!=self.Roi_init[2] and self.Roi_init[1]!=self.Roi_init[3]:
			rgb_img, self.hsv_range = self.color.Roi_hsv(rgb_img, self.Roi_init)
			self.new_color_selected = True
		return rgb_img

	def save_color_params(self):
		P, T = rclpy.parameter.Parameter, rclpy.Parameter.Type.INTEGER
		low, high = self.hsv_range
		self.set_parameters([
			P('Hmin', T, low[0]), P('Smin', T, low[1]), P('Vmin', T, low[2]),
			P('Hmax', T, high[0]), P('Smax', T, high[1]), P('Vmax', T, high[2]),
		])
		self.new_color_selected = False

	def execute(self, point_x, color_radius):
		if color_radius == 0:
			print("Not Found")
			self.pub_cmdVel.publish(Twist())
		else:
			twist = Twist()
			[z_Pid, _] = self.PID_controller.update([(point_x - 320)*1.0/16, 0])
			twist.angular.z = z_Pid
			twist.linear.x = self.linear
			self.pub_cmdVel.publish(twist)

	def process(self, rgb_img, action):
		
		binary = []
		rgb_img = cv.resize(rgb_img, (640, 480))
        
		if action == 32: self.Track_state = 'tracking'
		elif action == ord('r'): self.Reset()
		elif action == ord('q'): self.cancel()
		if self.Track_state == 'init' and self.is_dragging == True:
			rgb_img = self.select_color_roi(rgb_img)
		if self.Track_state != 'init' and len(self.hsv_range) != 0:
			rgb_img, binary, self.circle, detect_lines, act_lines = self.color.line_follow(rgb_img, self.hsv_range)
			msg = String()
			msg.data = f"Detected lines: {detect_lines} | Acting on lines: {act_lines}"
			self.pub_turnOptions.publish(msg)
			if self.new_color_selected:
				self.save_color_params()
		if self.Track_state == 'tracking':
			if len(self.circle) != 0:
				self.execute(self.circle[0], self.circle[2])
		return rgb_img, binary


	def on_timer(self):
		self.get_param()
		ret, frame = self.capture.read()
		action = cv.waitKey(10) & 0xFF
		frame, binary =self.process(frame, action)
		start = time.time()
		fps = 1 / (start - self.end)
		text = "FPS : " + str(int(fps))
		self.end = start
		cv.putText(frame, text, (30, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1)
		if len(binary) != 0: cv.imshow('frame', ManyImgs(1, ([frame, binary])))
		else:cv.imshow('frame', frame)
		if action == ord('q'):
			self.capture.release()
			cv.destroyAllWindows()

        
		
	def cancel(self):
		cmd1 = "ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "
		cmd2 = '''"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"'''
		cmd = cmd1 +cmd2
		os.system(cmd)
		print("Shutting down this node.")
		cv.destroyAllWindows()	

	def Reset(self):
		self.PID_init()
		self.Track_state = 'init'
		self.hsv_range = ()
		self.Mouse_XY = (0, 0)
		self.pub_cmdVel.publish(Twist())
		print("Reset succes!!!")

	def get_param(self):
		#hsv
		self.Hmin = self.get_parameter('Hmin').get_parameter_value().integer_value
		self.Smin = self.get_parameter('Smin').get_parameter_value().integer_value
		self.Vmin = self.get_parameter('Vmin').get_parameter_value().integer_value
		self.Hmax = self.get_parameter('Hmax').get_parameter_value().integer_value
		self.Smax = self.get_parameter('Smax').get_parameter_value().integer_value
		self.Vmax = self.get_parameter('Vmax').get_parameter_value().integer_value
		#kpi
		self.Kd = self.get_parameter('Kd').get_parameter_value().integer_value
		self.Ki = self.get_parameter('Ki').get_parameter_value().integer_value
		self.Kp = self.get_parameter('Kp').get_parameter_value().integer_value
		self.FollowLinePID = (self.Kp,self.Ki,self.Kd)
		#
		self.scale = self.get_parameter('scale').get_parameter_value().integer_value
		self.linear = self.get_parameter('linear').get_parameter_value().double_value

		
def main():
	rclpy.init()
	linedetect = LineDetect("follow_line")
	print("start it")
	try:
		rclpy.spin(linedetect)
	except KeyboardInterrupt:
		pass
	finally:
		linedetect.cancel()
		linedetect.destroy_node()
