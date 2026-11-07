#ros lib
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage, LaserScan, Image
from yahboomcar_msgs.msg import Position, ServoControl
from cv_bridge import CvBridge
#common lib
import os
import threading
import math
import time
from yahboomcar_astra.common.astra_common import *
print("import done")

class color_Follow(Node):
    def __init__(self,name):
        super().__init__(name)
        #create the publisher
        self.pub_Servo = self.create_publisher(ServoControl, 'Servo', 10)
        self.pub_cmdVel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_img = self.create_publisher(Image, '/image_raw', 500)
        self.pub_position = self.create_publisher(Position, "/Current_point", 10)
        #create the subscription
        self.sub_JoyState = self.create_subscription(Bool, '/JoyState',  self.JoyStateCallback, 1)
        
        self.bridge = CvBridge()
        self.Center_x = 0
        self.Center_y = 0
        self.Center_r = 0
        self.prev_time = 0

        self.Joy_active = False
        self.img_flip = False
        self.twist = Twist()
        self.encoding = ['8UC3']

        # hsv
        self.Roi_init = ()
        self.hsv_range = ()
        self.end = 0
        self.circle = (0, 0, 0)
        self.dyn_update = True
        self.Start_state = True
        self.select_flags = False
        self.gTracker_state = False
        self.windows_name = 'frame'
        self.Track_state = 'identify'
        self.color = color_follow()
        self.cols, self.rows = 0, 0
        self.Mouse_XY = (0, 0)
        # pid 
        self.declare_param()
        self.PID_init()
        self.prev_main_PID = None
        self.prev_angular_PID = None
        self.get_param()
        
        
        self.capture = cv.VideoCapture(0, cv.CAP_V4L2)
        self.capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'MJPG'))
        self.hsv_text = "/root/yahboomcar_ros2_ws/yahboomcar_ws/src/yahboomcar_astra/yahboomcar_astra/colorHSV.text"
        self.timer = self.create_timer(0.011, self.on_timer)
        print("init done")
        
    def declare_param(self):
        # limit
        self.declare_parameter("mintra_r",65.0)
        self.mintra_r = self.get_parameter('mintra_r').get_parameter_value().double_value
        self.declare_parameter("tolerance_x",3.0)
        self.tolerance_x = self.get_parameter('tolerance_x').get_parameter_value().double_value
        self.declare_parameter("tolerance_r",10.0)
        self.tolerance_r = self.get_parameter('tolerance_r').get_parameter_value().double_value
        
        # pid init
        self.declare_parameter("linear_Kp",12.0)
        self.linear_Kp = self.get_parameter('linear_Kp').get_parameter_value().double_value
        self.declare_parameter("linear_Ki",0.0)
        self.linear_Ki = self.get_parameter('linear_Ki').get_parameter_value().double_value
        self.declare_parameter("linear_Kd",1.0)
        self.linear_Kd = self.get_parameter('linear_Kd').get_parameter_value().double_value

        self.declare_parameter("angular_Kp",35.0)
        self.angular_Kp = self.get_parameter('angular_Kp').get_parameter_value().double_value
        self.declare_parameter("angular_Ki",0.0)
        self.angular_Ki = self.get_parameter('angular_Ki').get_parameter_value().double_value
        self.declare_parameter("angular_Kd",0.8)
        self.angular_Kd = self.get_parameter('angular_Kd').get_parameter_value().double_value

        self.scale = 1000
        # servo init
        self.declare_parameter("Servo_Kp",12.0)
        self.Servo_Kp = self.get_parameter('Servo_Kp').get_parameter_value().double_value
        self.declare_parameter("Servo_Ki",0.0)
        self.Servo_Ki = self.get_parameter('Servo_Ki').get_parameter_value().double_value
        self.declare_parameter("Servo_Kd",0.8)
        self.Servo_Kd = self.get_parameter('Servo_Kd').get_parameter_value().double_value
        
        self.init_servos1 = self.PWMServo_X = int(os.environ.get("INIT_SERVO_S1"))
        self.init_servos2 = self.PWMServo_Y = int(os.environ.get("INIT_SERVO_S2"))
        self.servo_angle = ServoControl()
        self.servo_angle.s1 = self.init_servos1
        self.servo_angle.s2 = self.init_servos2
        for i in range(3):
            self.pub_Servo.publish(self.servo_angle)

        # HSV
        self.declare_parameter("Hmin", 0)
        self.Hmin = self.get_parameter('Hmin').get_parameter_value().integer_value
        self.declare_parameter("Smin", 85)
        self.Smin = self.get_parameter('Smin').get_parameter_value().integer_value
        self.declare_parameter("Vmin", 126)
        self.Vmin = self.get_parameter('Vmin').get_parameter_value().integer_value
        self.declare_parameter("Hmax", 9)
        self.Hmax = self.get_parameter('Hmax').get_parameter_value().integer_value
        self.declare_parameter("Smax", 253)
        self.Smax = self.get_parameter('Smax').get_parameter_value().integer_value
        self.declare_parameter("Vmax", 253)
        self.Vmax = self.get_parameter('Vmax').get_parameter_value().integer_value


    def get_param(self):
        self.mintra_r = self.get_parameter('mintra_r').get_parameter_value().double_value
        self.tolerance_x = self.get_parameter('tolerance_x').get_parameter_value().double_value
        self.tolerance_r = self.get_parameter('tolerance_r').get_parameter_value().double_value
        # PID
        self.linear_Kp = self.get_parameter('linear_Kp').get_parameter_value().double_value
        self.linear_Ki = self.get_parameter('linear_Ki').get_parameter_value().double_value
        self.linear_Kd = self.get_parameter('linear_Kd').get_parameter_value().double_value

        self.angular_Kp = self.get_parameter('angular_Kp').get_parameter_value().double_value
        self.angular_Ki = self.get_parameter('angular_Ki').get_parameter_value().double_value
        self.angular_Kd = self.get_parameter('angular_Kd').get_parameter_value().double_value

        self.Servo_Kp = self.get_parameter('Servo_Kp').get_parameter_value().double_value
        self.Servo_Ki = self.get_parameter('Servo_Ki').get_parameter_value().double_value
        self.Servo_Kd = self.get_parameter('Servo_Kd').get_parameter_value().double_value

        current_main_PID = (self.linear_Kp, self.linear_Ki, self.linear_Kd, self.Servo_Kp, self.Servo_Ki, self.Servo_Kd)
        if current_main_PID != self.prev_main_PID:
            self.PID_controller.update_params(
                [self.linear_Kp / float(self.scale), self.Servo_Kp / float(self.scale), self.Servo_Kp / float(self.scale)],
                [self.linear_Ki / float(self.scale), self.Servo_Ki / float(self.scale), self.Servo_Ki / float(self.scale)],
                [self.linear_Kd / float(self.scale), self.Servo_Kd / float(self.scale), self.Servo_Kd / float(self.scale)]
            )
            self.prev_main_PID = current_main_PID

        current_angular_PID = (self.angular_Kp, self.angular_Ki, self.angular_Kd)
        if current_angular_PID != self.prev_angular_PID:
            self.angular_PID_controller.update_params(
                [self.angular_Kp / float(self.scale)],
                [self.angular_Ki / float(self.scale)],
                [self.angular_Kd / float(self.scale)]
            )
            self.prev_angular_PID = current_angular_PID

        # HSV 
        self.Hmin = self.get_parameter('Hmin').get_parameter_value().integer_value
        self.Smin = self.get_parameter('Smin').get_parameter_value().integer_value
        self.Vmin = self.get_parameter('Vmin').get_parameter_value().integer_value
        self.Hmax = self.get_parameter('Hmax').get_parameter_value().integer_value
        self.Smax = self.get_parameter('Smax').get_parameter_value().integer_value
        self.Vmax = self.get_parameter('Vmax').get_parameter_value().integer_value


    def PID_init(self):
        self.PID_controller = simplePID(
            [0, 0, 0], 
            [self.linear_Kp / float(self.scale), self.Servo_Kp / float(self.scale), self.Servo_Kp / float(self.scale)],  # 调整参数数量
            [self.linear_Ki / float(self.scale), self.Servo_Ki / float(self.scale), self.Servo_Ki / float(self.scale)],
            [self.linear_Kd / float(self.scale), self.Servo_Kd / float(self.scale), self.Servo_Kd / float(self.scale)]
        )
        
        self.angular_PID_controller = simplePID(
            [0],
            [self.angular_Kp / float(self.scale)],
            [self.angular_Ki / float(self.scale)],
            [self.angular_Kd / float(self.scale)]
        )

    def on_timer(self):
        ret, frame = self.capture.read()
        action = cv.waitKey(10) & 0xFF
        frame, binary = self.process(frame, action)
        start = time.time()
        fps = 1 / (start - self.end)
        text_fps = "FPS : " + str(int(fps))
        text = "x:" + str(self.Center_x) + " y:" + str(self.Center_y)+ " radius:" + str(self.Center_r)
        self.end = start
        cv.putText(frame, text_fps, (30, 30), cv.FONT_HERSHEY_SIMPLEX, 0.8, (100, 200, 200), 2)
        cv.putText(frame, text, (30, 55), cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1)
        msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.pub_img.publish(msg)
        if len(binary) != 0: 
            cv.imshow('frame', ManyImgs(1, ([frame, binary])))
        else:
            cv.imshow('frame', frame)
        if action == ord('q') or action == 113:
            self.capture.release()
            cv.destroyAllWindows()

    def process(self, rgb_img, action):
        self.get_param()
        rgb_img = cv.resize(rgb_img, (640, 480))
        binary = []
        if action == 32:
            self.Track_state = 'tracking'
            print("Color Tracking!!!")
        elif action == ord('i') or action == ord('I'):
            self.Track_state = "identify"
        elif action == ord('r') or action == ord('R'):
            self.Reset()
        elif action == ord('q') or action == ord('Q'):
            self.cancel()
        if self.Track_state == 'init':
            self.servo_angle.s1 = self.init_servos1
            self.servo_angle.s2 = self.init_servos2
            self.pub_Servo.publish(self.servo_angle)
            cv.namedWindow(self.windows_name, cv.WINDOW_AUTOSIZE)
            cv.setMouseCallback(self.windows_name, self.onMouse, 0)
            if self.select_flags == True:
                cv.line(rgb_img, self.cols, self.rows, (255, 0, 0), 2)
                cv.rectangle(rgb_img, self.cols, self.rows, (0, 255, 0), 2)
                if self.Roi_init[0] != self.Roi_init[2] and self.Roi_init[1] != self.Roi_init[3]:
                    rgb_img, self.hsv_range = self.color.Roi_hsv(rgb_img, self.Roi_init)
                    self.gTracker_state = True
                    self.dyn_update = True
                else:
                    self.Track_state = 'init'
        elif self.Track_state == "identify":
            if os.path.exists(self.hsv_text):
                self.hsv_range = read_HSV(self.hsv_text)
            else:
                self.Track_state = 'init'
        if self.Track_state != 'init':
            if len(self.hsv_range) != 0:
                rgb_img, binary, self.circle = self.color.object_follow(rgb_img, self.hsv_range)
                if self.dyn_update == True:
                    write_HSV(self.hsv_text, self.hsv_range)
                    self.Hmin  = rclpy.parameter.Parameter('Hmin', rclpy.Parameter.Type.INTEGER, self.hsv_range[0][0])
                    self.Smin  = rclpy.parameter.Parameter('Smin', rclpy.Parameter.Type.INTEGER, self.hsv_range[0][1])
                    self.Vmin  = rclpy.parameter.Parameter('Vmin', rclpy.Parameter.Type.INTEGER, self.hsv_range[0][2])
                    self.Hmax  = rclpy.parameter.Parameter('Hmax', rclpy.Parameter.Type.INTEGER, self.hsv_range[1][0])
                    self.Smax  = rclpy.parameter.Parameter('Smax', rclpy.Parameter.Type.INTEGER, self.hsv_range[1][1])
                    self.Vmax  = rclpy.parameter.Parameter('Vmax', rclpy.Parameter.Type.INTEGER, self.hsv_range[1][2])
                    all_new_parameters = [self.Hmin, self.Smin, self.Vmin, self.Hmax, self.Smax, self.Vmax]
                    self.set_parameters(all_new_parameters)
                    self.dyn_update = False
        if self.Track_state in ['mouse', 'identify', 'tracking']:
            values = [p.value if isinstance(p, rclpy.Parameter) else int(p) 
                for p in [self.Hmin, self.Smin, self.Vmin, self.Hmax, self.Smax, self.Vmax]]
            self.hsv_range = (
                (values[0], values[1], values[2]), 
                (values[3], values[4], values[5])
            )
        if self.Track_state == 'tracking':

            if self.circle[2] != 0:
                self.Center_x = self.circle[0]
                self.Center_y = self.circle[1]
                self.Center_r = self.circle[2]
                threading.Thread(target=self.execute, args=(self.circle[0], self.circle[1], self.circle[2])).start()
            now_time = time.time()
            if now_time - self.prev_time > 3:
                self.prev_time = now_time
                self.execute(320, 240,self.mintra_r)
                #self.execute(self.Center_x, self.Center_y,self.Center_r)
        else:
            if self.Start_state == True:
                self.pub_cmdVel.publish(Twist())
                self.Start_state = False
        return rgb_img, binary

    def JoyStateCallback(self, msg):
        if not isinstance(msg, Bool): return
        self.Joy_active = msg.data
        self.servo_angle.s1 = self.PWMServo_X = self.init_servos1
        self.servo_angle.s2 = self.PWMServo_Y = self.init_servos2
        self.pub_cmdVel.publish(Twist())

    def execute(self, x, y, z=None):
        position = Position()
        position.anglex = x * 1.0
        position.angley = y * 1.0
        position.distance = z * 1.0
        self.pub_position.publish(position)
        if self.Joy_active == True: return
        #print(f"z {z},x {x}")
        [linear_Pid, Servox_Pid, Servoy_Pid] = self.PID_controller.update([z - self.mintra_r, x - 320, y - 240])
        angular_Pid = self.angular_PID_controller.update([self.PWMServo_X - self.init_servos1])[0]
        #print(f"linear_Pid {linear_Pid}, angular_Pid {angular_Pid}")
        linear_Pid = max(-0.8, min(linear_Pid, 0.8)) 
        angular_Pid = max(-3.0, min(angular_Pid, 3.0))

        linear_Pid = abs(linear_Pid) if (z - self.mintra_r > 0) else -abs(linear_Pid)
        angular_Pid = -abs(angular_Pid) if (self.PWMServo_X - self.init_servos1) > 0 else abs(angular_Pid)
        Servox_Pid = Servox_Pid * (abs(Servox_Pid) <=self.Servo_Kp/2.4)
        Servoy_Pid = Servoy_Pid * (abs(Servoy_Pid) <=self.Servo_Kp/2.4)
        #print(f"linear_Pid {linear_Pid}, angular_Pid {angular_Pid}, Servoy_Pid {Servoy_Pid}")
        if abs(self.PWMServo_X - self.init_servos1) < self.tolerance_x: angular_Pid = 0.0
        if abs(z - self.mintra_r) < self.tolerance_r or z <10: linear_Pid = 0.0
        self.twist.linear.x = -linear_Pid
        if self.img_flip == True:
            self.twist.angular.z = angular_Pid
            self.PWMServo_X -= Servox_Pid
            self.PWMServo_Y -= Servoy_Pid
        else:
            self.twist.angular.z = -angular_Pid
            self.PWMServo_X  += Servox_Pid
            self.PWMServo_Y  += Servoy_Pid
        #print(f"twist.linear.x {self.twist.linear.x}, twist.angular.z {self.twist.angular.z}")
        self.PWMServo_Y = int(max(0, min(100, self.PWMServo_Y)))
        self.PWMServo_X = int(max(0, min(180, self.PWMServo_X)))
        self.servo_angle.s1 = self.PWMServo_X
        self.servo_angle.s2 = self.PWMServo_Y
        #print(f"twist.linear.x {self.twist.linear.x}, twist.angular.z {self.twist.angular.z}, servo_angle.s2 {self.servo_angle.s2}")
        self.pub_Servo.publish(self.servo_angle)
        self.pub_cmdVel.publish(self.twist)
        self.Start_state = True

    def Reset(self):
        self.hsv_range = ()
        self.circle = (0, 0, 0)
        self.Mouse_XY = (0, 0)
        self.Center_x = self.Center_y = self.Center_r = 0
        self.Track_state = 'init'
        for _ in range(3):
            self.pub_position.publish(Position())
        print("succes!!!")
    
    def cancel(self):
        cmd1 = "ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "
        cmd2 = '''"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"'''
        cmd = cmd1 +cmd2
        os.system(cmd)

        cmd1 = "ros2 topic pub -1 /Servo yahboomcar_msgs/msg/ServoControl "
        cmd2 = f'''"{{'s1': {self.init_servos1}, 's2': {self.init_servos2}}}"'''
        cmd = cmd1 +cmd2
        os.system(cmd)
        print("Shutting down this node.")
        cv.destroyAllWindows()

    def onMouse(self, event, x, y, flags, param):
        if event == 1:
            self.Track_state = 'init'
            self.select_flags = True
            self.Mouse_XY = (x, y)
        if event == 4:
            self.select_flags = False
            self.Track_state = 'mouse'
        if self.select_flags == True:
            self.cols = min(self.Mouse_XY[0], x), min(self.Mouse_XY[1], y)
            self.rows = max(self.Mouse_XY[0], x), max(self.Mouse_XY[1], y)
            self.Roi_init = (self.cols[0], self.cols[1], self.rows[0], self.rows[1])

class simplePID:
    '''very simple discrete PID controller'''

    def __init__(self, target, P, I, D):
        '''Create a discrete PID controller
        each of the parameters may be a vector if they have the same length
        Args:
        target (double) -- the target value(s)
        P, I, D (double)-- the PID parameter
        '''
        # check if parameter shapes are compatabile.
        if (not (np.size(P) == np.size(I) == np.size(D)) or ((np.size(target) == 1) and np.size(P) != 1) or (
                np.size(target) != 1 and (np.size(P) != np.size(target) and (np.size(P) != 1)))):
            raise TypeError('input parameters shape is not compatable')
        self.Kp = np.array(P)
        self.Ki = np.array(I)
        self.Kd = np.array(D)
        self.last_error = 0
        self.integrator = 0
        self.timeOfLastCall = None
        self.setPoint = np.array(target)
        self.integrator_max = float('inf')

    def update_params(self, P, I, D):
        '''Dynamic update of PID parameters'''
        self.Kp = np.array(P)
        self.Ki = np.array(I)
        self.Kd = np.array(D)
        self.integrator = 0
        self.last_error = 0

    def update(self, current_value):
        '''Updates the PID controller.
        Args:
            current_value (double): vector/number of same legth as the target given in the constructor
        Returns:
            controll signal (double): vector of same length as the target
        '''
        current_value = np.array(current_value)
        if (np.size(current_value) != np.size(self.setPoint)):
            raise TypeError('current_value and target do not have the same shape')
        if (self.timeOfLastCall is None):
            # the PID was called for the first time. we don't know the deltaT yet
            # no controll signal is applied
            self.timeOfLastCall = time.perf_counter()
            return np.zeros(np.size(current_value))
        error = self.setPoint - current_value
        P = error
        currentTime = time.perf_counter()
        deltaT = (currentTime - self.timeOfLastCall)
        # integral of the error is current error * time since last update
        self.integrator = self.integrator + (error * deltaT)
        I = self.integrator
        # derivative is difference in error / time since last update
        D = (error - self.last_error) / deltaT
        self.last_error = error
        self.timeOfLastCall = currentTime
        # return controll signal
        return self.Kp * P + self.Ki * I + self.Kd * D

def main():
    rclpy.init()
    color_follow = color_Follow("ColorFollow")
    print("start it")
    try:
        rclpy.spin(color_follow)
    except KeyboardInterrupt:
        pass
    finally:
        color_follow.cancel()
        color_follow.destroy_node()