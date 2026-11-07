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

class color_Tracker(Node):
    def __init__(self,name):
        super().__init__(name)
        #create the publisher
        self.pub_Servo = self.create_publisher(ServoControl, 'Servo', 10)
        self.pub_position = self.create_publisher(Position, "/Current_point", 10)

        self.bridge = CvBridge()

        self.Center_x = 0
        self.Center_y = 0
        self.Center_r = 0
        self.prev_time = 0

        self.Robot_Run = False
        self.img_flip = False
        self.encoding = ['8UC3']

        # hsv
        self.Roi_init = ()
        self.hsv_range = ()
        self.end = 0
        self.circle = (0, 0, 0)
        self.point_pose = (0, 0, 0)
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
        self.prev_linear_PID = None
        self.get_param()
        
        self.capture = cv.VideoCapture(0, cv.CAP_V4L2)
        self.capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'MJPG'))
        self.hsv_text = "/root/yahboomcar_ros2_ws/yahboomcar_ws/src/yahboomcar_astra/yahboomcar_astra/colorHSV.text"
        self.timer = self.create_timer(0.011, self.on_timer)
        print("init done")
        
    def declare_param(self):
        # pid init
        self.declare_parameter("linear_Kp",12.0)
        self.linear_Kp = self.get_parameter('linear_Kp').get_parameter_value().double_value
        self.declare_parameter("linear_Ki",0.0)
        self.linear_Ki = self.get_parameter('linear_Ki').get_parameter_value().double_value
        self.declare_parameter("linear_Kd",0.8)
        self.linear_Kd = self.get_parameter('linear_Kd').get_parameter_value().double_value

        self.scale = 1000
        # servo init
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
        # PID
        self.linear_Kp = self.get_parameter('linear_Kp').get_parameter_value().double_value
        self.linear_Ki = self.get_parameter('linear_Ki').get_parameter_value().double_value
        self.linear_Kd = self.get_parameter('linear_Kd').get_parameter_value().double_value

        current_linear_PID = (self.linear_Kp, self.linear_Ki, self.linear_Kd)
        if current_linear_PID != self.prev_linear_PID:
            self.linear_pid.update_params(
                [self.linear_Kp / float(self.scale), self.linear_Kp / float(self.scale)],
                [self.linear_Ki / float(self.scale), self.linear_Ki / float(self.scale)],
                [self.linear_Kd / float(self.scale), self.linear_Kd / float(self.scale)]
            )
            self.prev_linear_PID = current_linear_PID

        # HSV 
        self.Hmin = self.get_parameter('Hmin').get_parameter_value().integer_value
        self.Smin = self.get_parameter('Smin').get_parameter_value().integer_value
        self.Vmin = self.get_parameter('Vmin').get_parameter_value().integer_value
        self.Hmax = self.get_parameter('Hmax').get_parameter_value().integer_value
        self.Smax = self.get_parameter('Smax').get_parameter_value().integer_value
        self.Vmax = self.get_parameter('Vmax').get_parameter_value().integer_value


    def PID_init(self):
        self.linear_pid = simplePID(
            [0, 0],
            [self.linear_Kp / float(self.scale), self.linear_Kp / float(self.scale)],
            [self.linear_Ki / float(self.scale), self.linear_Ki / float(self.scale)],
            [self.linear_Kd / float(self.scale), self.linear_Kd / float(self.scale)]
        )

    def on_timer(self):
        ret, frame = self.capture.read()
        action = cv.waitKey(10) & 0xFF
        frame, binary = self.process(frame, action)
        start = time.time()
        fps = 1 / (start - self.end)
        text_fps = "FPS : " + str(int(fps))
        text = "x:" + str(self.Center_x) + " y:" + str(self.Center_y)
        self.end = start
        cv.putText(frame, text_fps, (30, 30), cv.FONT_HERSHEY_SIMPLEX, 0.8, (100, 200, 200), 2)
        cv.putText(frame, text, (30, 55), cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1)
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
            self.Start_state = True
            if self.circle[2] != 0:
                self.Center_x = self.circle[0]
                self.Center_y = self.circle[1]
                self.Center_r = self.circle[2]
                threading.Thread(target=self.execute, args=(self.circle[0], self.circle[1], self.circle[2])).start()
            if self.point_pose[0] != 0 and self.point_pose[1] != 0:
                self.Center_x = self.point_pose[0]
                self.Center_y = self.point_pose[1]
                self.Center_r = self.point_pose[2]
                threading.Thread(target=self.execute, args=(self.point_pose[0], self.point_pose[1], self.point_pose[2])).start()
            now_time = time.time()
            if now_time - self.prev_time > 5:
                self.prev_time = now_time
                self.execute(self.Center_x, self.Center_y)
        else:
            if self.Start_state == True:
                self.Robot_Run = False
                self.Start_state = False
        return rgb_img, binary

    def execute(self, x, y, z=None):
        position = Position()
        position.anglex = x * 1.0
        position.angley = y * 1.0
        if z is not None:
            position.distance = z * 1.0
        self.pub_position.publish(position)
        [x_Pid, y_Pid] = self.linear_pid.update([(x - 320), y - 240])
        #print(f"x_Pid {x_Pid}, y_Pid {y_Pid}")
        x_Pid = x_Pid * (abs(x_Pid) <=self.linear_Kp/2.4)
        y_Pid = y_Pid * (abs(y_Pid) <=self.linear_Kp/2.4)
        if self.img_flip == True:
            self.PWMServo_X -= x_Pid
            self.PWMServo_Y -= y_Pid
        else:
            self.PWMServo_X  += x_Pid
            self.PWMServo_Y  += y_Pid
        #print(f"x_Pid {x_Pid}, y_Pid {y_Pid},self.PWMServo_X {self.PWMServo_X}, self.PWMServo_Y {self.PWMServo_Y}")
        self.PWMServo_X = int(max(0, min(180, self.PWMServo_X)))
        self.PWMServo_Y = int(max(0, min(100, self.PWMServo_Y)))
        self.servo_angle.s1 = self.PWMServo_X
        self.servo_angle.s2 = self.PWMServo_Y
        #print(f"self.servo_angle.s1 {self.servo_angle.s1}, self.servo_angle.s2 {self.servo_angle.s2}")
        self.pub_Servo.publish(self.servo_angle)

    def Reset(self):
        self.hsv_range = ()
        self.circle = (0, 0, 0)
        self.Mouse_XY = (0, 0)
        self.Center_x = self.Center_y = 0
        self.Track_state = 'init'
        for _ in range(3):
            self.pub_position.publish(Position())
        print("succes!!!")

    def cancel(self):
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
    color_tracker = color_Tracker("ColorTracker")
    print("start it")
    try:
        rclpy.spin(color_tracker)
    except KeyboardInterrupt:
        pass
    finally:
        color_tracker.cancel()
        color_tracker.destroy_node()
