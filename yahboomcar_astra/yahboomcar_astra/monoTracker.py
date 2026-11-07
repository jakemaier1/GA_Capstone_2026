#!/usr/bin/env python3
# encoding: utf-8
import getpass
import threading
from sensor_msgs.msg import CompressedImage,Image
from yahboomcar_msgs.msg import Position,ServoControl
from yahboomcar_astra.common.track_common import *
from std_msgs.msg import Int32, Bool,UInt16
import math
import os
import rclpy
from rclpy.node import Node
class mono_Tracker(Node):
    def __init__(self,name):
        super().__init__(name)
        self.pub_Servo = self.create_publisher(ServoControl, 'Servo', 10)
        self.pub_position = self.create_publisher(Position, "/Current_point", 10)
        self.point_pose = (0, 0, 0)
        self.circle = (0, 0, 0)
        self.dyn_update = True
        self.select_flags = False
        self.gTracker_state = False
        self.windows_name = 'frame'
        self.cols, self.rows = 0, 0
        self.Mouse_XY = (0, 0)
        self.index = 2
        self.end = 0
    
        self.tracker_types = ['BOOSTING', 'MIL', 'KCF']
        self.tracker_type = ['KCF'] 
        self.VideoSwitch = True
        self.img_flip = False

        self.scale = 1000
        self.declare_param()
        self.PID_init()
        self.prev_linear_PID = None
        self.get_param()
        

        print("OpenCV Version: ",cv.__version__)
        self.gTracker = Tracker(tracker_type=self.tracker_type)
        self.tracker_type = self.tracker_types[self.index]
        self.Track_state = 'init'
        self.capture = cv.VideoCapture(0, cv.CAP_V4L2)
        self.capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'MJPG'))
        self.timer = self.create_timer(0.011, self.on_timer)


    def declare_param(self):
        #PID
        self.declare_parameter("Kp",15.0)
        self.Kp = self.get_parameter('Kp').get_parameter_value().integer_value
        self.declare_parameter("Ki",0)
        self.Ki = self.get_parameter('Ki').get_parameter_value().integer_value
        self.declare_parameter("Kd",0.1)
        self.Kd = self.get_parameter('Kd').get_parameter_value().integer_value
        # servo init
        self.init_servos1 = self.PWMServo_X = int(os.environ.get("INIT_SERVO_S1"))
        self.init_servos2 = self.PWMServo_Y = int(os.environ.get("INIT_SERVO_S2"))
        self.servo_angle = ServoControl()
        self.servo_angle.s1 = self.init_servos1
        self.servo_angle.s2 = self.init_servos2
        for i in range(3):
            self.pub_Servo.publish(self.servo_angle)
            time.sleep(0.01)
    
    def get_param(self):
        self.Kd = self.get_parameter('Kd').get_parameter_value().double_value
        self.Ki = self.get_parameter('Ki').get_parameter_value().double_value
        self.Kp = self.get_parameter('Kp').get_parameter_value().double_value

        current_linear_PID = (self.Kp, self.Ki, self.Kd)
        if current_linear_PID != self.prev_linear_PID:
            self.PID_controller.update_params(
                [self.Kp / float(self.scale), self.Kp / float(self.scale)],
                [self.Ki / float(self.scale), self.Ki / float(self.scale)],
                [self.Kd / float(self.scale), self.Kd / float(self.scale)]
            )
            self.prev_linear_PID = current_linear_PID
    
    def on_timer(self):
        self.get_param()
        ret, frame = self.capture.read()
        action = cv.waitKey(10) & 0xFF
        frame, binary =self.process(frame, action)
        start = time.time()
        fps = 1 / (start - self.end)
        text_fps = "FPS : " + str(int(fps))
        text = "x:" + str(self.point_pose[0]) + " y:" + str(self.point_pose[1])
        self.end = start
        cv.putText(frame, text_fps, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
        cv.putText(frame, text, (20, 55), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)
        if len(binary) != 0: cv.imshow('frame', ManyImgs(1, ([frame, binary])))
        else:cv.imshow('frame', frame)
        if action == ord('q') or action == 113:
            self.capture.release()
            cv.destroyAllWindows()

    def Reset(self):
        self.circle = (0, 0, 0)
        self.Mouse_XY = (0, 0)
        self.Track_state = 'init'
        self.PWMServo_X = 95
        self.PWMServo_Y = 35
        self.servo_angle = ServoControl()
        self.servo_angle.s1 = self.init_servos1
        self.servo_angle.s2 = self.init_servos2
        for _ in range(3):
            self.pub_position.publish(Position())
            self.pub_Servo.publish(self.servo_angle)
    
    def cancel(self):
        cmd1 = "ros2 topic pub -1 /Servo yahboomcar_msgs/msg/ServoControl "
        cmd2 = f'''"{{'s1': {self.init_servos1}, 's2': {self.init_servos2}}}"'''
        cmd = cmd1 +cmd2
        os.system(cmd)
        print("Shutting down this node.")
        cv.destroyAllWindows()

    def execute(self, point_x, point_y):
        position = Position()
        position.anglex = point_x * 1.0
        position.angley = point_y * 1.0
        position.distance = 0.0
        self.pub_position.publish(position)
        # rospy.loginfo("point_x: {}, point_y: {}".format(point_x, point_y))
        [x_Pid, y_Pid] = self.PID_controller.update([point_x - 320, point_y - 240])
        #print(f"x_Pid {x_Pid}, y_Pid {y_Pid}")
        x_Pid = x_Pid * (abs(x_Pid) <=self.Kp/3)
        y_Pid = y_Pid * (abs(y_Pid) <=self.Kp/3)
        if self.img_flip == True:
            self.PWMServo_X -= x_Pid
            self.PWMServo_Y -= y_Pid
        else:
            self.PWMServo_X  += x_Pid
            self.PWMServo_Y  += y_Pid

        self.PWMServo_X = int(max(0, min(180, self.PWMServo_X)))
        self.PWMServo_Y = int(max(0, min(100, self.PWMServo_Y)))
        self.servo_angle.s1 = self.PWMServo_X
        self.servo_angle.s2 = self.PWMServo_Y
        self.pub_Servo.publish(self.servo_angle)

    def PID_init(self):
        self.PID_controller = simplePID(
            [0, 0],
            [self.Kp / float(self.scale), self.Kp / float(self.scale)],
            [self.Ki / float(self.scale), self.Ki / float(self.scale)],
            [self.Kd / float(self.scale), self.Kd / float(self.scale)])

    def onMouse(self, event, x, y, flags, param):
        if event == 1:
            self.Track_state = 'init'
            self.select_flags = True
            self.Mouse_XY = (x,y)
        if event == 4:
            self.select_flags = False
            self.Track_state = 'identify'
        if self.select_flags == True:
            self.cols = min(self.Mouse_XY[0], x), min(self.Mouse_XY[1], y)
            self.rows = max(self.Mouse_XY[0], x), max(self.Mouse_XY[1], y)
            self.Roi_init = (self.cols[0], self.cols[1], self.rows[0], self.rows[1])

    def process(self, rgb_img, action):
        #rgb_img = cv.resize(rgb_img, (640, 480))
        binary = []
        if self.img_flip == True: rgb_img = cv.flip(rgb_img, 1)
        if action == 32: 
            self.Track_state = 'tracking' 
            print('tracking!!!')
        elif action == ord('i') or action == 105: self.Track_state = "identify"
        elif action == ord('r') or action == 114: self.Reset()
        elif action == ord('q') or action == 113: self.cancel()
        if self.Track_state == 'init':
            cv.namedWindow(self.windows_name, cv.WINDOW_AUTOSIZE)
            cv.setMouseCallback(self.windows_name, self.onMouse, 0)
            if self.select_flags == True:
                cv.line(rgb_img, self.cols, self.rows, (255, 0, 0), 2)
                cv.rectangle(rgb_img, self.cols, self.rows, (0, 255, 0), 2)
                if self.Roi_init[0] != self.Roi_init[2] and self.Roi_init[1] != self.Roi_init[3]:
                    self.gTracker_state = True
                    self.dyn_update = True
                else: self.Track_state = 'init'
        if self.Track_state != 'init':
            if self.tracker_type in self.tracker_types:
                if self.gTracker_state == True:
                    Roi = (self.Roi_init[0], self.Roi_init[1], self.Roi_init[2] - self.Roi_init[0], self.Roi_init[3] - self.Roi_init[1])
                    self.gTracker = Tracker(tracker_type=self.tracker_type)
                    self.gTracker.initWorking(rgb_img, Roi)
                    self.gTracker_state = False
                rgb_img, (targBegin_x, targBegin_y), (targEnd_x, targEnd_y) = self.gTracker.track(rgb_img)
                center_x = targEnd_x / 2 + targBegin_x / 2
                center_y = targEnd_y / 2 + targBegin_y / 2
                width = targEnd_x - targBegin_x
                high = targEnd_y - targBegin_y
                self.point_pose = (center_x, center_y, min(width, high))
        if self.Track_state == 'tracking':
            if self.circle[2] != 0: threading.Thread(target=self.execute, args=(self.circle[0], self.circle[1])).start()
            if self.point_pose[0] != 0 and self.point_pose[1] != 0: threading.Thread(target=self.execute, args=(self.point_pose[0], self.point_pose[1])).start()
        if self.tracker_type in self.tracker_types: cv.putText(rgb_img, " Tracker", (260, 20), cv.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2)
        return rgb_img, binary

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
       # rospy.loginfo('P:{}, I:{}, D:{}'.format(P, I, D))
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
    mono_tracker = mono_Tracker("MonoIdentify")
    try:
        rclpy.spin(mono_tracker)
    except KeyboardInterrupt:
        pass
    finally:
        mono_tracker.cancel()
        mono_tracker.destroy_node()

