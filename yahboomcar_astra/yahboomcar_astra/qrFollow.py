#ros lib
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from std_msgs.msg import Int32, Bool,UInt16
#common lib
import os
import threading
import math
from ament_index_python.packages import get_package_share_directory
from yahboomcar_astra.common.astra_common import *
from yahboomcar_msgs.msg import Position,ServoControl
print("import done")


class qrFollow(Node):
    def __init__(self,name):
        super().__init__(name)
        
        #create the publisher
        self.pub_Servo = self.create_publisher(ServoControl, 'Servo', 10)
        self.pub_cmdVel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_position = self.create_publisher(Position, "/Current_point", 10)
        self.servo_angle = ServoControl()
        #create the subscriber
        self.sub_JoyState = self.create_subscription(Bool,'/JoyState',  self.JoyStateCallback,1)
        self.bridge = CvBridge()

        self.Joy_active = False
        self.Robot_Run = False
        self.img_flip = False
        self.twist = Twist()

        self.end = 0
        self.encoding = ['8UC3']
        self.Center_x = self.Center_y = self.Center_r = 0
        self.prev_time = 0
        # pid init
        self.declare_param()
        self.PID_init()
        self.prev_main_PID = None
        self.prev_angular_PID = None
        self.get_param()
        
        self.capture = cv.VideoCapture(0, cv.CAP_V4L2)
        self.capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'YUYV'))
        self.capture.set(cv.CAP_PROP_AUTO_EXPOSURE, 1)
        self.capture.set(cv.CAP_PROP_EXPOSURE, 111.5)
        #self.capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'MJPG'))
        self.timer = self.create_timer(0.03, self.on_timer)
        print("init done")
    
    def declare_param(self):
        # limit
        self.declare_parameter("mintra_r",350.0)
        self.mintra_r = self.get_parameter('mintra_r').get_parameter_value().double_value
        self.declare_parameter("tolerance_x",5.0)
        self.tolerance_x = self.get_parameter('tolerance_x').get_parameter_value().double_value
        self.declare_parameter("tolerance_r",65.0)
        self.tolerance_r = self.get_parameter('tolerance_r').get_parameter_value().double_value
        
        # pid init
        self.declare_parameter("linear_Kp",1.5)
        self.linear_Kp = self.get_parameter('linear_Kp').get_parameter_value().double_value
        self.declare_parameter("linear_Ki",0.0)
        self.linear_Ki = self.get_parameter('linear_Ki').get_parameter_value().double_value
        self.declare_parameter("linear_Kd",0.1)
        self.linear_Kd = self.get_parameter('linear_Kd').get_parameter_value().double_value

        self.declare_parameter("angular_Kp",35.0)
        self.angular_Kp = self.get_parameter('angular_Kp').get_parameter_value().double_value
        self.declare_parameter("angular_Ki",0.0)
        self.angular_Ki = self.get_parameter('angular_Ki').get_parameter_value().double_value
        self.declare_parameter("angular_Kd",0.5)
        self.angular_Kd = self.get_parameter('angular_Kd').get_parameter_value().double_value

        self.scale = 1000
        # servo init
        self.declare_parameter("Servo_Kp",28.0)
        self.Servo_Kp = self.get_parameter('Servo_Kp').get_parameter_value().double_value
        self.declare_parameter("Servo_Ki",0.0)
        self.Servo_Ki = self.get_parameter('Servo_Ki').get_parameter_value().double_value
        self.declare_parameter("Servo_Kd",3.0)
        self.Servo_Kd = self.get_parameter('Servo_Kd').get_parameter_value().double_value
        
        self.init_servos1 = self.PWMServo_X = int(os.environ.get("INIT_SERVO_S1"))
        self.init_servos2 = self.PWMServo_Y = int(os.environ.get("INIT_SERVO_S2"))
        self.servo_angle = ServoControl()
        self.servo_angle.s1 = self.init_servos1
        self.servo_angle.s2 = self.init_servos2
        for i in range(3):
            self.pub_Servo.publish(self.servo_angle)
            time.sleep(0.01)

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

    def on_timer(self):
        self.get_param()

        ret, frame = self.capture.read()
        action = cv.waitKey(10) & 0xFF
        start = time.time()
        fps = 1 / (start - self.end)
        text_fps = "FPS : " + str(int(fps))
        self.end = start
        cv.putText(frame, text_fps, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
        processed_frame, (x, y, w, h) = detect_qrcode(frame)
        if processed_frame is not None: 
            frame = processed_frame 
            self.Center_x = x + w // 2
            self.Center_y = y + h // 2
            self.Center_r = x * w // 100
            cv.circle(frame, (self.Center_x, self.Center_y), 5, (0, 255, 0), -1) 
            text = "x:" + str(int(self.Center_x)) + " y:" + str(int(self.Center_y))+ " area:" + str(int(self.Center_r))
            cv.putText(frame, text, (20, 55), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)
            threading.Thread(target=self.execute, args=(self.Center_x, self.Center_y,self.Center_r)).start()
        else:
            now_time = time.time()
            if now_time - self.prev_time > 0.5:
                self.prev_time = now_time
                self.execute(320, 240,self.mintra_r)

        cv.imshow('frame', frame)
        if action == ord('q') or action == 113:
            self.cancel()


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
            
    def JoyStateCallback(self, msg):
        if not isinstance(msg, Bool): return
        self.Joy_active = msg.data
        self.servo_angle.s1 = self.PWMServo_X = self.init_servos1
        self.servo_angle.s2 = self.PWMServo_Y = self.init_servos2
        self.pub_cmdVel.publish(Twist())

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
        linear_Pid = max(-0.55, min(linear_Pid, 0.28)) 
        angular_Pid = max(-3.0, min(angular_Pid, 3.0))
        linear_Pid = abs(linear_Pid) if (z - self.mintra_r > 0) else -abs(linear_Pid)
        angular_Pid = -abs(angular_Pid) if (self.PWMServo_X - self.init_servos1) > 0 else abs(angular_Pid)
        Servox_Pid = Servox_Pid * (abs(Servox_Pid) <=self.Servo_Kp/5.6)
        Servoy_Pid = Servoy_Pid * (abs(Servoy_Pid) <=self.Servo_Kp/5.6)
        #print(f"linear_Pid {linear_Pid}, angular_Pid {angular_Pid}, Servoy_Pid {Servoy_Pid}")
        if abs(self.PWMServo_X - self.init_servos1) < self.tolerance_x: angular_Pid = 0.0
        if z < self.mintra_r and abs(z - self.mintra_r) < self.tolerance_r: linear_Pid = 0.0
        elif z > self.mintra_r and abs(z - self.mintra_r) < self.tolerance_r/2.5: linear_Pid = 0.0
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
        #rospy.loginfo('P:{}, I:{}, D:{}'.format(P, I, D))
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
    qr_Follow = qrFollow("QrFollow")
    try:
        rclpy.spin(qr_Follow)
    except KeyboardInterrupt:
        pass
    finally:
        qr_Follow.cancel()
        qr_Follow.destroy_node()