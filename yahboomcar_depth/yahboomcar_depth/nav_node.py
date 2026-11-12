#imports
import rclpy
from rclpy.node import Node
import math

from enum import Enum, auto
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Int32

class State(Enum):
    READY = auto()
    DRIVE = auto()
    STOP = auto()


class NavNode():
    def __init__(self):
        super().__init__("nav_node")

        self.instructions = [A, ]
        self.options = []
        self.available_signs = []
        self.instruction_timer_period = 0.6 #some arbitrary timing
        self.vision_timer_period = 0.6

        self.sub_to_tower = self.create_subscription(
            String,
            'THE TOPIC WE CHOOSE',
            self.instructions,
            10
        )

        self.sub_to_CV = self.create_subscription(
            String,
            "ANOTHA TOPIC",
            self.available_signs,
            10
        )

        self.sub_to_line_follow = self.create_subscription(
            String,
            ""
        )

        self.line_follow_instructions = self.create_publisher(
            String,
            'DIRECTION TOPIC',
            self.direction,
            10
            )

        # self.instruction_timer = self.create_timer(self.instruction_timer_period, self.instruction_callback)
        # self.vision_timer = self.create_timer(self.vision_timer_period, self.vision_callback)

    def BDT(self, available_signs, instructions):
        #Check for stop first
        if("STOP" in available_signs){
            #figure out stop logic here
        }

        #start BDT
        if( instructions[0] in available_signs){
            
        }

    def FSM(self, ):

