#imports
import rclpy
from rclpy.node import Node
import math

from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Int32

class NavNode():
    def __init__(self):
        super().__init__("nav_node")

        self.instructions = []
        self.direction = ""
        self.instruction_timer_period = 0.6 #some arbitrary timing
        self.vision_timer_period = 0.6

        self.sub_to_tower = self.create_subscription(
            String,
            'THE TOPIC WE CHOOSE',
            self.instructions,
            10
        )

        self.sub_to_vision = self.create_subscription(
            String,
            "ANOTHA TOPIC",
            self.direction,
            10
        )

        self.line_follow_instructions = self.create_publisher(
            String,
            'DIRECTION TOPIC',
            self.direction,
            10
            )

        self.instruction_timer = self.create_timer(self.instruction_timer_period, self.instruction_callback)
        self.vision_timer = self.create_timer(self.vision_timer_period, self.vision_callback)

    def instruction_callback(self, ):

    def vision_callback(self, ):

