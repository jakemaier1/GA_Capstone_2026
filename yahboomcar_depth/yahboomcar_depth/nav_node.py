#imports
import rclpy
from rclpy.node import Node
import math

from enum import Enum, auto
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Int32
from std_msgs.msg import StringArray
from dataclasses import dataclass


@dataclass
class Sign:
    sign_type: str
    sign_label: str
    sign_direction: str

    @classmethod
    def from_string(cls, name_string):
        sign_type, sign_label, sign_direction = (
            name_string.lower().split("_", 2)
        )

        return cls(
            sign_type=sign_type,
            sign_label=sign_label,
            sign_direction=sign_direction
        )


class NavNode(Node):
    def __init__(self):
        super().__init__("nav_node")

        self.instructions = []
        self.options = []
        self.available_signs = []
        self.next_sign = None


        self.sub_to_tower = self.create_subscription(
            StringArray,
            'InstructionTopic',
            self.tower_callback,
            10
        )

        self.sub_to_CV = self.create_subscription(
            StringArray,
            "ANOTHA TOPIC",
            self.CV_callback,
            10
        )

        self.sub_to_line_follow = self.create_subscription(
            StringArray,
            "/turn_options",
            self.line_follow_callback,
            10
        )

        self.line_follow_instructions = self.create_publisher(
            String,
            'DIRECTION TOPIC',
            10
            )

        self.create_timer(0.1, self.BDT)  #timer to make FSM run this often

      
    #This is literally to grab the instructions from the tower and put them into a global var
    def tower_callback(self, msg):
        self.instructions_raw = list(msg.data) #list() is used so that this file has a separate copy to change however
        self.instructions = [Sign.from_string(instruction) for instruction in self.instructions_raw]
        self.next_sign = self.instructions[0]

    def CV_callback(self, msg):
        self.available_signs = list(msg.data)

    def line_follow_callback(self, msg):
        self.options = list(msg.data)

    def BDT(self):
        direction_msg = String()

        if not self.instructions:
            return

        available_sign_objects = [Sign.from_string(sign) for sign in self.available_signs]
        available_turn_options = [option.lower() for option in self.options]
            
        #TODO: Implement stop logic after we design stopping signs etc

        target_sign_is_visible = False

        for available_sign in available_sign_objects:
            if (available_sign.sign_type == self.next_sign.sign_type
                    and available_sign.sign_label == self.next_sign.sign_label):
                target_sign_is_visible = True
                break 

        if not target_sign_is_visible:
            direction_msg.data = "straight"

        elif self.next_sign.sign_direction in available_turn_options:
            direction_msg.data = self.next_sign.sign_direction
            self.instructions.pop(0)
            if self.instructions:
                self.next_sign = self.instructions[0]
            else:
                self.next_sign = None

        else:
            direction_msg.data = "straight"

        self.line_follow_instructions.publish(direction_msg)

                
