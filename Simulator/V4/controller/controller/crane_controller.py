import rclpy
from rclpy.node import Node
import std_msgs.msg as std_msgs
import geo_msgs.msg as geo_msgs

import numpy as np

def main(args = None):
    rclpy.init(args=args)

    controller = CraneController()
    rclpy.spin(controller)

class CraneController(Node):
    def __init__(self):
        super().__init__("crane_controller")

        self.target_depth = 0

        self.rov_listener = self.create_subscription(geo_msgs.Pose,
                                                "rov_pose",
                                                self.depth_callback,
                                                5)
        self.crane_publisher = self.create_publisher(std_msgs.Float32,
                                                "winch_speed",
                                                5)

    def depth_callback(self, msg):
        depth = msg.position.z

        command = std_msgs.Float32()
        command.data = self.control_depth(depth, self.target_depth)
        self.crane_publisher.publish(command)


    def control_depth(self, current_depth, target_depth):
        kp, ki, kd = 5,0,0

        error = target_depth - current_depth

        controlled = kp * error

        return controlled

