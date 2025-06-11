import rclpy
from rclpy.node import Node
import std_msgs.msg as std_msgs
import geometry_msgs.msg as geo_msgs

import numpy as np

def main(args = None):
    rclpy.init(args=args)

    controller = CraneController()
    rclpy.spin(controller)

class CraneController(Node):
    def __init__(self):
        super().__init__("crane_controller")

        self.target_depth = -0


        self.rov_listener = self.create_subscription(geo_msgs.Pose,
                                                "rov_pose",
                                                self.depth_callback,
                                                5)
        self.crane_publisher = self.create_publisher(std_msgs.Float32,
                                                "winch_speed",
                                                5)
        self.desired_depth_listener = self.create_subscription(std_msgs.Float32,
                                                               "desired_rov_depth",
                                                               self.update_depth_callback,
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

    def update_depth_callback(self, msg):
        desired_depth = msg.data

        self.target_depth = desired_depth

if __name__ == '__main__':
    main()
