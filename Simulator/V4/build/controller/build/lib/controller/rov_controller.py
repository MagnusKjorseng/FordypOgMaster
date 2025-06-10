import rclpy
from rclpy.node import Node
import std_msgs.msg as std_msgs
import geometry_msgs.msg as geo_msgs

import numpy as np

def main(args = None):
    rclpy.init(args=args)

    controller = RovController()
    rclpy.spin(controller)

class RovController(Node):
    def __init__(self, target_depth):
        super().__init__("rov_controller")

        self.target_depth = target_depth
        self.position = None
        self.rotation = None

        self.pose_listener = self.create_subscription(geo_msgs.Pose,
                                                "rov_pose",
                                                self.pose_callback,
                                                5)
        self.vel_listener = self.create_subscription(geo_msgs.Vector3,
                                                     "rov_velocity",
                                                     self.vel_callback,
                                                     5)
        self.desired_position_listener = self.create_subscription(geo_msgs.Pose,
                                                               "desired_rov_pose",
                                                               self.update_pose_callback,
                                                               5)
        self.rov_force_publisher = self.create_publisher(geo_msgs.Vector3,
                                                         "rov_desired_force_on_cog",
                                                         5)


    def pose_callback(self, msg):
        self.position = msg.position
        self.rotation = msg.orientation


    def control_position(self, current, target):
        kp, ki, kd = 5,0,0

        error = target - current

        controlled = kp * error

        return controlled

    def update_pose_callback(self, msg):
        desired_depth = msg.data

        self.target_depth = desired_depth

if __name__ == '__main__':
    main()
