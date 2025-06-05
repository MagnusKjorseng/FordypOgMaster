import rclpy
from rclpy.node import Node
import std_msgs.msg as std_msgs
import geo_msgs.msg as geo_msgs

import numpy as np

import matplotlib.pyplot as plt

def main(args = None):
    rclpy.init(args = args)

    collector = DataCollector()
    rclpy.spin(collector)

class DataCollector(Node):
    def __init__(self):
        super().__init__("data_collector")

        self.usv_pose_listener = self.create_subscription(geo_msgs.Pose,
                                                      "usv_pose",
                                                      self.position_callback,
                                                      5)


        self.rov_pose_listener = self.create_subscription(geo_msgs.Pose,
                                                      "rov_pose",
                                                      self.position_callback,
                                                      5)

    def position_callback(self, msg):
        position = msg.position
        position = [position.x, position.y, position.z]

if __name__ == '__main__':
    main()
