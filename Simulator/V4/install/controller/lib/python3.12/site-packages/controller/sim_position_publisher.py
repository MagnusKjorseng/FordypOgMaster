#################################
##
## This is a translator node between the simulator and the controller
## The purpose of this translator is to take the position from the sim,
## which is given in ROS2 Vector3's, and convert it to standard
## NMEA-GGA messages, similar to the ones that come from the GPS.
## The purpose of this node is to make the controller more agnostic
## This node should not be included and run in the proper implementation.
##
##
#################################


import numpy as np
import rclpy
from rclpy.node import Node
import geometry_msgs.msg as geo_msgs
import nmea_msgs.msg as nmea

import os
import yaml
from ament_index_python.packages import get_package_share_directory


def main(args=None):
    rclpy.init(args=args)

    #Since this is translating from a simulation based around origin (0,0,0) to real-world simulated coordinates, a baseline "starting point" is necessary. This is what (0,0,0) will be converted to.
    # I've chosen an arbitrary point outside Ålesund as my baseline.
    # The baseline is formatted as degreesminutes.decimals because this is the standard in GGA messages.
    baseline = ["6229.00", "N", "00606.00", "E"]

    position = Translator(baseline)

    rclpy.spin(position)

class Translator(Node):
    def __init__(self, baseline):
        super().__init__("position_translator")

        self.pose_subscriber = self.create_subscription(geo_msgs.Pose,
                                                        "usv_pose",
                                                        self.pose_callback,
                                                        5)
        self.baseline = baseline

    def pose_callback(self, msg):
        position = msg.position
        orientation = msg.orientation #NYI

        position = [position.x, position.y, position.z]



if __name__ == '__main__':
    main()

