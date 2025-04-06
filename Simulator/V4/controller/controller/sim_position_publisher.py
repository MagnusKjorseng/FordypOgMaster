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
from interfaces.msg import ThrusterCommand

import os
import yaml
from ament_index_python.packages import get_package_share_directory


def main(args=None):
    rclpy.init(args=args)

    rclpy.spin()

class Translator(Node):
    def __init__(self):


if __name__ == '__main__':
    main()

