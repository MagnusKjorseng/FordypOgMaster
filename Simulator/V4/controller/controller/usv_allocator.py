#################################################
#################################################
#################################################
#           ALLOCATOR
#################################################
#################################################
#################################################
import numpy as np
import rclpy
from rclpy.node import Node

import yaml




def main(args = None):
    rclpy.init(args=args)


class Allocator(Node):
    def __init__(self, config):
        super().__init__("usv_allocator")

        self.thrusters = self.load_config(config)

    # Find thrusters from config file
    def load_config(self, filename):
        with open(filename) as f:
            config = yaml.safe_load(f)

        thrusters = config["thrusters"]

        azimuths = []

        for thruster in thrusters:
            if thrusters[thruster]["type"] == "azimuth":
                azimuths.append(thrusters[thruster]["position"])

        return azimuths
