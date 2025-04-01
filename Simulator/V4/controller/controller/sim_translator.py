#################################
##
## This is a translator node between the allocator and the simulator.
## This should only be used with the simulator
## It exists because AGX does not accept custom message types.
## In the physical implementation this is equivalent to a local thruster controller.
##
##
#################################


import numpy as np
import rclpy
from rclpy.node import Node
import geometry_msgs.msg as geometry_msgs
from interfaces.msg import ThrusterCommand

import os
import yaml
from ament_index_python.packages import get_package_share_directory


def main(args=None):
    rclpy.init(args=args)

    config = "controller"
    yaml_package_path = get_package_share_directory(config)

    translator = Translator(os.path.join(yaml_package_path, "config.yml"))

    rclpy.spin(translator)

class Translator(Node):
    def __init__(self, configfile):
        super().__init__("translator")

        self.thrusters = self.load_config(configfile)

        self.command_subscribers = [self.create_subscription(ThrusterCommand,
                                                             "{}_force".format(thruster),
                                                             self.callback,
                                                             5)
                                    for thruster in self.thrusters]

        self.force_publishers = [self.create_publisher(geometry_msgs.Vector3,
                                                       "sim_{}_force".format(thruster),
                                                       10)
                                for thruster in self.thrusters]


    def callback(self, msg):
        angle = msg.angle
        rpm = msg.rpm

        Kt = [thruster["Kt"]for thruster in self.thrusters]
        Dp = [thruster["Dp"]for thruster in self.thrusters]


        return

    # Find thrusters from config file
    def load_config(self, filename):
        with open(filename) as f:
            config = yaml.safe_load(f)

        thrusters = config["thrusters"]
        return thrusters

if __name__ == '__main__':
    main()

