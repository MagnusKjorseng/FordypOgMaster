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
import geometry_msgs.msg as geo_msgs
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
        super().__init__("thrust_translator")

        self.thrusters = self.load_config(configfile)

        self.command_subscribers = [self.create_subscription(ThrusterCommand,
                                                             "thruster1_force",
                                                             self.callback1,
                                                             5),
                                    self.create_subscription(ThrusterCommand,
                                                             "thruster2_force",
                                                             self.callback2,
                                                             5)]

        self.force_publishers = [self.create_publisher(geo_msgs.Vector3,
                                                       "sim_{}_force".format(thruster),
                                                       10)
                                for thruster in self.thrusters]

    #Couldn't think of a better way to combine these two, so I've implemented them as two identical methods.
    def callback1(self, msg):
        angle = msg.angle
        rpm = msg.rpm
        thruster = "thruster1"
        Kt = self.thrusters[thruster]["Kt"]
        Dp = self.thrusters[thruster]["Dp"]
        rho = 1025

        force = self.force_calculator(angle, rpm, Kt, Dp, rho)
        self.send_message(force, 0)

    def callback2(self, msg):
        angle = msg.angle
        rpm = msg.rpm
        thruster = "thruster2"
        Kt = self.thrusters[thruster]["Kt"]
        Dp = self.thrusters[thruster]["Dp"]
        rho = 1025

        force = self.force_calculator(angle, rpm, Kt, Dp, rho)
        self.send_message(force, 1)

    def force_calculator(self, angle, rpm, Kt, Dp, rho):
        rps = rpm / 60
        thrust = Kt * rho * rps**2 * Dp**4

        x_force = thrust * np.sin(angle)
        y_force = thrust * np.cos(angle)

        return [x_force, y_force]

    def send_message(self, message, index):
        publisher = self.force_publishers[index]

        msg = geo_msgs.Vector3()
        msg.x = float(message[0])
        msg.y = float(message[1])
        msg.z = 0.0
        #self.get_logger().info(msg.x)

        publisher.publish(msg)
        #self.get_logger().info("Message sent: x: %f, y: %f" %(msg.x, msg.y))

    # Find thrusters from config file
    def load_config(self, filename):
        with open(filename) as f:
            config = yaml.safe_load(f)

        thrusters = config["thrusters"]
        return thrusters

if __name__ == '__main__':
    main()

