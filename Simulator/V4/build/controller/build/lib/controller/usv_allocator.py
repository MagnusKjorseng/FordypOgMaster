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
import geometry_msgs.msg as geo_msgs
from ament_index_python.packages import get_package_share_directory
import os

import yaml




def main(args = None):
    rclpy.init(args=args)

    config = "controller"
    yaml_package_path = get_package_share_directory(config)

    allocator = Allocator(os.path.join(yaml_package_path, "config.yml"))

    rclpy.spin(allocator)

class Allocator(Node):
    def __init__(self, config):
        super().__init__("usv_allocator")

        self.thrusters = self.load_config(config)

        self.transform = np.array(self.find_transform(self.thrusters))
        #pseudoinverse to go from global to local frame
        self.inv_transform = np.linalg.pinv(self.transform)

        self.force_subscriber = self.create_subscription(geo_msgs.Vector3,
                                                            "usv_desired_force_on_cog",
                                                            self.force_callback,
                                                            5)
        publishers = []

        for thruster in self.thrusters:
            topic = "{}_force".format(thruster[0])
            publishers.append(self.create_publisher(geo_msgs.Vector3,
                                                    topic,
                                                    10))
        self.pubs = publishers

    def run(self):
        return

    #Desired force recieved from controller
    def force_callback(self, msg):
        X = msg.x
        Y = msg.y
        N = msg.z

        force = np.array([X,Y,N]).T
        #self.get_logger().info('Force calculated: %f, %f, %f' % (force[0], force[1], force[2]))

        tau = self.inv_transform * force
        #self.get_logger().info('Tau calculated: %f, %f, %f, %f' % (tau[0][0], tau[1], tau[2], tau[3]))
        print(tau)
        '''
        self.parse_tau(tau)
        # unwrap tau into individual thrusters and publish each
        '''

    def parse_tau(self, tau):
        alpha = np.atan(tau[0],tau[1])
        thrust = np.sqrt(tau[0]**2 + tau[1]**2)

        return alpha, thrust


    # Find thrusters from config file
    def load_config(self, filename):
        with open(filename) as f:
            config = yaml.safe_load(f)

        data = config["thrusters"]

        thrusters = []

        for thruster in data:
            thrusters.append([thruster,
                             data[thruster]["type"],
                             data[thruster]["position"]])
        print(thrusters)
        return thrusters

    # Finds the transform matrix T used to convert from local to global frame
    def find_transform(self, thrusters):
        T = []

        for thruster in thrusters:
            if thruster[1] == "azimuth":
                azi = thruster[2]
                t = [[1,       0],
                 [0,       1],
                 [-azi[1], azi[0]]]
                T.append(t)
            elif thruster[1] == "tunnel":
                print("tunnel thruster")

        return T

if __name__ == '__main__':
    main()
