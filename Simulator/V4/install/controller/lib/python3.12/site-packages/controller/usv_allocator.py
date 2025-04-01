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
from interfaces.msg import ThrusterCommand
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
    def __init__(self, configfile):
        super().__init__("usv_allocator")

        self.thrusters = self.load_config(configfile)

        self.transform = np.array(self.find_transform(self.thrusters))
        #print(self.transform)
        #pseudoinverse to go from global to local frame
        self.inv_transform = np.linalg.pinv(self.transform)
        #print(self.inv_transform)

        self.force_subscriber = self.create_subscription(geo_msgs.Vector3,
                                                            "usv_desired_force_on_cog",
                                                            self.force_callback,
                                                            5)
        self.pubs = [self.create_publisher(ThrusterCommand,
                                           "{}_force".format(thruster),
                                           10)
                    for thruster in self.thrusters]

    def run(self):
        return

    #Desired force recieved from controller
    def force_callback(self, msg):
        X = msg.x
        Y = msg.y
        N = msg.z

        force = np.array([X,Y,N])
        #self.get_logger().info('Force calculated: %f, %f, %f' % (force[0], force[1], force[2]))

        tau = np.dot(self.inv_transform, force)
        #self.get_logger().info('Tau calculated: %f, %f, %f, %f' % (tau[0][0], tau[1], tau[2], tau[3]))


        alpha, rpm = self.parse_tau(tau)

        print(alpha, rpm)

        for i in range(len(self.pubs)):
            msg = ThrusterCommand()
            msg.rpm = rpm[i]
            msg.angle = alpha[i]
            self.pubs[i].publish(msg)


    def parse_tau(self, tau):
        # since tau is in the form [x_1, y_1, x_2, y_2, ...]
        # and we want the form [x_1, y_1], [x_2, y_2], ...
        # this makes a list of every two elements
        thrust = [tau[i*2:i*2+2] for i in range(len(tau)//2)]

        alpha = [np.arctan(command[0]/command[1]) for command in thrust]

        thrust = [np.sqrt(command[0]**2 + command[1]**2) for command in thrust]

        Kt = [thruster["Kt"]for thruster in self.thrusters]
        Dp = [thruster["Dp"]for thruster in self.thrusters]
        rpms = self.force_to_rpm(thrust, Kt, Dp)

        return alpha, rpms

    def force_to_rpm(self, thrust, Kt=0.5, Dp = 0.2, rho=1025):
        # Taken from propeller force calculation
        # T = Kt * rho * n^2 * Dp^4, where n is in rps
        rps = np.sqrt(thrust/(Kt*rho*Dp**4))
        rpm = rps * 60
        return rpm

    # Find thrusters from config file
    def load_config(self, filename):
        with open(filename) as f:
            config = yaml.safe_load(f)

        thrusters = config["thrusters"]
        return thrusters

    # Finds the transform matrix T used to convert from local to global frame
    def find_transform(self, thrusters):
        T = None

        for thruster in thrusters:
            if thrusters[thruster]["type"] == "azimuth":

                azi = thrusters[thruster]["position"]
                t = [[1,       0],
                 [0,       1],
                 [-azi[1], azi[0]]]
                if T == None:
                    T = t
                else:
                    T = np.concatenate((T,t),1)
            elif thruster[1] == "tunnel":
                print("tunnel thruster")

        return T

if __name__ == '__main__':
    main()
