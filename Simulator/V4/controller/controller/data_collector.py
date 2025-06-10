import rclpy
from rclpy.node import Node
import std_msgs.msg as std_msgs
import geometry_msgs.msg as geo_msgs

import numpy as np
from scipy.spatial.transform import Rotation as R

import matplotlib.pyplot as plt

def main(args = None):
    rclpy.init(args = args)

    collector = DataCollector()
    rclpy.spin(collector)

class DataCollector(Node):
    def __init__(self):
        super().__init__("data_collector")

        self.usv_target = [0.,0.,0.]
        self.rov_target = [0.,0.,-50.]
        self.scenario = "Stationary"
        self.wave_height = "0m"

        self.plot_time = 2
        self.plot_timer = self.create_timer(self.plot_time, self.plot)

        self.usv_poses = []
        self.usv_pose_listener = self.create_subscription(geo_msgs.Pose,
                                                      "usv_pose",
                                                      self.usv_position_callback,
                                                      5)

        self.rov_poses = []
        self.rov_pose_listener = self.create_subscription(geo_msgs.Pose,
                                                      "rov_pose",
                                                      self.rov_position_callback,
                                                      5)

        self.usv_forces = []
        self.usv_force_listener = self.create_subscription(geo_msgs.Wrench,
                                                           "usv_desired_force_on_cog",
                                                           self.usv_force_callback,
                                                           5)
        self.rov_forces = []
        self.rov_force_listener = self.create_subscription(geo_msgs.Wrench,
                                                           "rov_desired_force_on_cog",
                                                           self.rov_force_callback,
                                                           5)
        self.crane_responses = []
        self.crane_listener  = self.create_subscription(std_msgs.Float32,
                                                        "winch_speed",
                                                        self.crane_callback,
                                                        5)


    def plot(self):
        usv_poses = np.array(self.usv_poses)
        usv_positions = usv_poses[:, 0]
        usv_orientations = usv_poses[:,1]

        usv_position_errors = self.usv_target - usv_positions
        usv_position_mse = np.square(usv_position_errors).mean(axis=1)

        usv_pos, usv_pos_ax = plt.subplots()
        usv_pos_ax.plot(usv_positions[:,0], usv_positions[:,1])
        usv_pos_ax.set_xlabel("X-position (m)")
        usv_pos_ax.set_ylabel("Y-position (m)")
        usv_pos_ax.set_title("Movement of USV")


        plt.show()

        plt.plot(usv_position_errors[:,:2])
        # plt.show()

        rov_poses = np.array(self.rov_poses)
        rov_positions = rov_poses[0,:]
        rov_orientations = rov_poses[1,:]

        usv_forces = np.array(self.usv_forces)
        usv_force = usv_forces[0,:]
        usv_torque = usv_forces[1,:]

        rov_forces = np.array(self.rov_forces)
        rov_force = rov_forces[0,:]
        rov_torque = rov_forces[1,:]






    def position_callback(self, msg):
        position = msg.position
        position = [position.x, position.y, position.z]
        orientation = msg.orientation
        orientation = [orientation.x, orientation.y, orientation.z, orientation.w]
        r = R.from_quat(orientation)
        orientation = r.as_euler("xyz", degrees = True)

        return position, orientation

    def usv_position_callback(self,msg):
        position, orientation = self.position_callback(msg)
        self.usv_poses.append([position, orientation])

    def rov_position_callback(self, msg):
        position, orientation = self.position_callback(msg)
        self.rov_poses.append([position, orientation])

    def force_callback(self, msg):
        force = msg.force
        torque = msg.torque
        force = [force.x, force.y, force.z]
        torque = [torque.x, torque.y, torque.z]

        return force, torque

    def usv_force_callback(self, msg):
        force, torque = self.force_callback(msg)
        usv_forces.append([force, torque])

    def rov_force_callback(self, msg):
        force, torque = self.force_callback(msg)
        rov_forces.append([force, torque])

    def crane_callback(self, msg):
        return



if __name__ == '__main__':
    main()
