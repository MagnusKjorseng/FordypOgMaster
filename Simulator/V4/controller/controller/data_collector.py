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

        self.usv_target = [0.,0.,0.] #meters
        self.usv_heading_target = 0 #degrees
        self.rov_target = [0.,0.,10.] #meters
        self.scenario = "Stationary"
        self.wave_height = "0m"



        self.delta_time = 1/100 #time between simulation steps

        self.plot_time = 30
        self.plot_timer = self.create_timer(self.plot_time, self.plot)

        self.sample_time = np.arange(0, self.plot_time, self.delta_time) #total time of sampling
        self.sample_time = self.sample_time[:-1]

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
        controlled = "controlled"

        usv_poses = np.array(self.usv_poses)
        usv_positions = usv_poses[:, 0]
        usv_orientations = usv_poses[:,1]

        usv_position_errors = self.usv_target - usv_positions
        # usv_position_mean_error = usv_position_errors.mean(axis=0)

        usv_pos, usv_pos_ax = plt.subplots()
        usv_pos_ax.plot(usv_positions[:,0], usv_positions[:,1], "-")
        usv_pos_ax.plot(usv_positions[0,0], usv_positions[0,1], "o")
        usv_pos_ax.plot(self.usv_target[0], self.usv_target[1], "x")
        usv_pos_ax.set_xlabel("X-position (m)")
        usv_pos_ax.set_ylabel("Y-position (m)")
        plt.savefig(f"usv_position_{controlled}")
        # usv_pos_ax.set_title("Movement of USV")
        # print("usv_position")

        # usv_mean_err, usv_mean_err_ax = plt.subplots()

        # usv_mean_err_ax.plot(self.sample_time, usv_position_mean_error)
        # usv_mean_err_ax.set_xlabel("Time (s)")
        # usv_mean_err_ax.set_ylabel("Mean error (m)")

        usv_err, usv_err_ax = plt.subplots()
        usv_err_ax.plot(np.linspace(0,self.plot_time, len(usv_position_errors)), usv_position_errors[:,:2])
        usv_err_ax.set_xlabel("Time (s)")
        usv_err_ax.set_ylabel("Position error (m)")
        plt.savefig(f"usv_pos_error_{controlled}")
        # print("usv_error")


        usv_heading = usv_orientations[:,2]
        usv_heading_error = self.usv_heading_target - usv_heading - 180
        usv_heading_error = [heading + 360 if heading < -180 else heading for heading in usv_heading_error]
        usv_hd_err, usv_hd_err_ax = plt.subplots()
        usv_hd_err_ax.plot(np.linspace(0,self.plot_time, len(usv_heading_error)), usv_heading_error)
        usv_hd_err_ax.set_xlabel("Time (s)")
        usv_hd_err_ax.set_ylabel("Heading error (Degrees)")
        plt.savefig(f"usv_heading_error_{controlled}")
        # print("usv_heading")


        # plt.show()

        rov_poses = np.array(self.rov_poses)
        rov_positions = rov_poses[:,0,:]
        rov_orientations = rov_poses[:,1,:]

        rov_error = usv_positions - rov_positions
        rov_err, rov_err_ax = plt.subplots()
        rov_err_ax.plot(np.linspace(0,self.plot_time, len(rov_error)), rov_error[:,:2])
        rov_err_ax.set_xlabel("Time (s)")
        rov_err_ax.set_ylabel("Position error (m)")
        plt.savefig(f"rov_position_error_{controlled}")
        # print("rov_error")

        depth_error = self.rov_target[2] - rov_positions[:,2]
        depth_err, depth_err_ax = plt.subplots()
        depth_err_ax.plot(np.linspace(0,self.plot_time, len(depth_error)), depth_error)
        depth_err_ax.set_xlabel("Time (s)")
        depth_err_ax.set_ylabel("Depth error (m)")
        plt.savefig(f"rov_depth_error_{controlled}")

        # plt.show()
        # rov_mean_error = rov_error[:,:2].mean(axis=1)
        # rov_mean_err, rov_mean_err_ax = plt.subplots()
        # rov_mean_err_ax.plot(self.sample_time, rov_mean_err)
        # rov_mean_err_ax.set_xlabel("Time (s)")
        # rov_mean_err_ax.set_ylabel("Mean error (m)")



        usv_forces = np.array(self.usv_forces)
        usv_force = usv_forces[:,0,:]
        usv_torque = usv_forces[:,1,:]


        usv_forc, usv_forc_ax = plt.subplots()
        usv_forc_ax.plot(np.linspace(0,self.plot_time, len(usv_forces)),usv_force[:,:2]/1000)
        usv_forc_ax.set_ylabel("Controlled force (kN)")
        usv_forc_ax.set_xlabel("Time (s)")
        plt.savefig(f"usv_forces")
        # usv_forc_ax.set_title("Force from controller on USV")

        usv_torq, usv_torq_ax = plt.subplots()
        usv_torq_ax.plot(np.linspace(0,self.plot_time, len(usv_torque)),usv_torque[:,2]/1000)
        usv_torq_ax.set_xlabel("Time (s)")
        usv_torq_ax.set_ylabel("Controlled torque (kNm)")
        plt.savefig(f"usv_torque")

        ''''
        rov_forces = np.array(self.rov_forces)
        print(rov_forces.shape)
        rov_force = rov_forces[:,0,:]
        rov_torque = rov_forces[:,1,:]


        rov_forc, rov_forc_ax = plt.subplots()
        rov_forc_ax.plot(np.linspace(0,self.plot_time, len(rov_force)), rov_force[:,:2]/1000)
        rov_forc_ax.set_ylabel("Controlled force (kN)")
        rov_forc_ax.set_xlabel("Time (s)")
        print("rov_forces")
        '''
        # plt.show()
        # input("Data saved")

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
        self.usv_forces.append([force, torque])

    def rov_force_callback(self, msg):
        force, torque = self.force_callback(msg)
        self.rov_forces.append([force, torque])

    def crane_callback(self, msg):
        return




if __name__ == '__main__':
    main()
