import rclpy
from rclpy.node import Node
import std_msgs.msg as std_msgs
import geometry_msgs.msg as geo_msgs

import numpy as np
from scipy.spatial.transform import Rotation as R

def main(args = None):
    rclpy.init(args=args)

    controller = RovController()
    rclpy.spin(controller)

class RovController(Node):
    def __init__(self):
        super().__init__("rov_controller")

        self.target_depth = -0
        self.target_position = np.array([0.,0.,0.])
        self.target_rotation = np.array([0.,0.,0.])

        self.current_position = np.array([0.,0.,0.])
        self.current_rotation = np.array([0.,0.,0.])
        self.current_velocity = np.array([0.,0.,0.])

        self.usv_position = np.array([0.,0.,0.])
        self.is_master = False

        self.pose_listener = self.create_subscription(geo_msgs.Pose,
                                                "rov_pose",
                                                self.pose_callback,
                                                5)
        self.vel_listener = self.create_subscription(geo_msgs.Vector3,
                                                     "rov_velocity",
                                                     self.vel_callback,
                                                     5)
        self.desired_position_listener = self.create_subscription(geo_msgs.Pose,
                                                               "desired_rov_pose",
                                                               self.update_target_pose_callback,
                                                               5)
        self.desired_depth_listener = self.create_subscription(std_msgs.Float32,
                                                               "desired_rov_depth",
                                                               self.depth_callback,
                                                               5)
        self.rov_force_publisher = self.create_publisher(geo_msgs.Vector3,
                                                         "rov_desired_force_on_cog",
                                                         5)
        self.usv_pose_subscriber = self.create_subscription(geo_msgs.Pose,
                                                            "usv_pose",
                                                            self.usv_callback,
                                                            5)

        self.delta_time = 0.01
        self.timer = self.create_timer(self.delta_time, self.control_position)




    def control_position(self):
        kp, ki, kd = 5,0,0

        #If ROV is master, it follows its own directives, otherwise follow USV
        if self.is_master:
            target = self.target_position
            target[2] = self.target_depth
        else:
            target = self.usv_position
            target[2] = self.target_depth

        error = target - self.current_position

        controlled = kp * error

        msg = geo_msgs.Vector3()
        msg.x = controlled[0]
        msg.y = controlled[1]
        msg.z = controlled[2]

        self.rov_force_publisher.publish(msg)

    def pose_callback(self, msg):
        position = msg.position
        position = np.array([position.x, position.y, position.z])
        self.current_position = position

        rotation = msg.orientation
        rotation = np.array([rotation.x, rotation.y, rotation.z, rotation.w])
        r = R.from_quat(rotation).as_euler("xyz")
        self.current_rotation = r

    def vel_callback(self, msg):
        self.current_velocity = np.array([msg.x, msg.y, msg.z])

    def update_target_pose_callback(self, msg):
        position = msg.position
        position = np.array([position.x, position.y, position.z])
        self.target_position = position


        rotation = msg.orientation
        rotation = np.array([rotation.x, rotation.y, rotation.z, rotation.w])
        r = R.from_quat(rotation).as_euler("xyz")
        self.target_rotation = r

    def depth_callback(self,msg):
        self.target_depth = msg.data

    def usv_callback(self, msg):
        position = msg.position
        position = np.array([position.x, position.y, position.z])
        self.usv_position = position


if __name__ == '__main__':
    main()
