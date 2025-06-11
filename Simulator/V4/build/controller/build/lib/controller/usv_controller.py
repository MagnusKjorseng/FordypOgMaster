import rclpy
from rclpy.node import Node
import geometry_msgs.msg as geo_msgs # Wrench, Vector3 Pose
import std_msgs.msg as std_msgs

import numpy as np
#import matplotlib.pyplot as plt


def main(args = None):
    rclpy.init(args = args)

    m_usv=2384 #kg
    omega = 0.5
    zeta = 1.3
    
    
    kp = m_usv*omega**2
    ki = 0
    kd = 2*m_usv*zeta*omega
    #targets = np.array([[10,15,0],[50, -30, 0],[-20, 10, 0],[0,0,0]])
    
    controller = UsvController(kp, ki, kd)#, targets=[[10,30,0]])#, targets)

    rclpy.spin(controller)


class UsvController(Node):
    def __init__(self, Kp, Ki, Kd, authority=2e4, torque = 2e5, targets = np.array([[0,0,0]]), target_heading = None):
        super().__init__("usv_controller")

        self.is_master = True
        
        self.kp = Kp
        self.ki = Ki
        self.kd = Kd
        self.last = 0
        self.integral = np.array([0.,0.,0.]) #for integral part
        self.steps = 0
        
        self.position = np.array([0,0,0])
        self.rotation = np.array([0,0,0,0]) #quaternion for rotation

        self.velocity = np.array([0,0,0])

        self.rov_position = np.array([0,0,0])

        self.heading = 0 #heading in degrees from north
        self.last_heading = 0
        if target_heading is None:
            self.desired_heading = 0 #in degrees from north
            #self.heading_desired = np.zeros(4) #desired rotation as quaternion
            self.heading_set_manually = False
        else:
            self.desired_heading = target_heading
            self.heading_set_manually = True
        self.has_plotted = False
        self.lastmag = 0

        self.authority = authority
        self.torque = torque
        
        self.targets = targets
        self.current_target_index  = 0
        self.current_target = np.array([0,0,0])

        self.errors = []
        

        
        self.connection_active = False

        #self.rotation_publisher = self.create_publisher()

        self.desired_force_publisher = self.create_publisher(geo_msgs.Wrench,
                                                             "usv_desired_force_on_cog",
                                                             5)
        # self.desired_force_publisher = self.create_publisher(geo_msgs.Vector3,
                                                             # "usv_desired_force_on_cog",
                                                             # 10)
        self.delta_time = 0.01
        self.timer = self.create_timer(self.delta_time, self.run)
        #TODO: Change this to be implemented in the callbacks, requires callbacks to be correct first though.

        '''
        self.rotation_msg = agxROS2.StdMsgsFloat32()
        self.rotation_publisher = agxROS2.PublisherStdMsgsFloat32("usv_rotation_command")
        '''

        #TODO: Change velocity to be a Twist, not a vector3
        self.velocity_subscriber = self.create_subscription(geo_msgs.Vector3,
                                                            "usv_velocity",
                                                            self.velocity_callback,
                                                            5)

        self.pose_subscriber     = self.create_subscription(geo_msgs.Pose,
                                                            "usv_pose",
                                                            self.pose_callback,
                                                            5)
        #Temporary subscriber because HDT messages are broken atm
        self.heading_subscriber  = self.create_subscription(std_msgs.Float32,
                                                            "temp_heading",
                                                            self.heading_callback,
                                                            5)

        self.rov_subscriber     = self.create_subscription(geo_msgs.Pose,
                                                           "rov_pose",
                                                           self.rov_callback,
                                                           5)

        # self.pose = agxROS2.GeometryMsgsPose()
        # self.pose_sub = agxROS2.SubscriberGeometryMsgsPose("usv_pose")


    #Actual control loop
    def run(self):
        if self.is_master:
            self.current_target = self.targets[self.current_target_index]
        else:
            self.current_target = self.rov_position

        controlled_force, force_error = self.controlForce(self.position, self.velocity, self.current_target)

        controlled_heading, heading_error = self.controlHeading()
        # self.get_logger().info(f"controlled_heading: {controlled_heading}")

        force = geo_msgs.Vector3()
        force.x = controlled_force[0]
        force.y = controlled_force[1]
        force.z = controlled_force[2]

        torque = geo_msgs.Vector3()
        torque.x = 0.
        torque.y = 0.
        torque.z = controlled_heading

        wrench = geo_msgs.Wrench()
        wrench.force = force
        wrench.torque = torque
        # self.get_logger().info(f"\nForce: {force} \nTorque: {torque}")# \nWrench: {wrench}")

        self.desired_force_publisher.publish(wrench)
        # self.get_logger().info("pubslished")
        self.errors.append(force_error)
        # self.get_logger().info("appended")
        self.last = force_error
        # self.get_logger().info("last")
        #checkNextTarget()

    def pose_callback(self, msg):
        x = msg.position.x
        y = msg.position.y
        z = msg.position.z

        a = msg.orientation.w
        i = msg.orientation.x
        j = msg.orientation.y
        k = msg.orientation.z

        self.position = np.array([x,y,z])
        self.rotation = np.array([a,i,j,k])

    def rov_callback(self, msg):
        position = msg.position
        position = np.array([position.x, position.y, position.z])
        self.rov_position = position

    def heading_callback(self, msg):
        self.heading = msg.data
    
    def velocity_callback(self, msg):
        self.velocity[0] = msg.x
        self.velocity[1] = msg.y
        self.velocity[2] = msg.z
        #self.get_logger().info('Velocity Received')
        
    def controlForce(self, position, velocity, target):
        
        error = target - position
        error[-1] = 0 #no force in Z-direction, no error in z-direction
        #TODO: implement force commands in heave-direction
        vel_error = np.array([0,0,0]) - velocity #zero speed is desired
        self.integral += error
        difference = error - self.last
        
        controlled = self.kp * error + self.ki * self.integral + self.kd * vel_error
        
        #Check that the controlled variable is not outside authority
        controlled = self.clamp(controlled, self.authority)
        
              
        return controlled, error
    
    def controlHeading(self, authority=1000):
        if not self.heading_set_manually:
            self.findHeading()
            # self.get_logger().info(f"Desired heading: {self.desired_heading}")

        error_heading = self.desired_heading - self.heading
        turning_rate = (self.heading - self.last_heading)/self.delta_time
        error_rate = 0 - turning_rate #desired turning rate is 0
        # self.get_logger().info(f"heading error: {error_heading}")
        
        #error_heading = np.array([0,0, desired_heading] - rot)
        
        kp = -15
        kd = -5
        command = error_heading * kp + error_rate * kd
        command = self.clamp(command, authority)
        
        self.last_heading = self.heading

        return command, error_heading

    #Finds the vector from the current point to the desired target and then finds the heading of that vector in world space
    def findHeading(self):
        current_pos = self.position
        target = self.targets[self.current_target_index]

        heading_vector = target - current_pos
        # self.get_logger().info(f"heading_vector: {heading_vector}")

        if heading_vector[1] == 0:
            angle_from_north = 0.
        else:
            angle_from_north = np.rad2deg(np.arctan(heading_vector[0]/heading_vector[1])) # The angle from north is atan x/y which provides clockwise rotation with 0 at north
        # self.get_logger().info(f"angle: {angle_from_north}")
        #correction to provide a continous range -0.5pi < a < 1.5pi
        if heading_vector[1] < 0:
            angle_from_north += 180

        self.desired_heading = angle_from_north

    #Clamps variable to the range 
    #-clamp < variable < clamp
    def clamp(self, variable, clamp):
        try:
            length = sum(variable * variable) ** 0.5
        except TypeError:
            length = np.sqrt(variable*variable)
        if  length > clamp:             #If the length of variable is larger than clamp
            vec = variable/length       #find the normalized vector
            return vec * clamp          #and return it scaled by clamp
        else:
            return variable
    #'''
    
    def checkNextTarget(self):
        avgLast50 = np.sum(self.errors[-50:], axis=0)/50
        
        if np.less(abs(avgLast50), 0.1).all():
            if self.current_target_index + 1 < len(self.targets):
                self.current_target_index += 1
                print("New Target")
                print(self.targets[self.current_target_index])
                self.heading_set = False
                input()
                
            #elif not self.has_plotted:
    
    # def plot(self):
    #     self.has_plotted = True
    #     print("End of targets, plotting...")
    #     #print(self.errors.shape())
    #     errors = np.array(self.errors)
    #     errors = np.sqrt(errors[:,0]**2 + errors[:,1]**2)
    #     print(errors.shape)
    #     plt.plot(abs(errors))
    #     plt.xlabel("Time(ms)")
    #     plt.ylabel("Positional error(m)")
    #     plt.title("Control system error with 4 set-points")
    #     plt.legend
    #     plt.show()
    #     input()
   
if __name__ == '__main__':
    main()
