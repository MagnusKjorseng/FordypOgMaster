import rclpy
from rclpy.node import Node
import geometry_msgs.msg as geo_msgs # Wrench, Vector3 Pose

import numpy as np
#import matplotlib.pyplot as plt


def main(args = None):
    rclpy.init(args = args)

    m_usv=2384 #kg
    omega = 0.45
    zeta = 1.2
    
    
    kp = m_usv*omega**2
    ki = 0
    kd = 2*m_usv*zeta*omega
    targets = np.array([[10,15,0],[50, -30, 0],[-20, 10, 0],[0,0,0]])
    
    controller = UsvController(kp, ki, kd, targets = targets)

    rclpy.spin(controller)


class UsvController(Node):
    def __init__(self, Kp, Ki, Kd, authority=2e4, torque = 2e5, targets = np.array([[0,0,0]])):
        super().__init__("usv_controller")
        
        self.kp = Kp
        self.ki = Ki
        self.kd = Kd
        self.last = 0
        self.integral = np.array([0,0,0]) #for integral part
        self.steps = 0
        
        self.position = np.array([0,0,0])
        self.rotation = np.array([0,0,0,0]) #quaternion for rotation
        self.velocity = np.array([0,0,0])

        self.heading_desired = np.zeros(4) #desired rotation as quaternion
        self.heading_set = False
        self.has_plotted = False
        self.lastmag = 0

        self.authority = authority
        self.torque = torque
        
        self.targets = targets
        self.current_target = 0

        self.errors = []
        

        
        self.connection_active = False

        #self.rotation_publisher = self.create_publisher()

        # self.desired_force_publisher = self.create_publisher(Wrench, "usv_desired_force_on_cog", 10)
        self.desired_force_publisher = self.create_publisher(geo_msgs.Vector3,
                                                             "usv_desired_force_on_cog",
                                                             10)
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.run)

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

        # self.pose = agxROS2.GeometryMsgsPose()
        # self.pose_sub = agxROS2.SubscriberGeometryMsgsPose("usv_pose")


    #Actual control loop
    def run(self):
        #self.updateSubscriptions()
        
#         if not self.connection_active:
#             print("No connection detected")
#             return
#
#         if not self.heading_set:
#             goal = self.targets[self.current_target]
#             direction = goal - self.position
#             direction.normalize()
#
#             angle = np.arcsin(direction[0])
#             self.heading_desired = angle
#
        #rotation = self.controlHeading(self.frame, self.heading_desired, self.torque)
        #msg = agxROS2.StdMsgsFloat32()
        #msg.data = command
        #self.rotation_publisher.sendMessage(msg)
        
        
        controlled, error = self.controlForce(self.position, self.velocity, self.targets[self.current_target])
        
        #TODO: Change from vector to wrench, include torque
        force = geo_msgs.Vector3()
        force.x = controlled[0]
        force.y = controlled[1]
        force.z = controlled[2]


        self.desired_force_publisher.publish(force)
        print("Force published")
        self.errors.append(error)
        self.last = error
        
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
    
    def controlHeading(self, frame, desired_heading, authority):
        rot = frame.getLocalRotate().getAsEulerAngles()[2]#Rotation in local frame 
        
        error_heading = np.array([0,0, desired_heading] - rot)
        #print(error_heading)
        
        kp = 3000
        command = error_heading * kp #just proportional control
        self.clamp(command, authority)
        
        return command
        

    #Clamps variable to the range 
    #-clamp < variable < clamp
    def clamp(self, variable, clamp):
        length = sum(variable * variable) ** 0.5
        if  length > clamp:             #If the length of variable is larger than clamp
            vec = variable/length       #find the normalized vector
            return vec * clamp          #and return it scaled by clamp
        else:
            return variable
    #'''
    
    def checkNextTarget(self):
        avgLast50 = np.sum(self.errors[-50:], axis=0)/50
        
        if np.less(abs(avgLast50), 0.1).all():
            if self.current_target + 1 < len(self.targets):
                self.current_target += 1
                print("New Target")
                print(self.targets[self.current_target]) 
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
