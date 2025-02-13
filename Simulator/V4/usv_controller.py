import agx
import agxSDK
import agxROS2
import numpy as np
import matplotlib.pyplot as plt
import time


def main():
    m_usv=2384 #kg
    omega = 0.45
    zeta = 1.2
    
    
    kp = m_usv*omega**2
    ki = 0
    kd = 2*m_usv*zeta*omega
    targets = [agx.Vec3(10,15,0), agx.Vec3(50, -30, 0), agx.Vec3(-20, 10, 0), agx.Vec3(0,0,0)]
    
    controller = UsvController(kp, ki, kd, targets = targets)
    
    while True:
        controller.run()
        time.sleep(0.1)


class UsvController():
    def __init__(self, Kp, Ki, Kd, authority=2e4, torque = 2e5, targets = [agx.Vec3(0,0,0)]):
        super().__init__()
        
        self.kp = Kp
        self.ki = Ki
        self.kd = Kd
        self.last = 0
        self.sum = agx.Vec3(0,0,0) #for integral part
        self.steps = 0
        
        self.position = agx.Vec3(0,0,0)
        self.rotation = agx.Quat(0,0,0,0)
        self.velocity = agx.Vec3(0,0,0)
        
        self.authority = authority
        self.torque = torque
        
        self.targets = targets
        self.current_target = 0

        self.errors = []
        
        self.heading_desired = agx.Quat()
        self.heading_set = False
        self.has_plotted = False
        self.lastmag = 0
        
        self.connection_active = False
        
        self.rotation_msg = agxROS2.StdMsgsFloat32()
        self.rotation_publisher = agxROS2.PublisherStdMsgsFloat32("usv_rotation_command")
        
        self.vel = agxROS2.GeometryMsgsVector3()
        self.vel_sub = agxROS2.SubscriberGeometryMsgsVector3("usv_velocity")
        
        self.vele = agxROS2.GeometryMsgsVector3()
        self.vele_pub = agxROS2.PublisherGeometryMsgsVector3("usv_desired_force_on_cog")
        
        self.pose = agxROS2.GeometryMsgsPose()
        self.pose_sub = agxROS2.SubscriberGeometryMsgsPose("usv_pose")
        

    #Actual control loop
    def run(self):
        self.updateSubscriptions()
        
        if not self.connection_active:
            print("No connection detected")
            return
        
        if not self.heading_set:
            goal = self.targets[self.current_target]
            direction = goal - self.position
            direction.normalize()
            
            angle = np.arcsin(direction[0])
            self.heading_desired = angle
           
        #rotation = self.controlHeading(self.frame, self.heading_desired, self.torque)
        #msg = agxROS2.StdMsgsFloat32()
        #msg.data = command
        #self.rotation_publisher.sendMessage(msg)
        
        
        force, error = self.controlForce(self.position, self.velocity, self.targets[self.current_target])
        
        self.vele.x = force[0]
        self.vele.y = force[1]
        self.vele.z = force[2]
        self.vele_pub.sendMessage(self.vele)  
        print("Velocity published")
        self.errors.append(np.array(error))
        self.last = error
        
        #checkNextTarget()
    
    def updateSubscriptions(self):
        self.connection_active = False
        
        if self.pose_sub.receiveMessage(self.pose):
            self.connection_active = True
            x = self.pose.position.x
            y = self.pose.position.y
            z = self.pose.position.z
                
            a = self.pose.orientation.w
            i = self.pose.orientation.x
            j = self.pose.orientation.y
            k = self.pose.orientation.z

            self.position = agx.Vec3(x,y,z)            
            self.rotation = agx.Quat(a,i,j,k)
            
        if self.vel_sub.receiveMessage(self.vel):
            self.connection_active = True
            x = self.vel.x
            y = self.vel.y
            z = self.vel.z
                
            self.velocity = agx.Vec3(x,y,z)      
            
            
            
        
    def controlForce(self, position, velocity, target):
        
        error = target - position
        error[-1] = 0 #no force in Z-direction, no error in z-direction
        #TODO: implement force commands in heave-direction

        vel_error = agx.Vec3(0,0,0) - velocity #zero speed is desired
        self.sum += error
        difference = error - self.last
        
        controlled = self.kp * error + self.ki * self.sum + self.kd * vel_error
        
        #Check that the controlled variable is not outside authority
        controlled = self.clamp(controlled, self.authority)
        
              
        return controlled, error
    
    def controlHeading(self, frame, desired_heading, authority):
        rot = frame.getLocalRotate().getAsEulerAngles()[2]#Rotation in local frame 
        
        error_heading = agx.Vec3(0,0, desired_heading - rot) 
        #print(error_heading)
        
        kp = 3000
        command = error_heading * kp #just proportional control
        self.clamp(command, authority)
        
        return command
        

    #Clamps variable to the range 
    #-clamp < variable < clamp
    def clamp(self, variable, clamp):
        if variable.length() > clamp:
            vec = variable.normal()
            return vec * clamp
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
    
    def plot(self):
        self.has_plotted = True
        print("End of targets, plotting...")
        #print(self.errors.shape())
        errors = np.array(self.errors)
        errors = np.sqrt(errors[:,0]**2 + errors[:,1]**2)
        print(errors.shape)
        plt.plot(abs(errors))
        plt.xlabel("Time(ms)")
        plt.ylabel("Positional error(m)")
        plt.title("Control system error with 4 set-points")
        plt.legend
        plt.show()
        input()
   
if __name__ == '__main__':
    main()
