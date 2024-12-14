import agx
import agxSDK
import numpy as np
import matplotlib.pyplot as plt

class UsvController(agxSDK.StepEventListener):
    def __init__(self, Kp, Ki, Kd, vessel, authority=2e4, torque = 2e5, targets = [agx.Vec3(0,0,0)]):
        super().__init__()
        
        self.kp = Kp
        self.ki = Ki
        self.kd = Kd
        self.last = 0
        self.sum = agx.Vec3(0,0,0)
        self.steps = 0
        
        self.vessel = vessel
        
        self.authority = authority
        self.torque = torque
        
        self.targets = targets
        self.current_target = 0
        self.error = 0
        self.errors = []
        
        self.heading_desired = agx.Quat()
        self.heading_set = False
        self.has_plotted = False
        self.lastmag = 0
        
    def pre(self,t):
        self.vessel.hull.setVelocity(agx.Vec3(1,0,0))
        
    '''
    def pre(self, t):

        frame = self.vessel.hull.getFrame()
        pos = frame.getTranslate()      
        vel = self.vessel.hull.getVelocity()
        rot = frame.getLocalRotate().getAsEulerAngles()[2]#Rotation in local frame 
        #print(rot)
        if not self.heading_set:
            goal = self.targets[self.current_target]
            direction = goal - pos
            direction.normalize()
            
            angle = np.arcsin(direction[0])
            self.heading_desired = angle
           
        ########
        #heading command
        ########
        error_heading = agx.Vec3(0,0,self.heading_desired - rot)
        print(error_heading)
        kp = 1000
        command = error_heading * kp #just proportional control
        self.clamp(command, self.torque)
        self.vessel.add_torque(command)
        
        ########
        #Force command
        ########
        
        self.error = self.targets[self.current_target] - pos
        self.error[-1] = 0 #no force in Z-direction, no error in z-direction
        #print(self.error)
        self.vel_error = agx.Vec3(0,0,0) - vel
        self.errors.append(np.array(self.error))
        self.sum += self.error
        difference = self.error - self.last
        
        controlled = self.kp * self.error + self.ki * self.sum + self.kd * self.vel_error
        
        #Check that the controlled variable is not outside authority
        controlled = self.clamp(controlled, self.authority)
        #print(controlled)
        
        self.vessel.add_force(controlled)
        
        self.last = self.error
        
        avgLast10 = np.sum(self.errors[-10:], axis=0)/10
        #print(avgLast10)
        #
        #print(np.less(abs(avgLast10), 0.5).all())
        if np.less(abs(avgLast10), 0.5).all():
            if self.current_target + 1 < len(self.targets):
                self.current_target += 1
                print("New Target")
                print(self.targets[self.current_target])
                self.heading_set = False
                input()
                
            elif not self.has_plotted:
                self.has_plotted = True
                print("End of targets, plotting...")
                #print(self.errors.shape())
                errors = np.array(self.errors)
                print(errors.shape)
                plt.semilogy(abs(errors)[:,:2])
                plt.show()
                input()
            
        
    
    #Clamps variable to the range 
    #-clamp < variable < clamp
    def clamp(self, variable, clamp):
        if variable.length() > clamp:
            vec = variable.normal()
            return vec * clamp
        else:
            return variable
    '''