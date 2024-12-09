import agx
import agxSDK

class UsvController(agxSDK.StepEventListener):
    def __init__(self, Kp, Ki, Kd, vessel, authority=5e4, target = agx.Vec3(0,0,0)):
        super().__init__()
        
        self.kp = Kp
        self.ki = Ki
        self.kd = Kd
        self.last = None
        self.sum = agx.Vec3(0,0,0)
        self.steps = 0
        
        self.vessel = vessel
        
        self.authority = authority
        
        self.target = target
        
    def pre(self, t):
        self.steps += 1 
        frame = self.vessel.hull.getFrame()
        pos = frame.getTranslate()
        if self.last is None: 
            self.last = pos
            return        

        error = self.target - self.last
        print(error)
        self.sum += error 
        difference = pos - self.last
        controlled = self.kp * error + self.ki * self.sum + self.kd * difference
        
        #Check that the controlled variable is not outside authority
        controlled = self.clamp(controlled, self.authority)
        controlled[2] = 0 #no force in Z-direction
        
        self.vessel.add_force(controlled)
        
        self.last = pos
        
    #Clamps variable to the range 
    #-clamp < variable < clamp
    def clamp(self, variable, clamp):
        if variable.length() > clamp:
            vec = variable.normal()
            return vec * clamp
        else:
            return variable