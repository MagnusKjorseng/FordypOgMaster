"""
Base class for general vessels 
"""

import agx
import agxSDK
import agxUtil
import agxCollide
import agxModel

class Vessel(agxSDK.StepEventListener):
    def __init__(self, hullname):
        super().__init__()
        
        self.hull = agx.RigidBody(hullname)
    
    def add_force(self, force):
     
        #Translates force from local frame vector into global force.
        # frame = self.hull.getFrame()
        # force = frame.transformVectorToWorld(vector)
        
        
        self.hull.addForce(force)
    
    #adds torque in world frame
    def add_torque(self, torque):
        self.hull.addTorque(torque)
