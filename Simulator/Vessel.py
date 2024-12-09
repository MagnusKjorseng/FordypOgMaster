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
    
    def add_force(self, vector, position=None):
        if position is None:
            position = self.hull.getCmPosition()
        
        #Translates force from local frame vector into global force.
        frame = self.hull.getFrame()
        force = frame.transformVectorToWorld(vector)
        
        
        self.hull.addForce(force)