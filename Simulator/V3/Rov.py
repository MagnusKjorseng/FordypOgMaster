import agx
import agxCollide
import agxUtil

import Vessel

class Rov(Vessel.Vessel):
    def __init__(self, dims):
        super().__init__("rov_hull")
    
        #rov size in xyz, in meters
        self.dims = dims/2 

    def build(self, density, position):
        #add geometry
        self.hull.add(agxCollide.Geometry(agxCollide.Box(self.dims)))
        
        #define material properties
        material = agx.Material("rov_material")
        material.getBulkMaterial().setDensity(density)
        agxUtil.setBodyMaterial(self.hull, material)
        
        self.hull.updateMassProperties()
        
        self.hull.setPosition(position)
