
import agx
import agxSDK
import agxUtil
import agxCollide

class Usv(agxSDK.StepEventListener):
    def __init__(self):
        super().__init__()
        
        self.hull = agx.RigidBody("usv_hull")
        
        self.ship = agxSDK.Assembly()
        
    def build(self, hull_obj, density):
        material = agx.Material("usv_material")
        material.getBulkMaterial().setDensity(density)
    
        #The .obj file here is something I've just thrown together. It can be improved and changed for higher accuracy later
        trimesh = agxUtil.createTrimesh(hull_obj, agxCollide.Trimesh.REMOVE_DUPLICATE_VERTICES)
        geometry = agxCollide.Geometry(trimesh)

        
        self.hull = agx.RigidBody(geometry)
        agxUtil.setBodyMaterial(self.hull, material)
        
        self.ship.add(self.hull)
    
    
    def add_force(self, force, position=agx.Vec3(1.4,0,0)):
        self.hull.addForceAtLocalCmPosition(force, position)
