
import agx
import agxSDK
import agxUtil
import agxCollide
import agxModel
import agxOSG

import Vessel

class Usv(Vessel.Vessel):
    def __init__(self, hull_obj, density, thruster_positions):
        super().__init__("usv_hull")
        
        self.ship = agxSDK.Assembly()



        material = agx.Material("usv_material")
        material.getBulkMaterial().setDensity(density)

        #The .obj file here is something I've just thrown together. It can be improved and changed for higher accuracy later
        trimesh = agxUtil.createTrimesh(hull_obj, agxCollide.Trimesh.REMOVE_DUPLICATE_VERTICES)
        geometry = agxCollide.Geometry(trimesh)


        self.hull = agx.RigidBody(geometry)
        agxUtil.setBodyMaterial(self.hull, material)

        self.ship.add(self.hull)

        self.thruster_positions = thruster_positions

        for thruster in thruster_positions:
            self.add_thruster(thruster)

    
    def add_thruster(self, thruster_position):
        thruster = agx.RigidBody()

        position = agx.Vec3(thruster_position[0], thruster_position[1], thruster_position[2])
        thruster.setPosition(position)

        geometry = agxCollide.Geometry(agxCollide.Sphere(1))
        thruster.add(geometry)

        thruster.setEnable(False)



        self.ship.add(thruster)
    
