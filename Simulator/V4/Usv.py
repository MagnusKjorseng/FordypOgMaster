
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

        self.thrusters = thruster_positions



        #for debugging
        for thruster in self.thrusters:
            thr = self.add_thruster(thruster)
            #print(thr.getFrame())
            #print(thr.getLocalPosition())
            local_pos = agx.Vec3
            self.ship.add(thr)

    def add_force(self, forces):
        if len(forces) == len(self.thrusters):
            for i in range(len(self.thrusters)):
                self.hull.addForceAtLocalPosition(forces[i], self.thrusters[i])
        else:
            print("Forces vector not the same length as number of thrusters")

    #For debugging, shows the physical location of the thruster.
    def add_thruster(self, thruster_position):
        thruster = agx.RigidBody()
        thruster.setParentFrame(self.ship.getFrame())

        position = agx.Vec3(thruster_position[0], thruster_position[1], thruster_position[2])
        thruster.setLocalPosition(position)

        geometry = agxCollide.Geometry(agxCollide.Sphere(0.1)
        geometry.setEnableCollisions(False)
        thruster.add(geometry)

        #thruster.setEnable(False)
        return geometry
    

