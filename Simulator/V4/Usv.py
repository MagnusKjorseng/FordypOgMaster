
import agx
import agxSDK
import agxUtil
import agxCollide
import agxModel
import agxOSG

import Vessel

class Usv(Vessel.Vessel):
    def __init__(self, hull_obj, density, thrusters):
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

        self.thrusters = thrusters



        #for debugging
        # for thruster in self.thrusters:
        #     thr = self.add_thruster(thruster)
        #     #print(thr.getFrame())
        #     #print(thr.getLocalPosition())
        #     local_pos = agx.Vec3()
        #     self.ship.add(thr)

    #Takes in a force vector and the name of the thruster in question
    # def add_force(self, force, thruster):
    #     position = agx.Vec3(thruster[2][0],thruster[2][1],thruster[2][2])#position 2 is the position of the thruster
    #     self.hull.addForceAtLocalPosition(force, position)

    #For debugging, shows the physical location of the thruster.
    def add_thruster(self, thruster):
        thruster_position = thruster["position"]

        thr = agx.RigidBody()
        thr.setParentFrame(self.ship.getFrame())

        position = agx.Vec3(thruster_position[0], thruster_position[1], thruster_position[2])
        thr.setLocalPosition(position)

        geometry = agxCollide.Geometry(agxCollide.Sphere(0.1))
        geometry.setEnableCollisions(False)
        thr.add(geometry)

        #thruster.setEnable(False)
        return geometry
    

