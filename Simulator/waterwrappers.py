import agxModel
import math

class WaveWaterWrapper(agxModel.WaterWrapper):
    #Shamelessly stolen from the hydrodynamics tutorial for AGX
    def __init__(self):
        super().__init__()

    def findHeightFromSurface(self, worldPoint, upVector, t):
        surface_level = -1 + 0.5 * math.sin(0.5 * worldPoint.x() + 0.6 * t) + \
            0.25 * math.cos(0.6 * worldPoint.y() + 0.3 * worldPoint.x() + 1.45 * t)
        return worldPoint.z() - surface_level

class StillWaterWrapper(agxModel.WaterWrapper):
    def __init__(self):
        super().__init__()
        
    def findHeightFromSurface(self, worldPoint, upVector, t):
        return worldPoint.z()