import agxModel
import math

class WaveWaterWrapper(agxModel.WaterWrapper):
    #Shamelessly stolen from the hydrodynamics tutorial for AGX
    def __init__(self, wave_heigh):
        super().__init__()
        self.wave_heigh = wave_height

    def findHeightFromSurface(self, worldPoint, upVector, t):
        surface_level = self.wave_height*(0.5 * math.sin(0.5 * worldPoint.x() + 0.6 * t) + \
            0.25 * math.cos(0.6 * worldPoint.y() + 0.3 * worldPoint.x() + 1.45 * t))
        return worldPoint.z() - surface_level
