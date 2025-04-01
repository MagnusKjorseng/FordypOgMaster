#################################
##
## This is a translator node between the simulator and the controller
## The purpose of this translator is to take positional data from the sim,
## which is given in ROS2 Vector3's, twists and other types, and convert it to standard
## NMEA-GGA messages, similar to the ones that come from the GPS.
## The purpose of this node is to make the controller more agnostic
## This node should not be included and run in the proper implementation.
##
##
#################################


import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
import geometry_msgs.msg as geo_msgs
import nmea_msgs.msg as nmea_msgs
from datetime import datetime, UTC

import utils.nmea_utils as nmea_utils #custom library
from utils.geo_utils import add_distance_to_lat_lon, calculate_distance_north_east

import os
import yaml
from ament_index_python.packages import get_package_share_directory


def main(args=None):
    rclpy.init(args=args)

    #Since this is translating from a simulation based around origin (0,0,0) to real-world simulated coordinates, a baseline "starting point" is necessary. This is what (0,0,0) will be converted to.
    # I've chosen an arbitrary point outside Ålesund as my baseline.
    baseline = [62.5, 6.1] #degrees North, east

    position = Translator(baseline)

    rclpy.spin(position)

class Translator(Node):
    def __init__(self, baseline):
        super().__init__("position_translator")

        self.pose_subscriber = self.create_subscription(geo_msgs.Pose,
                                                        "usv_pose",
                                                        self.pose_callback,
                                                        5)
        self.gga_publisher = self.create_publisher(nmea_msgs.Gpgga,
                                                   "gpgga_received",
                                                   5)
        self.hdt_publisher = self.create_publisher(nmea_msgs.Gphdt,
                                                   "gphdt_received",
                                                   5)
        self.baseline = baseline
        self.current_position = baseline

    def pose_callback(self, msg):
        position = msg.position
        position = [position.x, position.y, position.z]

        # since the position in the simulator is given in meters from the origin,
        # this function allows me to use the position as the
        new_lat, new_lon = add_distance_to_lat_lon(self.baseline[0], self.baseline[1], position[0], position[1])
        gga_msg = self.make_gga_message(new_lat, new_lon)

        self.gga_publisher.publish(gga_msg)



        orientation = msg.orientation
        orientation = [orientation.x, orientation.y, orientation.z, orientation.w]

        rot = Rotation.from_quat(orientation)
        rot = rot.as_euler("xyz", degrees=True)
        heading = rot[-1]

        hdt_msg = self.make_hdt_message(heading)

        self.hdt_publisher.publish(hdt_msg)

    def make_gga_message(self, lat, lon):
        message = nmea_msgs.Gpgga()

        message.message_id = "$GPGGA"
        #Get time in UTC formatted HHMMSS.ms to 2 sig figs
        message.utc_seconds = float(datetime.now(UTC).strftime("%H%M%S.%f")[:9])

        #the important stuff
        message.lat = float(lat)
        message.lon = float(lon)
        message.lat_dir = "N"
        message.lon_dir = "E"

        #other stuff
        message.gps_qual = 1
        message.num_sats = 10
        message.hdop = 1.0
        message.alt = 0.0 #TODO: implement height from simulation?
        message.altitude_units = "M"
        message.undulation = 0.0
        message.undulation_units = "M"
        message.diff_age = 0
        message.station_id = ""

        return message

    def make_hdt_message(self, heading):
        message = nmea_msgs.Gphdt()

        message.message_id = "$GPHDT"
        message.heading = heading
        message.rel_to = "T"

        return message

if __name__ == '__main__':
    main()

