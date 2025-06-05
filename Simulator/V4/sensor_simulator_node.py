import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import os
import uuid
import numpy as np
from ngc_interfaces.msg import Eta, Nu, NuDot, Wind, HeadingDevice, GNSS, IMU
from ngc_utils.nmea_utils import create_mwv_message, create_hdt_message, create_rot_message, create_gga_message, create_vtg_message
from ngc_utils.geo_utils import add_body_frame_pos_to_lat_lon, add_distance_to_lat_lon
import socket
import ngc_utils.math_utils as mu
from ngc_utils.qos_profiles import default_qos_profile
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from ngc_utils.ngc_base_node import NgcBaseNode
from std_msgs.msg import Bool
import math
import random

class IMUSimulator(Node):
    def __init__(self, sensor_node):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.sensor_node = sensor_node
        self.base        = NgcBaseNode(sensor_node, ['simulator', 'sensor'])
        
        # Load configurations and initialize VesselModel
        self.sensor_config     = self.base.sensor_config
        self.simulation_config = self.base.simulator_config

        # Find the corresponding sensor entry in the YAML file
        self.sensor_entry = self.find_sensor_config(self.sensor_node.sensor_name, self.sensor_node.sensor_type)

        if not self.sensor_entry:
            self.sensor_node.get_logger().error(f"Sensor {self.sensor_node.sensor_name} of type {self.sensor_node.sensor_type} not found in configuration.")
            return
        
        # Initialize parameters from the sensor entry
        self.fix_frequency = self.sensor_entry['simulator']['fix_frequency_hz']
        self.x_pos         = self.sensor_entry['position']['x']
        self.y_pos         = self.sensor_entry['position']['y']
        self.z_pos         = self.sensor_entry['position']['z']
        self.a_bias_max    = self.sensor_entry['simulator']['bias']['accelerometer_max_bias']
        self.w_bias_max    = np.deg2rad(60.0*self.sensor_entry['simulator']['bias']['gyroscope_deg_min_max_bias'])
        self.gravity       = self.simulation_config['physical_parameters']['gravity']

        # Sanity check to avoid zero division in timer setup
        if self.fix_frequency < 0.02:
            self.sensor_node.get_logger().warning('Simulator fix frequency is too high, or out of bounds, constraining to 50Hz')
            self.fix_frequency = 0.02

        # Create timer for main callback
        self.sensor_node.timer = self.sensor_node.create_timer(1.0 / self.fix_frequency, self.timer_callback)

        # Create subscriptions for Eta and Nu messages
        self.sensor_node.eta_subscription           = self.sensor_node.create_subscription(Eta, 'eta_sim', self.eta_callback, default_qos_profile)
        self.sensor_node.nu_subscription            = self.sensor_node.create_subscription(Nu, 'nu_sim', self.nu_callback, default_qos_profile)
        self.sensor_node.nu_dot_subscription        = self.sensor_node.create_subscription(NuDot, 'nu_dot_sim', self.nu_dot_callback, default_qos_profile)
        self.sensor_node.reload_config_subscription = self.sensor_node.create_subscription(Bool, 'sensor_config_update', self.reload_config_callback, default_qos_profile)

        # Create publication for wind measurements
        publication_topic_name        = self.sensor_node.sensor_name.lower().replace(' ', '_') + '_sim_meas'
        self.sensor_node.imu_meas_pub = self.sensor_node.create_publisher(IMU, publication_topic_name, default_qos_profile)

        # Initialize message variables
        self.latest_eta_msg    = None
        self.latest_nu_msg     = None
        self.latest_nu_dot_msg = None

        # Setup random but fixed sensor bias terms
        self.a_bias = np.array([random.uniform(-self.a_bias_max, self.a_bias_max), random.uniform(-self.a_bias_max, self.a_bias_max), random.uniform(-self.a_bias_max, self.a_bias_max)])
        self.w_bias = np.array([random.uniform(-self.w_bias_max, self.w_bias_max), random.uniform(-self.w_bias_max, self.w_bias_max), random.uniform(-self.w_bias_max, self.w_bias_max)])

        self.sensor_node.get_logger().info('IMU simulator setup done')
        
    def find_sensor_config(self, sensor_name, sensor_type):
        
        if sensor_type in self.sensor_config:
            for sensor in self.sensor_config[sensor_type]:
                if sensor['name'] == sensor_name:
                    return sensor
                    
        return None
      
    def reload_config_callback(self, msg: Bool):

        if msg.data == True:
            
            self.base.load_and_initialize_configs(['sensor'])
            self.sensor_config = self.base.sensor_config
            self.sensor_entry  = self.find_sensor_config(self.sensor_node.sensor_name, self.sensor_node.sensor_type)
            self.sensor_node.get_logger().info("The IMU simulator has updated its config!")
        else:
            self.sensor_node.get_logger().warning("The IMU simulator got an update signal, but did not update its file!")

    def eta_callback(self, msg):
        self.latest_eta_msg = msg

    def nu_callback(self, msg):
        self.latest_nu_msg = msg

    def nu_dot_callback(self, msg):
        self.latest_nu_dot_msg = msg

    def timer_callback(self):

        # Output node status message if enabled in config
        self.base.publish_node_status()

        # Simple position noise model
        self.accelerometers_std_dev    = self.sensor_entry['simulator']['noise']['accelerometers_std_dev']
        self.gyroscope_deg_min_std_dev = self.sensor_entry['simulator']['noise']['gyroscope_deg_min_std_dev']

        if (self.latest_eta_msg is not None) and (self.latest_nu_msg is not None) and (self.latest_nu_dot_msg is not None):
            
            position = np.array([self.x_pos, self.y_pos, self.z_pos])
            a_lin    = np.array([self.latest_nu_dot_msg.u_dot, self.latest_nu_dot_msg.v_dot, self.latest_nu_dot_msg.w_dot])
            a_rot    = np.array([self.latest_nu_dot_msg.p_dot, self.latest_nu_dot_msg.q_dot, self.latest_nu_dot_msg.r_dot])
            a_g      = np.array([0.0, 0.0, -self.gravity])
            R        = mu.RotationMatrix(self.latest_eta_msg.phi,self.latest_eta_msg.theta,self.latest_eta_msg.psi)
            omega    = np.array([self.latest_nu_msg.p, self.latest_nu_msg.q, self.latest_nu_msg.r])
            a_noise  = np.random.normal(0, self.accelerometers_std_dev, 3)
            w_noise  = np.random.normal(0, self.gyroscope_deg_min_std_dev, 3)

            a_cross  = np.cross(a_rot,position)
            w_cross  = np.cross(omega,position)
            ww_cross = np.cross(omega,w_cross)
            grav     = R.T @ a_g

            a_sense = a_lin + grav + a_cross + ww_cross + self.a_bias + a_noise
            w_sense = omega + self.w_bias + w_noise

            imu_measurement         = IMU()
            imu_measurement.a_x     = a_sense[0]
            imu_measurement.a_y     = a_sense[1]
            imu_measurement.a_z     = a_sense[2]
            imu_measurement.omega_x = w_sense[0]
            imu_measurement.omega_y = w_sense[1]
            imu_measurement.omega_z = w_sense[2]

            # Implement more faults here
            imu_measurement.signal_valid = True

            self.sensor_node.imu_meas_pub.publish(imu_measurement)

            self.latest_eta_msg = None
            self.latest_nu_msg  = None


############################################################################################################################################

class GNSSSimulator(Node):
    def __init__(self, sensor_node):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.sensor_node = sensor_node
        self.base = NgcBaseNode(sensor_node, ['simulator', 'sensor'])
        
        # Load configurations and initialize VesselModel
        self.sensor_config     = self.base.sensor_config
        self.simulation_config = self.base.simulator_config

        # Declare parameters for the package and config file
        self.sensor_node.declare_parameter('yaml_package_name', 'ngc_bringup')
        self.sensor_node.declare_parameter('sensor_file', 'config/sensor_config.yaml')

        # Find the corresponding sensor entry in the YAML file
        self.sensor_entry = self.find_sensor_config(self.sensor_node.sensor_name, self.sensor_node.sensor_type)

        if not self.sensor_entry:
            self.sensor_node.get_logger().error(f"Sensor {self.sensor_node.sensor_name} of type {self.sensor_node.sensor_type} not found in configuration.")
            return
        
        # Initialize parameters from the sensor entry
        self.fix_frequency = self.sensor_entry['simulator']['fix_frequency_hz']
        self.udp_ip        = self.sensor_entry['simulator']['network_nmea_output']['udp_ip']
        self.udp_port      = self.sensor_entry['simulator']['network_nmea_output']['udp_port']
        self.x_pos         = self.sensor_entry['position']['x']
        self.y_pos         = self.sensor_entry['position']['y']
        self.z_pos         = self.sensor_entry['position']['z']

        # Sanity check to avoid zero division in timer setup
        if self.fix_frequency < 0.02:
            self.sensor_node.get_logger().warning('Simulator fix frequency is too high, or out of bounds, constraining to 50Hz')
            self.fix_frequency = 0.02

        # Create timer for main callback
        self.sensor_node.timer = self.sensor_node.create_timer(1.0 / self.fix_frequency, self.timer_callback)

        # Create subscriptions for Eta and Nu messages
        self.sensor_node.eta_subscription           = self.sensor_node.create_subscription(Eta, 'eta_sim', self.eta_callback, default_qos_profile)
        self.sensor_node.nu_subscription            = self.sensor_node.create_subscription(Nu, 'nu_sim', self.nu_callback, default_qos_profile)
        self.sensor_node.reload_config_subscription = self.sensor_node.create_subscription(Bool, 'sensor_config_update', self.reload_config_callback, default_qos_profile)

        # Create publication for wind measurements
        publication_topic_name            = self.sensor_node.sensor_name.lower().replace(' ', '_') + '_sim_meas'
        self.sensor_node.gnss_meas_pub = self.sensor_node.create_publisher(GNSS, publication_topic_name, default_qos_profile)

        # Initialize noise state and other variables
        self.position_noise_state = np.array([0.0, 0.0])
        self.velocity_noise_state = np.array([0.0, 0.0])

        # Initialize message variables
        self.latest_eta_msg = None
        self.latest_nu_msg  = None

        self.sensor_node.get_logger().info('GNSS simulator setup done')

    def find_sensor_config(self, sensor_name, sensor_type):
        
        if sensor_type in self.sensor_config:
            for sensor in self.sensor_config[sensor_type]:
                if sensor['name'] == sensor_name:
                    return sensor
        return None
      
    def reload_config_callback(self, msg: Bool):

        if msg.data == True:
            self.base.load_and_initialize_configs(['sensor'])
            self.sensor_config = self.base.sensor_config
            self.sensor_entry  = self.find_sensor_config(self.sensor_node.sensor_name, self.sensor_node.sensor_type)
            self.sensor_node.get_logger().info("The GNSS simulator has updated its config!")
        else:
            self.sensor_node.get_logger().warning("The GNSS simulator got an update signal, but did not update its file!")

    def eta_callback(self, msg):
        self.latest_eta_msg = msg

    def nu_callback(self, msg):
        self.latest_nu_msg = msg

    def timer_callback(self):

        # Output node status message if enabled in config
        self.base.publish_node_status()

        # Simple position noise model
        self.position_m_std_dev  = self.sensor_entry['simulator']['noise']['position_m_std_dev']
        self.velocity_ms_std_dev = self.sensor_entry['simulator']['noise']['velocity_ms_std_dev']

        white_noise = np.random.normal(0, self.position_m_std_dev, 2)

        self.position_noise_state[0] += 0.1*(-0.1*self.position_noise_state[0] + white_noise[0])
        self.position_noise_state[1] += 0.1*(-0.1*self.position_noise_state[1] + white_noise[1])

        self.velocity_noise_state = np.random.normal(0, self.velocity_noise_state, 2)

        if (self.latest_eta_msg is not None) and (self.latest_nu_msg is not None):
            # Calculate velocity considering noise and movement due to roll and pitch rates
            # For simplicity, roll_rate affects sway (v) and pitch_rate affects surge (u)
            # But with SNAME conventions, positive roll increases sway and positive pitch increases surge

            antenna_movement_due_to_pitch = -self.latest_nu_msg.q * self.z_pos
            antenna_movement_due_to_roll = -self.latest_nu_msg.p * self.z_pos
            
            u_with_noise_and_movement = self.latest_nu_msg.u + self.velocity_noise_state[0] + antenna_movement_due_to_pitch
            v_with_noise_and_movement = self.latest_nu_msg.v + self.velocity_noise_state[1] + antenna_movement_due_to_roll
            
            antenna_lat, antenna_lon = add_body_frame_pos_to_lat_lon(self.latest_eta_msg.lat, self.latest_eta_msg.lon, self.x_pos, self.y_pos, self.z_pos, math.degrees(self.latest_eta_msg.phi), math.degrees(self.latest_eta_msg.theta), math.degrees(self.latest_eta_msg.psi))
            noisy_lat, noisy_lon     = add_distance_to_lat_lon(antenna_lat, antenna_lon, self.position_noise_state[0], self.position_noise_state[1])

            # Rotate velocities to NED for COG calculation
            nu_3_dof       = np.array([u_with_noise_and_movement,v_with_noise_and_movement,0.0])
            R              = mu.RotationMatrix(0.0,0.0,self.latest_eta_msg.psi)
            NED_velocities = R @ nu_3_dof

            # Generate and send GGA and VTG NMEA messages
            gga_message = create_gga_message(noisy_lat, noisy_lon)
            vtg_message = create_vtg_message(NED_velocities[0], NED_velocities[1])
            
            self.sock.sendto(gga_message.encode(), (self.udp_ip, self.udp_port))
            self.sock.sendto(vtg_message.encode(), (self.udp_ip, self.udp_port))

            true_course_radians = math.atan2(NED_velocities[1], NED_velocities[0])  # Angle in radians from North
            true_course_degrees = math.degrees(true_course_radians)
            
            if true_course_degrees < 0:
                true_course_degrees += 360  # Normalize angle to 0-360 degrees

            gnss_measurement           = GNSS()
            gnss_measurement.latitude  = noisy_lat
            gnss_measurement.longitude = noisy_lon
            gnss_measurement.sog       = math.sqrt(NED_velocities[0]**2 + NED_velocities[1]**2)
            gnss_measurement.cog       = true_course_degrees

            # Implement more faults here
            gnss_measurement.signal_valid        = True
            gnss_measurement.sogcog_signal_valid = True

            self.sensor_node.gnss_meas_pub.publish(gnss_measurement)

            self.latest_eta_msg = None
            self.latest_nu_msg  = None


####################################################################################################################################################

class HeadingDeviceSimulator(Node):
    def __init__(self, sensor_node):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.sensor_node = sensor_node
        self.base        = NgcBaseNode(sensor_node, ['simulator', 'sensor'])
        
        # Load configurations and initialize VesselModel
        self.sensor_config     = self.base.sensor_config
        self.simulation_config = self.base.simulator_config

        # Declare parameters for the package and config file
        self.sensor_node.declare_parameter('yaml_package_name', 'ngc_bringup')
        self.sensor_node.declare_parameter('sensor_file', 'config/sensor_config.yaml')

        # Find the corresponding sensor entry in the YAML file
        self.sensor_entry = self.find_sensor_config(self.sensor_node.sensor_name, self.sensor_node.sensor_type)

        if not self.sensor_entry:
            self.sensor_node.get_logger().error(f"Sensor {self.sensor_node.sensor_name} of type {self.sensor_node.sensor_type} not found in configuration.")
            return
        
        # Initialize parameters from the sensor entry
        self.fix_frequency = self.sensor_entry['simulator']['fix_frequency_hz']
        self.udp_ip        = self.sensor_entry['simulator']['network_nmea_output']['udp_ip']
        self.udp_port      = self.sensor_entry['simulator']['network_nmea_output']['udp_port']
        
        # Sanity check to avoid zero division in timer setup
        if self.fix_frequency < 0.02:
            self.sensor_node.get_logger().warning('Simulator fix frequency is too high, or out of bounds, constraining to 50Hz')
            self.fix_frequency = 0.02

        # Create timer for main callback
        self.sensor_node.timer = self.sensor_node.create_timer(1.0 / self.fix_frequency, self.timer_callback)

        # Create subscriptions for Eta and Nu messages
        self.sensor_node.eta_subscription           = self.sensor_node.create_subscription(Eta, 'eta_sim', self.eta_callback, default_qos_profile)
        self.sensor_node.nu_subscription            = self.sensor_node.create_subscription(Nu, 'nu_sim', self.nu_callback, default_qos_profile)
        self.sensor_node.reload_config_subscription = self.sensor_node.create_subscription(Bool, 'sensor_config_update', self.reload_config_callback, default_qos_profile)

        # Create publication for wind measurements
        publication_topic_name            = self.sensor_node.sensor_name.lower().replace(' ', '_') + '_sim_meas'
        self.sensor_node.heading_meas_pub = self.sensor_node.create_publisher(HeadingDevice, publication_topic_name, default_qos_profile)

        # Initialize message variables
        self.latest_eta_msg = None
        self.latest_nu_msg  = None

        self.sensor_node.get_logger().info('Heading device simulator node setup done')
        
    def find_sensor_config(self, sensor_name, sensor_type):
        
        if sensor_type in self.sensor_config:
            for sensor in self.sensor_config[sensor_type]:
                if sensor['name'] == sensor_name:
                    return sensor
        return None
      
    def reload_config_callback(self, msg: Bool):

        if msg.data == True:
            self.base.load_and_initialize_configs(['sensor'])
            self.sensor_config = self.base.sensor_config
            self.sensor_entry  = self.find_sensor_config(self.sensor_node.sensor_name, self.sensor_node.sensor_type)
            self.sensor_node.get_logger().info("The heading device simulator has updated its config!")
        else:
            self.sensor_node.get_logger().warning("The headind devics simulator got an update signal, but did not update its file!")

    def eta_callback(self, msg):
        self.latest_eta_msg = msg

    def nu_callback(self, msg):
        self.latest_nu_msg = msg

    def timer_callback(self):
        
        # Output node status message if enabled in config
        self.base.publish_node_status()

        # Update the noise model
        self.position_std_dev = self.sensor_entry['simulator']['noise']['heading_deg_std_dev']
        self.velocity_std_dev = self.sensor_entry['simulator']['noise']['rot_deg_min_std_dev']

        self.heading_noise_state = np.random.normal(0, self.position_std_dev, 1)
        self.rot_noise_state     = np.random.normal(0, self.velocity_std_dev, 1)

        if (self.latest_eta_msg is not None) and (self.latest_nu_msg is not None):
            
            noisy_heading_deg = np.rad2deg(mu.mapToZeroTwoPi(self.latest_eta_msg.psi + np.deg2rad(self.heading_noise_state[0])))
            noisy_rot_deg_sec = (np.rad2deg(self.latest_nu_msg.r) + self.rot_noise_state[0]/60.0)

            # Generate and send HDT and ROT NMEA messages
            hdt_message = create_hdt_message(noisy_heading_deg)
            rot_message = create_rot_message(noisy_rot_deg_sec*60.0)
            
            self.sock.sendto(hdt_message.encode(), (self.udp_ip, self.udp_port))
            self.sock.sendto(rot_message.encode(), (self.udp_ip, self.udp_port))

            heading_measurement             = HeadingDevice()
            heading_measurement.heading_deg = noisy_heading_deg
            heading_measurement.rot_degsec  = noisy_rot_deg_sec

            # Implement faults here later
            heading_measurement.signal_valid = True

            self.sensor_node.heading_meas_pub.publish(heading_measurement)

            # Clear the local message copies
            self.latest_eta_msg = None
            self.latest_nu_msg  = None


#######################################################################################################################################################

class AnemometerSimulator(Node):
    def __init__(self, sensor_node):
    
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.sensor_node = sensor_node
        self.base        = NgcBaseNode(sensor_node, ['simulator', 'sensor'])
        
        # Load configurations and initialize VesselModel
        self.sensor_config     = self.base.sensor_config
        self.simulation_config = self.base.simulator_config

        # Declare parameters for the package and config file
        self.sensor_node.declare_parameter('yaml_package_name', 'ngc_bringup')
        self.sensor_node.declare_parameter('sensor_file', 'config/sensor_config.yaml')

        # Find the corresponding sensor entry in the YAML file
        self.sensor_entry = self.find_sensor_config(self.sensor_node.sensor_name, self.sensor_node.sensor_type)

        if not self.sensor_entry:
            self.sensor_node.get_logger().error(f"Sensor {self.sensor_node.sensor_name} of type {self.sensor_node.sensor_type} not found in configuration.")
            return
        
        # Initialize parameters from the sensor entry
        self.fix_frequency = self.sensor_entry['simulator']['fix_frequency_hz']
        self.udp_ip        = self.sensor_entry['simulator']['network_nmea_output']['udp_ip']
        self.udp_port      = self.sensor_entry['simulator']['network_nmea_output']['udp_port']
        self.x_pos         = self.sensor_entry['position']['x']
        self.y_pos         = self.sensor_entry['position']['y']
        self.z_pos         = self.sensor_entry['position']['z']

        # Sanity check to avoid zero division in timer setup
        if self.fix_frequency < 0.02:
            self.sensor_node.get_logger().warning('Simulator fix frequency is too high, or out of bounds, constraining to 50Hz')
            self.fix_frequency = 0.02

        # Create timer for main callback
        self.sensor_node.timer = self.sensor_node.create_timer(1.0 / self.fix_frequency, self.timer_callback)

        # Create subscriptions for Eta and Nu messages
        self.sensor_node.nu_subscription            = self.sensor_node.create_subscription(Nu, 'nu_sim', self.nu_callback, default_qos_profile)
        self.sensor_node.wind_subscription          = self.sensor_node.create_subscription(Wind, 'wind_sim', self.wind_callback, default_qos_profile)
        self.sensor_node.reload_config_subscription = self.sensor_node.create_subscription(Bool, 'sensor_config_update', self.reload_config_callback, default_qos_profile)

        # Create publication for wind measurements
        publication_topic_name    = self.sensor_node.sensor_name.lower().replace(' ', '_') + '_sim_meas'
        self.sensor_node.wind_pub = self.sensor_node.create_publisher(Wind, publication_topic_name, default_qos_profile)
        
        # Initialize noise state and other variables
        self.direction_deg_noise_state = 0.0
        self.magnitude_ms_noise_state  = 0.0

        # Initialize message variables
        self.latest_wind_msg = None
        self.latest_nu_msg   = None

        self.sensor_node.get_logger().info('Anemometer simulator node setup done')

    def find_sensor_config(self, sensor_name, sensor_type):
        
        if sensor_type in self.sensor_config:
            for sensor in self.sensor_config[sensor_type]:
                if sensor['name'] == sensor_name:
                    return sensor
        return None

    def reload_config_callback(self, msg: Bool):

        if msg.data == True:
            self.base.load_and_initialize_configs(['sensor'])
            self.sensor_config = self.base.sensor_config
            self.sensor_entry  = self.find_sensor_config(self.sensor_node.sensor_name, self.sensor_node.sensor_type)
            self.sensor_node.get_logger().info("The anemometer simulator has updated its config!")
        else:
            self.sensor_node.get_logger().warning("The anemometer simulator got an update signal, but did not update its file!")
    
    def nu_callback(self, msg):
        self.latest_nu_msg = msg

    def wind_callback(self, msg):
        self.latest_wind_msg = msg

    def timer_callback(self):

        # Output node status message if enabled in config
        self.base.publish_node_status()

        self.direction_deg_std_dev = self.sensor_entry['simulator']['noise']['direction_deg_std_dev']
        self.magnitude_ms_std_dev  = self.sensor_entry['simulator']['noise']['magnitude_ms_std_dev']
        
        # Simple position noise model
        self.direction_deg_noise_state = np.random.normal(0, self.direction_deg_std_dev, 1)
        self.magnitude_ms_noise_state  = np.random.normal(0, self.magnitude_ms_noise_state, 1)    

        if (self.latest_nu_msg is not None) and (self.latest_wind_msg is not None):
            
            lever_arm_pitch = np.sqrt(self.x_pos**2 + self.z_pos**2)
            lever_arm_roll  = np.sqrt(self.y_pos**2 + self.z_pos**2)
            u_rw            = self.latest_wind_msg.magnitude_ms*np.cos(np.radians(self.latest_wind_msg.direction_relative_deg)) - lever_arm_pitch*self.latest_nu_msg.q
            v_rw            = self.latest_wind_msg.magnitude_ms*np.sin(np.radians(self.latest_wind_msg.direction_relative_deg)) - lever_arm_roll*self.latest_nu_msg.p

            relative_wind_direction = np.rad2deg(mu.mapToPiPi(-np.arctan2(v_rw,u_rw)))
            relative_wind_speed     = np.sqrt(u_rw**2 + v_rw**2)

            noisy_wind_direction_deg = np.deg2rad(mu.mapToPiPi(np.deg2rad(relative_wind_direction + self.direction_deg_noise_state[0])))
            noisy_wind_speed_ms      = relative_wind_speed + self.magnitude_ms_noise_state[0]

            # Generate and send MWV NMEA messages
            mwv_message = create_mwv_message(noisy_wind_direction_deg,noisy_wind_speed_ms)
            
            self.sock.sendto(mwv_message.encode(), (self.udp_ip, self.udp_port))

            wind_measurement                        = Wind()
            wind_measurement.direction_relative_deg = noisy_wind_direction_deg
            wind_measurement.magnitude_ms           = noisy_wind_speed_ms
            
            # Extend this with faults later
            wind_measurement.signal_valid = True

            self.sensor_node.wind_pub.publish(wind_measurement)

            self.latest_nu_msg   = None
            self.latest_wind_msg = None
    
############################################################################################################################

class SensorSimulatorNode(Node):
    def __init__(self):
        # Append a random number to the node name to avoid conflicts
        unique_id = str(uuid.uuid4()).split('-')[0]  # Use the first part of a UUID
        node_name = f'sensor_simulator_node_{unique_id}'

        super().__init__(node_name)
        self.get_logger().info(f"Setting up a general sensor node simualtor with name: {node_name}, where the name will be replaced shortly")

        # Declare parameters and retrieve their values after node initialization
        self.declare_parameter('sensor_name', 'default_sensor')
        self.declare_parameter('sensor_type', 'default_type')

        self.sensor_name = self.get_parameter('sensor_name').get_parameter_value().string_value
        self.sensor_type = self.get_parameter('sensor_type').get_parameter_value().string_value

        # Process the sensor name for logging or logic
        processed_name  = self.sensor_name.lower().replace(' ', '_') + '_simulator'
        self._node_name = processed_name

        if self.sensor_type == 'anemometer':
            AnemometerSimulator(self)
        elif self.sensor_type == 'heading_device':
            HeadingDeviceSimulator(self)
        elif self.sensor_type == 'gnss':
            GNSSSimulator(self)
        elif self.sensor_type == 'imu':
            IMUSimulator(self)
        else:
            self.get_logger().error(f"Did not match sensor type, cannot spawn simulator for this device: {self.sensor_name} {self.sensor_type}")

def main(args=None):
    rclpy.init(args=args)

    # Create the node
    node = SensorSimulatorNode()

    # Spin the node
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
