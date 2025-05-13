import numpy as np
import json
from datetime import datetime
from .dynamixel_client import *


class oldRapidNode:
    """
    The RapidNode class controls the robot's servomotors by configuring them with initial parameters such as PID gains,
    current limits, and positions, as well as reading their status (position, velocity, current). The class can 
    also load and save its configuration from/to a JSON file, and it allows controlling the servos via LEAP pose data.
    """

    def __init__(self, config_path='args/rapid.json', port='/dev/ttyUSB0', init_set=False):
        """
        Initializes a RapidNode instance by loading configuration, initializing motor parameters, and establishing 
        communication with the motors using the specified port.

        :param config_path: Path to the configuration file, default is 'args/rapid.json'.
        :param port: Communication port for the Dynamixel motor controller, default is '/dev/ttyUSB0'.
        :param init_set: Boolean flag to initialize the target position, default is False.
        """
        # Load configuration from the specified file path
        self.config_path = config_path
        self.config = self.load_args(config_path)
        
        # Initialize control parameters (PID gains and current limits) from the configuration
        self.kP = np.array(self.config['init']['kP'])  # Proportional gain
        self.kI = np.array(self.config['init']['kI'])  # Integral gain
        self.kD = np.array(self.config['init']['kD'])  # Derivative gain
        self.curr_lim = np.array(self.config['init']['curr_lim'])  # Current limit
        self.init_pos = self.prev_pos = self.curr_pos = np.array(self.config['init']['pos'])  # Current and previous positions
        
        # Initialize motors based on the number of positions in the configuration
        self.motors = range(len(self.config['init']['pos']))
        
        # Try to connect to the Dynamixel motors using the specified port
        try:
            self.dxl_client = DynamixelClient(self.motors, port, 4000000)  # Initialize motor client with the port
            self.dxl_client.connect()  # Connect to the motor controller
        except Exception as e:
            print(f"Failed to connect using port {port}: {e}")
            raise

        # Configure the motors (e.g., setting torque, gains, and current limits)
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * 5, 11, 1)  # Set initial values for motors
        self.dxl_client.set_torque_enabled(self.motors, True)  # Enable the motor torque
        self.dxl_client.sync_write(self.motors, self.kP, 84, 2)  # Set proportional gain (kP)
        self.dxl_client.sync_write(self.motors, self.kI, 82, 2)  # Set integral gain (kI)
        self.dxl_client.sync_write(self.motors, self.kD, 80, 2)  # Set derivative gain (kD)
        self.dxl_client.sync_write(self.motors, self.curr_lim, 102, 2)  # Set current limit

        # If init_set is True, initialize the motor positions
        if init_set:
            self.dxl_client.write_desired_pos(self.motors, self.init_pos)

    def load_args(self, file_path):
        """
        Load configuration from a JSON file. This method reads the configuration file and returns it as a dictionary.
        
        :param file_path: Path to the configuration file.
        :return: A dictionary containing the configuration.
        """
        with open(file_path, 'r', encoding='utf-8') as file:
            return json.load(file)

    def save_args(self, save_path=None):
        """
        Save the current configuration (PID gains, current limits, position) to a JSON file.
        
        :param save_path: Path to save the configuration, default is 'args/rapid_timestamp.json'.
        """
        if save_path is None:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            save_path = f'args/rapid_{timestamp}.json'
        
        # Construct the configuration dictionary to save
        current_config = {
            'init': {
                'kP': self.kP.tolist(),
                'kI': self.kI.tolist(),
                'kD': self.kD.tolist(),
                'curr_lim': self.curr_lim.tolist(),
                'pos': self.curr_pos.tolist()
            }
        }
        
        # Save the configuration to the specified path
        with open(save_path, 'w', encoding='utf-8') as file:
            json.dump(current_config, file, indent=4)
        print(f"Configuration saved to {save_path}")

    def set(self, pose):
        """
        Set the desired position of the servomotors by receiving a RAPID pose.

        This method calculates the desired motor position by adding the received RAPID pose (a 20-element array)
        to the initial position, then adjusts the motor positions based on the robot's coordinate system,
        and finally writes the computed position to the motors.

        :param pose: A 20-element array representing the RAPID pose (relative to the initial position).
        """
        self.prev_pos = self.curr_pos  # Save the current position as the previous position
        self.curr_pos = np.array([x + y for x, y in zip(self.init_pos, pose)])  # Compute the new position by adding the RAPID pose to the initial position
        
        # Adjust the motor positions (specific to this robot's coordinate system)
        temp = self.curr_pos[0]
        self.curr_pos[0] = -self.curr_pos[0] - self.curr_pos[1]
        self.curr_pos[1] = temp - self.curr_pos[1]
        # Adjust the motor positions (specific to this robot's coordinate system)
        temp = self.curr_pos[4]
        self.curr_pos[4] = -self.curr_pos[4] - self.curr_pos[5]
        self.curr_pos[5] = temp - self.curr_pos[5]
        # Adjust the motor positions (specific to this robot's coordinate system)
        temp = self.curr_pos[8]
        self.curr_pos[8] = -self.curr_pos[8] - self.curr_pos[9]
        self.curr_pos[9] = temp - self.curr_pos[9]
        # Adjust the motor positions (specific to this robot's coordinate system)
        temp = self.curr_pos[12]
        self.curr_pos[12] = -self.curr_pos[12] - self.curr_pos[13]
        self.curr_pos[13] = temp - self.curr_pos[13]
        # Adjust the motor positions (specific to this robot's coordinate system)
        temp = self.curr_pos[16]
        self.curr_pos[16] = -self.curr_pos[16] - self.curr_pos[17]
        self.curr_pos[17] = temp - self.curr_pos[17]
        
        # Write the desired position to the motors
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)


    def read_pos(self):
        """
        Read the current position of the servomotors.
        
        :return: The current position of the motors (adjusted for the initial position).
        """
        temppos = self.dxl_client.read_pos() - self.init_pos
        temp = temppos[0]
        
        # Adjust the position values (specific to this robot's coordinate system)
        temppos[0] = -(temppos[0] - temppos[1]) / 2
        temppos[1] = -(temp + temppos[1]) / 2
        temp = temppos[4]
        # Adjust the position values (specific to this robot's coordinate system)
        temppos[4] = -(temppos[4] - temppos[5]) / 2
        temppos[5] = -(temp + temppos[5]) / 2
        temp = temppos[8]
        # Adjust the position values (specific to this robot's coordinate system)
        temppos[8] = -(temppos[8] - temppos[9]) / 2
        temppos[9] = -(temp + temppos[9]) / 2
        temp = temppos[12]
        # Adjust the position values (specific to this robot's coordinate system)
        temppos[12] = -(temppos[12] - temppos[13]) / 2
        temppos[13] = -(temp + temppos[13]) / 2
        temp = temppos[16]
        # Adjust the position values (specific to this robot's coordinate system)
        temppos[16] = -(temppos[16] - temppos[17]) / 2
        temppos[17] = -(temp + temppos[17]) / 2
        
        return temppos

    def read_vel(self):
        """
        Read the velocity of the servomotors.
        
        :return: The velocity of the motors.
        """
        return self.dxl_client.read_vel()

    def read_cur(self):
        """
        Read the current flowing through the servomotors.
        
        :return: The current flowing through the motors.
        """
        return self.dxl_client.read_cur()

    def setkP(self, data):
        """
        Set the proportional gain (kP) for the PID controller.
        
        :param data: The new proportional gain (kP).
        """
        self.kP = np.array(data)
        self.dxl_client.sync_write(self.motors, self.kP, 84, 2)  # Update the proportional gain

    def setkI(self, data):
        """
        Set the integral gain (kI) for the PID controller.
        
        :param data: The new integral gain (kI).
        """
        self.kI = np.array(data)
        self.dxl_client.sync_write(self.motors, self.kI, 82, 2)  # Update the integral gain

    def setkD(self, data):
        """
        Set the derivative gain (kD) for the PID controller.
        
        :param data: The new derivative gain (kD).
        """
        self.kD = np.array(data)
        self.dxl_client.sync_write(self.motors, self.kD, 80, 2)  # Update the derivative gain

    def set_curr_lim(self, data):
        """
        Set the current limit for the servomotors.
        
        :param data: The new current limit.
        """
        self.curr_lim = np.array(data)
        self.dxl_client.sync_write(self.motors, self.curr_lim, 102, 2)  # Update the current limit



class LeapNode:
    """
    The LeapNode class is used to control a robot through LEAP gestures and manage associated configurations and parameters.
    It manages the robot's servomotors, control gains, current limits, and other settings, and provides interfaces for hardware interaction.
    """

    def __init__(self, config_path='args/leap.json', port='/dev/ttyUSB0', init_set=False):
        """
        Initializes a LeapNode instance, loads configuration, and initializes servomotors and control parameters.
        
        :param config_path: Path to the configuration file, default is 'args/leap.json'.
        :param port: Port for communication with the Dynamixel motor controller, default is '/dev/ttyUSB0'.
        :param init_set: Boolean flag to initialize the target position, default is False.
        """
        # Load configuration from the specified file
        self.config_path = config_path
        self.config = self.load_args(config_path)
        
        # Initialize control parameters from the configuration
        self.kP = np.array(self.config['init']['kP'])  # Proportional gain
        self.kI = np.array(self.config['init']['kI'])  # Integral gain
        self.kD = np.array(self.config['init']['kD'])  # Derivative gain
        self.curr_lim = np.array(self.config['init']['curr_lim'])  # Current limit
        self.prev_pos = self.init_pos = self.curr_pos = np.array(self.config['init']['pos'])  # Current and previous positions
        
        # Initialize servomotors
        self.motors = range(len(self.config['init']['pos']))
        
        # Try to connect to the Dynamixel motors using the specified port
        try:
            self.dxl_client = DynamixelClient(self.motors, port, 4000000)  # Initialize communication client with the specified port
            self.dxl_client.connect()  # Connect to the motors
        except Exception as e:
            print(f"Failed to connect using port {port}: {e}")
            raise

        # Configure motors
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * 5, 11, 1)  # Sync write to motors
        self.dxl_client.set_torque_enabled(self.motors, True)  # Enable motor torque
        self.dxl_client.sync_write(self.motors, self.kP, 84, 2)  # Set proportional gain
        self.dxl_client.sync_write(self.motors, self.kI, 82, 2)  # Set integral gain
        self.dxl_client.sync_write(self.motors, self.kD, 80, 2)  # Set derivative gain
        self.dxl_client.sync_write(self.motors, self.curr_lim, 102, 2)  # Set current limit

        if init_set:
            self.dxl_client.write_desired_pos(self.motors, self.curr_pos)  # If needed, set the target position

    def load_args(self, file_path):
        """
        Load configuration from a JSON file.
        
        :param file_path: Path to the configuration file.
        :return: A dictionary containing the configuration.
        """
        with open(file_path, 'r', encoding='utf-8') as file:
            return json.load(file)

    def save_args(self, save_path=None):
        """
        Save the current configuration to a JSON file.
        
        :param save_path: Path to save the configuration, default is 'args/leap_timestamp.json'.
        """
        if save_path is None:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            save_path = f'args/leap_{timestamp}.json'
        
        # Construct the configuration dictionary to save
        current_config = {
            'init': {
                'kP': self.kP.tolist(),
                'kI': self.kI.tolist(),
                'kD': self.kD.tolist(),
                'curr_lim': self.curr_lim.tolist(),
                'pos': self.curr_pos.tolist()
            }
        }
        
        with open(save_path, 'w', encoding='utf-8') as file:
            json.dump(current_config, file, indent=4)
        print(f"Configuration saved to {save_path}")

    def set(self, pose):
        """
        Control the servomotors based on the LEAP pose by directly setting the target position.
        
        :param pose: The LEAP pose (relative to the initial position).
        """
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array([x + y for x, y in zip(self.init_pos, pose)])
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)

    def read_pos(self):
        """
        Read the current position of the servomotors.
        
        :return: Current position of the motors (relative to the initial position).
        """
        return self.dxl_client.read_pos() - self.init_pos

    def read_vel(self):
        """
        Read the velocity of the servomotors.
        
        :return: Current velocity of the motors.
        """
        return self.dxl_client.read_vel()

    def read_cur(self):
        """
        Read the current flowing through the servomotors.
        
        :return: Current flowing through the motors.
        """
        return self.dxl_client.read_cur()

    def setkP(self, data):
        """
        Set the proportional gain (kP).
        
        :param data: The new proportional gain.
        """
        self.kP = np.array(data)
        self.dxl_client.sync_write(self.motors, self.kP, 84, 2)  # Update the proportional gain

    def setkI(self, data):
        """
        Set the integral gain (kI).
        
        :param data: The new integral gain.
        """
        self.kI = np.array(data)
        self.dxl_client.sync_write(self.motors, self.kI, 82, 2)  # Update the integral gain

    def setkD(self, data):
        """
        Set the derivative gain (kD).
        
        :param data: The new derivative gain.
        """
        self.kD = np.array(data)
        self.dxl_client.sync_write(self.motors, self.kD, 80, 2)  # Update the derivative gain

    def set_curr_lim(self, data):
        """
        Set the current limit for the servomotors.
        
        :param data: The new current limit.
        """
        self.curr_lim = np.array(data)
        self.dxl_client.sync_write(self.motors, self.curr_lim, 102, 2)  # Update the current limit


class RapidNode:
    """
    The RapidNode class controls the robot's servomotors by configuring them with initial parameters such as PID gains,
    current limits, and positions, as well as reading their status (position, velocity, current). The class can 
    also load and save its configuration from/to a JSON file, and it allows controlling the servos via LEAP pose data.
    """

    def __init__(self, config_path='args/rapid_full.json', port='/dev/ttyUSB0', init_set=False):
        """
        Initializes a RapidNode instance by loading configuration, initializing motor parameters, and establishing 
        communication with the motors using the specified port.

        :param config_path: Path to the configuration file, default is 'args/rapid.json'.
        :param port: Communication port for the Dynamixel motor controller, default is '/dev/ttyUSB0'.
        :param init_set: Boolean flag to initialize the target position, default is False.
        """
        # Load configuration from the specified file path
        self.config_path = config_path
        self.config = self.load_args(config_path)
        
        # Initialize control parameters (PID gains and current limits) from the configuration
        self.jointmap = self.config['init']['jointmap']
        self.kP = np.array(self.config['init']['kP'])  # Proportional gain
        self.kI = np.array(self.config['init']['kI'])  # Integral gain
        self.kD = np.array(self.config['init']['kD'])  # Derivative gain
        self.curr_lim = np.array(self.config['init']['curr_lim'])  # Current limit
        self.init_pos = self.prev_pos = self.curr_pos = np.array(self.config['init']['init_pos'])  # Current and previous positions
        
        # Initialize motors based on the number of positions in the configuration
        self.motors = range(len(self.config['init']['init_pos']))
        
        # Try to connect to the Dynamixel motors using the specified port
        try:
            self.dxl_client = DynamixelClient(self.motors, port, 4000000)  # Initialize motor client with the port
            self.dxl_client.connect()  # Connect to the motor controller
        except Exception as e:
            print(f"Failed to connect using port {port}: {e}")
            raise

        # Configure the motors (e.g., setting torque, gains, and current limits)
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * 5, 11, 1)  # Set initial values for motors
        #self.dxl_client.set_torque_enabled(self.motors, True)  # Enable the motor torque
        self.dxl_client.sync_write(self.motors, self.kP, 84, 2)  # Set proportional gain (kP)
        self.dxl_client.sync_write(self.motors, self.kI, 82, 2)  # Set integral gain (kI)
        self.dxl_client.sync_write(self.motors, self.kD, 80, 2)  # Set derivative gain (kD)
        self.dxl_client.sync_write(self.motors, self.curr_lim, 102, 2)  # Set current limit

        # If init_set is True, initialize the motor positions
        if init_set:
            self.dxl_client.write_desired_pos(self.motors, self.init_pos)

    def load_args(self, file_path):
        """
        Load configuration from a JSON file. This method reads the configuration file and returns it as a dictionary.
        
        :param file_path: Path to the configuration file.
        :return: A dictionary containing the configuration.
        """
        with open(file_path, 'r', encoding='utf-8') as file:
            return json.load(file)

    def save_args(self, save_path=None):
        """
        Save the current configuration (PID gains, current limits, position) to a JSON file.
        
        :param save_path: Path to save the configuration, default is 'args/rapid_timestamp.json'.
        """
        if save_path is None:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            save_path = f'args/rapid_{timestamp}.json'
        
        # Construct the configuration dictionary to save
        current_config = {
            'init': {
                'kP': self.kP.tolist(),
                'kI': self.kI.tolist(),
                'kD': self.kD.tolist(),
                'curr_lim': self.curr_lim.tolist(),
                'pos': self.curr_pos.tolist()
            }
        }
        
        # Save the configuration to the specified path
        with open(save_path, 'w', encoding='utf-8') as file:
            json.dump(current_config, file, indent=4)
        print(f"Configuration saved to {save_path}")
    
    def set_servo(self, pose): 
        self.dxl_client.write_desired_pos(self.motors, pose)
        
    def set_pos(self, pose):
        """
        Set the desired position of the servomotors by receiving a RAPID pose.

        This method calculates the desired motor position by adding the received RAPID pose (a 20-element array)
        to the initial position, then adjusts the motor positions based on the robot's coordinate system,
        and finally writes the computed position to the motors.

        :param pose: A 20-element array representing the RAPID pose (relative to the initial position).
        """
        
        self.prev_pos = self.curr_pos  # Save the current position as the previous position
        temp = np.zeros(len(self.config['init']['init_pos']))
        for i in range(len(self.jointmap)):
            for map in self.jointmap[i]:
                temp[map[0]] += map[1]* pose[i]
        
        self.curr_pos = np.array([x + y for x, y in zip(self.init_pos, temp)])  # Compute the new position by adding the RAPID pose to the initial position
        # Write the desired position to the motors
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)


    def read_pos(self):
        """
        Read the current position of the servomotors.
        
        :return: The current position of the motors (adjusted for the initial position).
        """
        temp = self.dxl_client.read_pos() - self.init_pos
    
        # 初始化关节位置向量
        pose = np.zeros(len(self.jointmap))
        
        # 逆向解析：遍历 jointmap 并累加每个关节的贡献
        for i in range(len(self.jointmap)):
            for map in self.jointmap[i]:
                pose[i] += temp[map[0]] / map[1] / len(self.jointmap[i])
                
        return pose

    def read_vel(self):
        """
        Read the velocity of the servomotors.
        
        :return: The velocity of the motors.
        """
        return self.dxl_client.read_vel()

    def read_cur(self):
        """
        Read the current flowing through the servomotors.
        
        :return: The current flowing through the motors.
        """
        return self.dxl_client.read_cur()

    def setkP(self, data):
        """
        Set the proportional gain (kP) for the PID controller.
        
        :param data: The new proportional gain (kP).
        """
        self.kP = np.array(data)
        self.dxl_client.sync_write(self.motors, self.kP, 84, 2)  # Update the proportional gain

    def setkI(self, data):
        """
        Set the integral gain (kI) for the PID controller.
        
        :param data: The new integral gain (kI).
        """
        self.kI = np.array(data)
        self.dxl_client.sync_write(self.motors, self.kI, 82, 2)  # Update the integral gain

    def setkD(self, data):
        """
        Set the derivative gain (kD) for the PID controller.
        
        :param data: The new derivative gain (kD).
        """
        self.kD = np.array(data)
        self.dxl_client.sync_write(self.motors, self.kD, 80, 2)  # Update the derivative gain

    def set_curr_lim(self, data):
        """
        Set the current limit for the servomotors.
        
        :param data: The new current limit.
        """
        self.curr_lim = np.array(data)
        self.dxl_client.sync_write(self.motors, self.curr_lim, 102, 2)  # Update the current limit

class RalloNode:
    """
    RalloNode is specifically designed for torque (current) control of Dynamixel servomotors.
    It initializes the motors in current control mode and provides methods to set current limits
    and read motor status. Configuration is loaded from and saved to a JSON file.
    """

    def __init__(self, config_path='args/rallo.json', port='COM11', init_set=False):
        """
        Initializes a RalloNode instance.

        Args:
            config_path (str, optional): Path to the configuration file. Defaults to 'args/rallo.json'.
            port (str, optional): Communication port for the Dynamixel motor controller. Defaults to '/dev/ttyUSB0'.
            init_set (bool, optional): Flag to initialize motor positions. Defaults to False.
        """

        self.config_path = config_path
        self.config = self.load_args(config_path)

        self.curr = None
        self.init_pos = self.prev_pos = self.curr_pos = np.array(self.config['init']['init_pos'])  # Initial positions
        self.motors = range(len(self.config['init']['init_pos']))  # Motor IDs

        try:
            self.dxl_client = DynamixelClient(self.motors, port, 4000000)
            self.dxl_client.connect()
        except Exception as e:
            print(f"Failed to connect using port {port}: {e}")
            raise

        # Set Operating Mode to Current Control (0) -  Crucially important!
        operating_mode = np.zeros(len(self.motors), dtype=int)  # All motors to current control
        self.dxl_client.sync_write(self.motors, operating_mode, 11, 1)  # Address 11: Operating Mode

        self.dxl_client.set_torque_enabled(self.motors, True)
        # self.dxl_client.sync_write(self.motors, self.curr_lim, 102, 2)  # Set current limits

        # if init_set:
        #     self.dxl_client.write_desired_pos(self.motors, self.init_pos)

    def load_args(self, file_path):
        """Loads configuration from a JSON file."""
        with open(file_path, 'r', encoding='utf-8') as file:
            return json.load(file)

    def save_args(self, save_path=None):
        """Saves the current configuration to a JSON file."""

        if save_path is None:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            save_path = f'args/rallo_{timestamp}.json'

        current_config = {
            'init': {
                'curr_lim': self.curr_lim.tolist(),
                'pos': self.curr_pos.tolist()
            }
        }

        with open(save_path, 'w', encoding='utf-8') as file:
            json.dump(current_config, file, indent=4)
        print(f"Configuration saved to {save_path}")

    def set_curr(self, data):
        """Sets the current limits for the motors."""
        self.curr = np.array(data)
        self.dxl_client.sync_write(self.motors, self.curr, 102, 2)

    def read_pos(self):
        """Reads the current position of the servomotors."""
        return self.dxl_client.read_pos() - self.init_pos

    def read_vel(self):
        """Reads the velocity of the servomotors."""
        return self.dxl_client.read_vel()

    def read_cur(self):
        """Reads the current flowing through the servomotors."""
        return self.dxl_client.read_cur()

