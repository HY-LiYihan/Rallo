import numpy as np
import json
from datetime import datetime
from dynamixel_client import *


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


if __name__ == '__main__':
    # Example Usage
    try:
        rallo_node = RalloNode(config_path='args/rallo.json')

        #  Example:  Setting new current limits (torque control)
        new_current_limits = [100, 100, 100, 100, 100, 100]  # Example: 100 mA (adjust as needed)
        rallo_node.set_curr(new_current_limits)
        print("Current limits set.")
        while True:
            # Example:  Read motor data
            positions = rallo_node.read_pos()
            velocities = rallo_node.read_vel()
            currents = rallo_node.read_cur()

            print("Positions:", positions)
            print("Velocities:", velocities)
            print("Currents:", currents)
            time.sleep(0.1) 
        # Read motor data
        positions = rallo_node.read_pos()
        velocities = rallo_node.read_vel()
        currents = rallo_node.read_cur()

        print("Positions:", positions)
        print("Velocities:", velocities)
        print("Currents:", currents)


    except Exception as e:
        print("Error:", e)