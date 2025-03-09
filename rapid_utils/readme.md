# Rapid Utils Documentation

## Overview

`rapid_utils` is a Python-based utility designed to facilitate the control of robots using Dynamixel motors. This module enables precise configuration and management of servomotor parameters, allowing for seamless integration with gesture-based control systems like LEAP motion.

The utility contains the following primary components:

- `rapidnode.py`: Core implementation for managing servomotor parameters and configurations.
- `dynamixel_client.py`: Communication interface with Dynamixel motor controllers.
- `__init__.py`: Initializes the `rapid_utils` module.

## Folder Structure

```
rapid_utils/
├── __init__.py
├── dynamixel_client.py
├── rapidnode.py
```

## Features

### `RapidNode`

The `RapidNode` class is the primary interface for controlling servomotors. Key functionalities include:

- Loading and saving configuration parameters (e.g., PID gains, current limits, initial positions).
- Managing servomotor torque and gains.
- Reading and writing motor states (position, velocity, current).

### `LeapNode`

The `LeapNode` class extends the functionality of `RapidNode`, providing integration with LEAP motion-based gesture control. It manages similar motor parameters while facilitating gesture-based control.

## Installation

1. Clone the repository or copy the `rapid_utils` folder to your project directory.
2. Ensure the following dependencies are installed:
   - `numpy`
   - `json`
   - `datetime`
   - `dynamixel_sdk`
3. Connect the Dynamixel motor controller to your system.

## Usage

### Initialization

Create an instance of `RapidNode` or `LeapNode` to initialize the system:

```python
from rapid_utils.rapidnode import RapidNode

node = RapidNode(config_path='args/rapid.json', port='/dev/ttyUSB0', init_set=True)
# node = RapidNode(config_path='args/rapid.json', port='com13', init_set=True)
```

### Configuration

#### Loading Configuration

Load a JSON configuration file:

```python
node.load_args('args/rapid.json')
```

#### Saving Configuration

Save the current configuration to a file:

```python
node.save_args('args/rapid_saved.json')
```

### Motor Control

#### Setting Positions

Set the desired position for servomotors:

```python
pose = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 170, 180, 190, 200]  # Example pose values for 20 joints
node.set(pose)
```

#### Reading Motor States

Read motor states such as position, velocity, and current:

```python
position = node.read_pos()
velocity = node.read_vel()
current = node.read_cur()
```

#### Adjusting PID Gains

Update PID gains dynamically:

```python
node.setkP([50, 50, 50])  # Proportional gain
node.setkI([30, 30, 30])  # Integral gain
node.setkD([10, 10, 10])  # Derivative gain
```

## Configuration File Format

The configuration file is a JSON file specifying initial motor parameters:

```json
{
    "init": {
        "kP": [50, 50, 50],
        "kI": [30, 30, 30],
        "kD": [10, 10, 10],
        "curr_lim": [100, 100, 100],
        "pos": [0, 0, 0]
    }
}
```

## Notes

- Ensure the specified port (e.g., `/dev/ttyUSB0`) is correctly configured for your system.
- The motor controller should be properly connected before initializing the `RapidNode` or `LeapNode` instance.
- Save configurations regularly to avoid data loss.

## License

This utility is open-source and available under the MIT License.

---

For further assistance, please refer to the code comments or contact the developers.

