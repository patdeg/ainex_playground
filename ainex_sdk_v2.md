# AINex Humanoid Robot SDK Documentation for GenAI Programming

## Introduction

This documentation is designed for a GenAI to facilitate the development of mini-programs for the HiWonder AINex ROS Educational Humanoid Robot. It offers a comprehensive guide to the functionalities of the AINex SDK, with a focus on controlling the robot's serial servos, gait, motion, and onboard peripherals using Python on a Raspberry Pi 5.

### HiWonder AINex ROS Educational Humanoid Robot

The HiWonder AINex ROS Educational Humanoid Robot is a programmable humanoid robot intended for education and research. Its modular design allows for easy assembly and customization. The robot is equipped with various sensors, including an IMU, camera, and touch sensors, enabling it to perceive and interact with its environment. The AINex robot operates on the Robot Operating System (ROS), a versatile framework for writing robot software.

This documentation specifically targets the standard edition of the AINex robot with hands for object manipulation. This version includes two movable mechanical hands, each with three degrees of freedom, allowing the robot to grasp and manipulate objects. The hands are controlled by serial servos, programmable using the AINex SDK.

## Modules and Functions

### `Board` Module

The `Board` module is the primary interface for interacting with the AINex robot's hardware. It provides functions for controlling the robot's serial servos and onboard peripherals such as the LED, RGB lights, buzzer, and IMU sensor.

**Functions:**

*   `set_led(state)`: Controls the state of the onboard LED.

    *   `state`: 1 to turn on the LED, 0 to turn it off.
    *   **Example:**

        ```python
        from ainex_sdk import Board

        board = Board()
        board.set_led(1)  # Turn on the LED
        ```

*   `set_rgb(data)`: Sets the color of the onboard RGB lights.

    *   `data`: A list of lists, where each inner list contains four values: the index of the RGB light, and the red, green, and blue color values (0-255).
    *   **Example:**

        ```python
        from ainex_sdk import Board

        board = Board()
        board.set_rgb()  # Set the first RGB light to red
        ```

*   `set_buzzer(freq, on_time, off_time, repeat)`: Controls the onboard buzzer.

    *   `freq`: The frequency of the buzzer sound in Hz.
    *   `on_time`: The duration for which the buzzer should be on in seconds.
    *   `off_time`: The duration for which the buzzer should be off in seconds.
    *   `repeat`: The number of times the buzzer sound should be repeated.
    *   **Example:**

        ```python
        from ainex_sdk import Board

        board = Board()
        board.set_buzzer(2400, 0.1, 0.1, 1)  # Sound the buzzer at 2400Hz for 0.1 seconds
        ```

*   `get_imu()`: Returns the IMU sensor data.

    *   **Example:**

        ```python
        from ainex_sdk import Board

        board = Board()
        imu_data = board.get_imu()
        print(imu_data)
        ```

### `Serial Servo` Module

The `Serial Servo` module provides functions for controlling the robot's serial servos. You can set the position, speed, and other parameters of each servo individually.

**Functions:**

*   `bus_servo_set_position(duration, data)`: Sets the position of one or more servos.

    *   `duration`: The duration of the movement in seconds.
    *   `data`: A list of lists, where each inner list contains two values: the servo ID and the target position (0-1000).
    *   **Example:**

        ```python
        from ainex_sdk import Board

        board = Board()
        board.bus_servo_set_position(0.5,)  # Set servo ID 23 to position 500
        ```

*   `bus_servo_set_speed(servo_id, speed)`: Sets the speed of a servo.

    *   `servo_id`: The ID of the servo.
    *   `speed`: The target speed.
    *   **Example:**

        ```python
        from ainex_sdk import Board

        board = Board()
        board.bus_servo_set_speed(24, 100)  # Set servo ID 24 to speed 100
        ```

*   `bus_servo_read_position(servo_id)`: Returns the current position of a servo.

    *   `servo_id`: The ID of the servo.
    *   **Example:**

        ```python
        from ainex_sdk import Board

        board = Board()
        position = board.bus_servo_read_position(23)
        print(position)
        ```

### `Gait` Module

The `Gait` module provides functions for controlling the robot's gait. You can make the robot walk, turn, and adjust its gait parameters.

**Functions:**

*   `move(step_mode, x, y, yaw)`: Makes the robot walk or turn.

    *   `step_mode`: The type of movement: 1 for walking forward, 2 for walking backward, 3 for turning left, and 4 for turning right.
    *   `x`: The step length in the x-direction (meters).
    *   `y`: The step length in the y-direction (meters).
    *   `yaw`: The rotation angle in degrees.
    *   **Example:**

        ```python
        from ainex_kinematics.gait_manager import GaitManager

        gait_manager = GaitManager()
        gait_manager.move(1, 0.02, 0, 0)  # Walk forward with a step length of 0.02 meters
        ```

*   `stop()`: Stops the robot's movement.

    *   **Example:**

        ```python
        from ainex_kinematics.gait_manager import GaitManager

        gait_manager = GaitManager()
        gait_manager.stop()
        ```

### `Motion` Module

The `Motion` module provides functions for controlling the robot's motion. You can make the robot perform pre-defined actions or create your own custom actions.

**Functions:**

*   `run_action(action_name)`: Executes a pre-defined action.

    *   `action_name`: The name of the action.
    *   **Example:**

        ```python
        from ainex_kinematics.motion_manager import MotionManager

        motion_manager = MotionManager()
        motion_manager.run_action('left_shot')  # Execute the 'left_shot' action
        ```

*   `set_servos_position(duration, data)`: Sets the position of multiple servos.

    *   `duration`: The duration of the movement in seconds.
    *   `data`: A list of lists, where each inner list contains two values: the servo ID and the target position (0-1000).
    *   **Example:**

        ```python
        from ainex_kinematics.motion_manager import MotionManager

        motion_manager = MotionManager()
        motion_manager.set_servos_position(500, [,])  # Set servos 23 and 24 to position 500
        ```

## Additional Examples

### Controlling the Robot's LED

```python
from ainex_sdk import Board
import time

# Initialize the board
board = Board()

# Blink the LED at 5Hz
while True:
    board.set_led(1)
    time.sleep(0.1)
    board.set_led(0)
    time.sleep(0.1)
```

### Making the Robot Walk in a Square

```python
from ainex_kinematics.gait_manager import GaitManager
import time

# Initialize the gait manager
gait_manager = GaitManager()

# Walk in a square
for _ in range(4):
    gait_manager.move(1, 0.5, 0, 0)  # Walk forward for 0.5 meters
    time.sleep(2)
    gait_manager.move(3, 0, 0, 90)  # Turn left by 90 degrees
    time.sleep(1)

# Stop the robot
gait_manager.stop()
```

### Making the Robot Greet

```python
from ainex_kinematics.motion_manager import MotionManager

# Initialize the motion manager
motion_manager = MotionManager()

# Run a pre-defined action named 'greet'
motion_manager.run_action('greet')
```

## Conclusion

This documentation provides a foundational overview of the AINex Humanoid Robot SDK, tailored for GenAI programming assistance. It covers essential modules and functions for controlling the robot's hardware and movements, including the `Board`, `Serial Servo`, `Gait`, and `Motion` modules. By understanding these functionalities and utilizing the provided examples, the GenAI can effectively assist in the development of mini-programs for the HiWonder AINex ROS Educational Humanoid Robot, enabling it to perform various tasks and interact with its environment.
