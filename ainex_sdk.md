# Personal notes on AINex SDK Documentation

## 1. Introduction to AINex

The **AINex Intelligent Vision Humanoid Robot** is a **24-degree-of-freedom humanoid robot** designed for **AI-driven navigation, object handling, and gesture-based control**. It is powered by **ROS Noetic on Ubuntu 20.04**, providing capabilities for **vision-based AI applications and robotic manipulation**.

### 1.1 Key Capabilities
- **AI Vision**: Face tracking, object recognition, gesture-based control.
- **Autonomous Navigation**: Line following, crossroad recognition, stair climbing, hurdle crossing.
- **Task Handling**: Color sorting, object placement, gripping and manipulation.
- **Self-Recovery**: Detects falls and autonomously stands up.
- **Robotic Arms**: 5 DOF mechanical arms for object manipulation.

---
## 2. SDK API Reference
### 2.1 Core SDK Modules

| Module | Functionality |
|--------|--------------|
| **`Board`** | Low-level control (servos, IMU, buzzer, LEDs) |
| **`MotionManager`** | Predefined robot actions |
| **`GaitManager`** | Walking & motion control |
| **`VisionManager`** | Object and gesture recognition using OpenCV & MediaPipe |
| **`IMU`** | Balance and movement tracking |
| **`ROS Services`** | ROS-based AI tasks |
| **`ArmControl`** | Robotic arm movement for gripping and placement |

---
## 3. AINex API and Function List

### 3.1 `Board` (Core Hardware Control)
The `Board` module handles **servo movement, IMU readings, LEDs, and the buzzer**.

#### **Functions:**
- `bus_servo_set_position(duration, [[servo_id, position]])`: Moves a servo to a position.
- `bus_servo_read_position(servo_id)`: Reads the current position of a servo.
- `bus_servo_set_id(old_id, new_id)`: Changes a servo's ID.
- `bus_servo_set_offset(servo_id, dev)`: Sets a servo deviation (calibration).
- `bus_servo_save_offset(servo_id)`: Saves the servo offset setting.
- `bus_servo_set_angle_limit(servo_id, [min_pos, max_pos])`: Sets position limits for a servo.
- `bus_servo_enable_torque(servo_id, status)`: Enables/disables torque.
- `get_imu()`: Retrieves IMU sensor data.
- `set_rgb([[index, R, G, B]])`: Controls the RGB LED color.
- `set_buzzer(freq, on_time, off_time, repeat)`: Controls the buzzer.

### 3.2 `MotionManager` (Predefined Actions)
Handles predefined **motion sequences**.

#### **Functions:**
- `run_action(action_name)`: Executes a predefined action.
- `set_servos_position(duration, [[servo_id, position]])`: Moves multiple servos to specified positions.

### 3.3 `GaitManager` (Walking & Motion Control)
Handles walking, running, and custom gait adjustments.

#### **Functions:**
- `get_gait_param()`: Retrieves current gait parameters.
- `set_step(dsp, x, y, yaw, gait_param, arm_swap, step_num)`: Controls walking motion.
- `stop()`: Stops all movement.
- `disable()`: Disables walking control.

### 3.4 `VisionManager` (Computer Vision & Object Tracking)
Handles **face tracking, color detection, and gesture recognition**.

#### **Functions:**
- `track_face()`: Tracks faces using OpenCV & MediaPipe.
- `detect_objects()`: Detects objects in camera feed.
- `track_color(color)`: Tracks objects of a specific color.

### 3.5 `ArmControl` (Robotic Arm Manipulation)
Handles robotic **gripping, lifting, and object placement**.

#### **Functions:**
- `move_gripper(servo_id, position)`: Moves the robotic arm’s gripper.
- `pick_object()`: Picks up an object.
- `place_object()`: Places an object.

---
## 4. AINex Modules: Detailed API

### 4.1 `ainex_kinematics`
Handles kinematic calculations for **walking and movement control**.

#### **Functions:**
- `GaitManager.get_gait_param()`: Returns the current gait settings.
- `GaitManager.set_step(dsp, x, y, yaw, gait_param, arm_swap, step_num)`: Executes a movement step.
- `MotionManager.run_action(action_name)`: Runs a predefined motion action.
- `MotionManager.set_servos_position(duration, [[servo_id, position]])`: Moves multiple servos to specific positions.

### 4.2 `ainex_sdk`
Handles **low-level robot operations** such as servo control, IMU access, and peripheral control.

#### **Functions:**
- `Board.bus_servo_set_position(duration, [[servo_id, position]])`: Moves a servo.
- `Board.get_imu()`: Retrieves IMU data.
- `Board.set_rgb([[index, R, G, B]])`: Controls LED color.
- `Board.set_buzzer(freq, on_time, off_time, repeat)`: Controls buzzer frequency and duration.

### 4.3 `ros_robot_controller`
Integrates **ROS-based controls for sensors, actuators, and communication**.

#### **ROS Topics & Services:**
- `/sensor/button/get_button_state`: Reads button status.
- `/ros_robot_controller/set_rgb`: Controls RGB LEDs.
- `/ros_robot_controller/set_buzzer`: Controls the buzzer.
- `/walking/command`: Sends walking commands.
- `/walking/is_walking`: Checks if the robot is currently walking.

---
## 5. AI Vision & Object Handling Examples
### 5.1 Autonomous Ball Tracking & Kicking
```python
import cv2
import numpy as np

cap = cv2.VideoCapture(0)
lower_blue = np.array([100, 50, 50])
upper_blue = np.array([140, 255, 255])

while True:
    _, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    result = cv2.bitwise_and(frame, frame, mask=mask)

    cv2.imshow("Ball Tracking", result)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
```
🔹 **Usage**: Tracks a **blue-colored ball** for autonomous kicking.

---
## 6. Conclusion
- **AINex SDK integrates AI vision, robotic arms, and motion control.**
- **Includes AI-driven navigation, color sorting, face recognition, and ball tracking.**
- **Leverages ROS for real-time robotics applications.**

