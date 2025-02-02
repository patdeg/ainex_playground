# Personal notes on AINex SDK Documentation

## **1. Introduction to AINex**

The **AINex Intelligent Vision Humanoid Robot** is a **24-degree-of-freedom humanoid robot** designed for **AI-driven navigation, object handling, and gesture-based control**. It is powered by **ROS Noetic on Ubuntu 20.04**, providing capabilities for **vision-based AI applications and robotic manipulation**.

### **1.1 Key Capabilities**
- **AI Vision**: Face tracking, object recognition, gesture-based control.
- **Autonomous Navigation**: Line following, crossroad recognition, stair climbing, hurdle crossing.
- **Task Handling**: Color sorting, object placement, gripping and manipulation.
- **Self-Recovery**: Detects falls and autonomously stands up.
- **Robotic Arms**: 5 DOF mechanical arms for object manipulation.

---

## **2. SDK API Reference**
### **2.1 Core SDK Modules**

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

## **3. AINex API and Function List**

### **3.1 `Board` (Core Hardware Control)**
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

#### **Examples:**
**Example: Moving a Servo**
```python
from ainex_sdk import Board
import time

servo = Board()
servo_id = 23
position = 600
duration = 0.5

servo.bus_servo_set_position(duration, [[servo_id, position]])
time.sleep(duration)
```

**Example: Reading IMU Data**
```python
from ainex_sdk import Board
import time

board = Board()
board.enable_reception()

while True:
    imu_data = board.get_imu()
    if imu_data:
        print("IMU Data:", imu_data)
    time.sleep(0.5)
```

---

### **3.2 `MotionManager` (Predefined Actions)**
Handles predefined **motion sequences**.

#### **Functions:**
- `run_action(action_name)`: Executes a predefined action.
- `set_servos_position(duration, [[servo_id, position]])`: Moves multiple servos to specified positions.

#### **Examples:**
**Example: Running Predefined Movements**
```python
from ainex_kinematics.motion_manager import MotionManager
import time

motion_manager = MotionManager('/home/ubuntu/software/ainex_controller/ActionGroups')
motion_manager.run_action('left_shot')
```

---

### **3.3 `GaitManager` (Walking & Motion Control)**
Handles walking, running, and custom gait adjustments.

#### **Functions:**
- `get_gait_param()`: Retrieves current gait parameters.
- `set_step(dsp, x, y, yaw, gait_param, arm_swap, step_num)`: Controls walking motion.
- `stop()`: Stops all movement.
- `disable()`: Disables walking control.

#### **Examples:**
**Example: Making the Robot Walk**
```python
from ainex_kinematics.gait_manager import GaitManager

gait_manager = GaitManager()
gait_param = gait_manager.get_gait_param()
gait_manager.set_step([400, 0.2, 0.02], 0.02, 0, 0, gait_param, arm_swap=30, step_num=5)
```

---

### **3.4 `VisionManager` (Computer Vision & Object Tracking)**
Handles **face tracking, color detection, and gesture recognition**.

#### **Examples:**
**Example: Face Detection**
```python
import cv2
import mediapipe as mp

mp_face_detection = mp.solutions.face_detection
cap = cv2.VideoCapture(0)

with mp_face_detection.FaceDetection(min_detection_confidence=0.5) as face_detection:
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        results = face_detection.process(frame)
        if results.detections:
            for detection in results.detections:
                print(detection)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()
```

---

## **4. AI Vision & Object Handling Examples**
### **4.1 Autonomous Ball Tracking & Kicking**
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

## **5. Conclusion**
- **AINex SDK integrates AI vision, robotic arms, and motion control.**
- **Includes AI-driven navigation, color sorting, face recognition, and ball tracking.**
- **Leverages ROS for real-time robotics applications.**

🚀 **Use this documentation as a reference for developing with AINex!**
