# 🎯 Vision-Based Target Tracking & Pan-Tilt Control

A demo video is available at [this link](https://photos.app.goo.gl/oTzyxrED9PwBCRbX7).

## 💡 Skills & Technologies

This project showcases expertise in:

- **Computer Vision:** OpenCV (Object Detection, Tracking, Filtering)
- **Microcontroller Programming:** Arduino (C++)
- **Robotics & Control:** Servo Control, Real-time Tracking Systems
- **Hardware Interfacing:** Serial Communication (Python-Arduino), Camera Control
- **Image Processing:** HSV Color Space, Morphological Operations, Contours
- **Languages:** Python, C++ (Arduino)
- **Data Communication:** Serial Port Management
- **Numerical Methods:** Mapping, Constraining, Scaling Functions
- **Version Control:** Git, GitHub

## ✨ Project Overview

This project combines computer vision with precision servo control to create a real-time target tracking system. A camera detects and tracks colored objects, then communicates with an Arduino Nano to adjust a two-axis pan-tilt mechanism and keep the target centered. The technical focus is on vision processing, serial communication, and smooth closed-loop actuation.

## ✅ Features

- **Real-time Object Tracking:** Utilizes OpenCV in Python to detect and follow objects based on their color in live camera feed. 📹

- **HSV Color Filtering:** Flexible color range definitions (e.g., Red, Orange, Yellow, Blue, Purple) for versatile target identification. 🎨

- **Adaptive Servo Control:** Arduino-based logic smoothly adjusts two servos (horizontal and vertical) to center the detected object in the camera's view. 🤖

- **Dynamic Tracking Correction:** Implements error calculation and stepped adjustments to prevent jerky movements and ensure stable tracking. 🔄

- **Configurable Parameters:** Easy-to-tune parameters for camera resolution, servo limits, movement steps, and alignment offsets. ⚙️

- **Serial Communication:** Robust data transfer between the Python vision system and the Arduino microcontroller. 📡

- **Visual Feedback:** Displays the detected object with a bounding box and crosshairs on the video feed. 🎯

## 🔍 How It Works

This project functions as a closed-loop control system:

1.  **Camera Input:** A webcam captures live video frames. 🖼️

2.  **Python Vision Processing:**
    - Each frame is converted from BGR to HSV color space.

    - Color masks are applied to isolate the target object (e.g., a red object).

    - Morphological operations (erode, dilate) clean up noise in the mask.

    - Contour detection identifies the largest blob corresponding to the target.

    - The **center coordinates (X, Y)** and **dimensions (width, height)** of the largest detected object are calculated.

    - This data (`X,Y,W,H`) is then sent over a serial connection to the Arduino.

3.  **Arduino Servo Control:**
    - The Arduino receives the object's `X` and `Y` coordinates from Python.

    - These coordinates are normalized (0 to 1) relative to the camera's resolution.

    - Optional **scaling** is applied to the normalized values for non-linear servo response, allowing for finer control around the center.

    - The scaled coordinates are mapped to target angles for the horizontal (`bottomServo`) and vertical (`topServo`) servos.

    - Small **correction offsets** can be applied for fine-tuning the pan-tilt response.

    - The current servo angles are compared to the target angles to calculate `error`.

    - Servos are adjusted in small, constrained steps (`maxStepBottom`, `maxStepTop`) to smoothly move the platform towards the target.

    - Real-time debug information (face position, servo angles) is sent back to Python (though not explicitly read by the Python script in the provided code).

4.  **Pan-Tilt Tracking:** The two servos control the physical orientation of the pan-tilt platform, constantly adjusting to keep the detected target centered. 🌟

## ⚙️ Components Used

- **Computer/Raspberry Pi:** Runs the Python vision script. 💻

- **Webcam:** Captures video feed for object detection. 🎥

- **Arduino Nano (or similar):** Microcontroller for servo control. 🔌

- **2x Servo Motors:**
  - One for **horizontal** (pan) movement.

  - One for **vertical** (tilt) movement.

- **Pan-Tilt Mount or Payload:** The code can drive a two-axis tracking platform for different mounted hardware. ⚡

- **USB Cable:** For serial communication between computer and Arduino. 🔗

- **Jumper Wires & Breadboard:** For connections. 🧰

## 🚀 Getting Started

### 📦 Installation

1.  **Python Libraries:**

    ```
    pip install opencv-python pyserial numpy

    ```

2.  **Arduino IDE:** Download and install the [Arduino IDE](https://www.arduino.cc/en/software).

3.  **Servo Library:** The `<Servo.h>` library is typically pre-installed with the Arduino IDE.

### 📝 Setup

1.  **Arduino Code:**
    - Open the `arduino_code.ino` (or equivalent) in Arduino IDE.

    - **Verify Servo Connections:** Ensure `bottomServo` and `topServo` are attached to the correct pins (A2 and A3 in the code).

    - Upload the code to your Arduino Nano.

2.  **Python Code:**
    - **`arduino_port`:** Modify `arduino_port = '/dev/cu.usbserial-110'` to match your Arduino's serial port. You can usually find this in the Arduino IDE under `Tools > Port`.

    - **Camera Index:** `cv2.VideoCapture(0)` assumes your default webcam. If you have multiple cameras, you might need to change `0` to `1`, `2`, etc.

    - **Color Ranges:** Adjust `lower_color`/`upper_color` arrays (e.g., `lower_red1`, `upper_red1`, `lower_red2`, `upper_red2` for red) to accurately detect your target object's HSV color range. You can use an HSV color picker tool or experiment. 🌈

3.  **Physical Setup:**
    - Mount your camera securely.

    - Attach the two servos to your pan-tilt base such that one controls horizontal rotation and the other controls vertical tilt.

    - Wire the servos to the Arduino according to the code (A2 and A3 for signal, plus power and ground).

## 🎮 Usage

1.  **Run Python Script:**

    ```
    python opencv.py

    ```

2.  **Observe:** A window named "Color Tracker" will pop up, showing the camera feed with bounding boxes and circle at the center of the detected object.

3.  **Aiming:** Point your camera towards the object you want to track. The servos connected to the Arduino will automatically adjust to aim towards the center of the detected object.

## 🎛️ Configurable Parameters

### Python (`opencv.py`)

- `arduino_port`: Serial port for Arduino communication.

- `baud_rate`: Baud rate for serial communication (must match Arduino).

- `lower_color`, `upper_color` (and `lower_red1`, etc.): HSV ranges for color detection.

### Arduino (`main.ino`)

- `cameraWidth`, `cameraHeight`: Expected camera resolution (match Python).

- `servoMinAngle`, `servoMaxAngle`: Physical angle limits of your servos.

- `maxStepBottom`, `maxStepTop`: Maximum angle change per loop for smooth movement.

- `angleBottomInit`, `angleTopInit`: Initial starting angles for the servos.

- `correctionOffsetX`, `correctionOffsetY`: Fine-tune fixed offset for pan-tilt alignment.

- `useScaling`: `true` or `false` to enable/disable non-linear scaling of input.

## 🚧 Future Enhancements

- **Payload Integration:** Add logic for whatever device or mechanism is mounted on the pan-tilt platform once a target is locked. 🔥

- **Multiple Object Tracking:** Track and prioritize multiple targets.

- **User Interface:** Implement a simple GUI for adjusting parameters in real-time without editing code. 🖥️

- **Calibration Routine:** Develop an automated routine to calibrate servo limits and camera offsets.

- **PID Control:** Implement a PID (Proportional-Integral-Derivative) controller for even smoother and more accurate servo movements. 📈

This project was made in collaboration with Francis Leahy (Binghamton University) and Matthew Russo (University at Buffalo)
