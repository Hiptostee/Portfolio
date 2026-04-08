# Vision-Based Target Tracking & Pan-Tilt Control

A demo video is available at [this link](https://photos.app.goo.gl/oTzyxrED9PwBCRbX7).

## Overview

This project combines OpenCV-based object tracking with Arduino servo control to create a real-time pan-tilt tracking system. A camera detects a colored target, estimates its position in the frame, and sends that data to a microcontroller that keeps the target centered through smooth two-axis actuation.

The technical focus is on vision processing, serial communication, and closed-loop control on simple hardware.

## Technical Highlights

- OpenCV pipeline for color-based object detection, contour extraction, and target localization
- Python-to-Arduino serial communication for low-latency control updates
- Two-axis pan-tilt control with bounded step sizes to reduce oscillation and jerk
- Configurable tracking parameters for camera resolution, servo limits, movement steps, and alignment offsets
- Real-time visual feedback with bounding boxes and crosshairs over the camera stream

## System Flow

1. A webcam captures live video frames.
2. The Python vision pipeline converts frames to HSV, applies color masks, reduces noise, and finds the dominant contour.
3. The target position is converted into `X,Y,W,H` data and sent over serial to the Arduino.
4. The Arduino maps the incoming coordinates to pan and tilt angles, applies correction and step limits, and updates the servos.
5. The platform continuously re-centers the target as it moves through the frame.

## Hardware and Software

- Python with OpenCV for detection and tracking
- Arduino Nano for servo control
- Two-servo pan-tilt mechanism
- USB serial connection between host computer and microcontroller
- Webcam or Raspberry Pi camera feed

## Future Improvements

- Add multi-target selection and prioritization
- Build a lightweight interface for live parameter tuning
- Add a calibration routine for servo limits and camera offsets
- Replace step-based control with a PID controller for smoother tracking

This project was developed in collaboration with Francis Leahy (Binghamton University) and Matthew Russo (University at Buffalo).
