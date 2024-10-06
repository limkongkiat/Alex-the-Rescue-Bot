# Alex to the Rescue: A Search and Rescue Robotic Vehicle
## Project Overview
"Alex to the Rescue" is a robotic vehicle built for search and rescue operations, developed by Team B03-1A as part of the CG2111A Engineering Principles and Practice course at NUS (Semester 2, 2023/2024). Alex is designed to navigate a maze, detect obstacles, and identify human figures in distress using advanced sensors, including a color sensor and Lidar. The project demonstrates the integration of robotics, embedded systems, and programming to solve real-world problems.

## Features and Functionality
Search and Rescue Robot: Alex navigates a maze to locate and classify human figures based on color.
Obstacle Detection and Mapping: Utilizes Lidar for mapping surroundings and detecting obstacles.
Remote Control: Controlled remotely via a laptop that sends commands to a Raspberry Pi, which communicates with an Arduino Mega.

## System Architecture
Hardware Components:
Motors, color sensor, Lidar, Raspberry Pi, Arduino Mega, ultrasonic sensor, gyroscope, and power supply.
Custom 3D-printed holder to optimize sensor placement.
Heat sinks used to manage heat from the Raspberry Pi during extended operations.

## Key Sensors:
MPU-6050 Gyroscope: To detect motion and adjust the robot's orientation accurately.
HC-SR04 Ultrasonic Sensor: Measures the distance to obstacles and prevents collisions.
Lidar System: Maps the environment using Hector SLAM for visualization.

## Software Components:
Arduino Mega: Controls movement, receives commands, and processes sensor data.
Raspberry Pi (RPi): Acts as the intermediary between the laptop and Arduino, processing Lidar data and relaying commands.
Laptop Interface: Commands are sent from the laptop to navigate the robot, and Lidar visualization is displayed via VNC Viewer.

## Technical Details
Programming Languages Used: C/C++, Python.
Communication: Implemented a packet structure (TPacket) for data exchange between Raspberry Pi and Arduino.

## Movement and Control:
Controlled via commands that allow forward/backward movement and turning.
Color Detection: Uses a color sensor to classify objects into different categories (e.g., "green" for healthy, "red" for injured).

## Challenges and Solutions
Weight Distribution: Initially, Alex struggled with turning due to weight imbalance. We adjusted the layout of components to lower the center of gravity, which significantly improved maneuverability.
Color Sensor Reliability: The color sensor's readings fluctuated due to inconsistent connections. To address this, we normalized the readings and relied on ratio-based detection for more consistent results.

## Lessons Learned
Adaptability is Key: The need to change our color detection approach highlighted the importance of adaptability in responding to real-world conditions.
Fundamental Troubleshooting: A significant issue with the gyroscope revealed that simple problems (e.g., loose wires) should be checked before diving into complex debugging.
