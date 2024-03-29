# Multi-Disciplinary Project README

## Introduction
### This project integrates various disciplines including algorithms, image recognition, Raspberry Pi, Android, and STM32 microcontroller to develop a versatile robotic system capable of executing different commands for movement and navigation.

## System Overview
### The system operates based on input commands provided through different interfaces. These commands control the movement of the robot, which is powered by the STM32 microcontroller. Additionally, the Raspberry Pi (RPI) plays a crucial role in coordinating communication between the user interface and the robot.

---------------------------------------------------

## Command Structure
### The commands consist of a sequence of characters representing specific actions and parameters for the robot. The following is the breakdown of the command structure:

## 1st Command: Indicates the type of movement (forward/backward straight or rotation).

### w - Forward straight
### s - Backward straight
### f - Forward rotation
### b - Backward rotation
### 2nd Command: Specifies additional parameters based on the movement type.

## If the movement is straight:
### x - Without ultrasonic sensor
### z - With ultrasonic sensor

## If the movement is rotation:
### a - Left
### d - Right

## Last 3 Commands: Define the target distance or angle.

## For straight movement:
### Distance range: 000 - 255
### Example: wx100 (forward without ultrasonic at 100 units)

## For rotation:
### Angle range: 000 - 360
### Example: fa090 (forward left at 90-degree angle)

---------------------------------------------------

## Getting Started with Raspberry Pi (RPI)
### To begin using the system with the Raspberry Pi, follow these steps:

### 1. Ensure that the Raspberry Pi is powered on and connected to the STM32.
### 2. Connect to the Raspberry Pi network named "MDPGrp5".
### 3. Establish a secure shell (SSH) connection using the command:
### command line: ssh mdp@'ip_address'
### 4. Replace 'ip_address' with the actual IP address of the Raspberry Pi.
### 5. Enter the password when prompted.
### 6. Once connected, you can start writing scripts to test out commands and control the robot.

---------------------------------------------------

## Conclusion
### This multi-disciplinary project brings together various technologies and skills to create a versatile robotic system capable of executing commands for movement and navigation. With the integration of algorithms, image recognition, Raspberry Pi, Android, and STM32 microcontroller, the system offers a platform for further exploration and development in robotics and automation.
