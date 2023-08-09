# Raspberry Pi Wheel Interface

This Python script interfaces with the Raspberry Pi-based robot using ROS (Robot Operating System) messages. It subscribes to `cmd_vel` messages and controls the robot's movements accordingly.

## Prerequisites

- Raspberry Pi with ROS installed
- Python 3
- RPi.GPIO library

## Installation

 Clone this repository:

   ```sh
    git clone https://github.com/malto101/ROS_RPI_BOT.git
   ```

## Usage

1. Run the ROS master on your Raspberry Pi:
  ```sh
  roscore
  ```
2. Run the wheel interface script:
  ```sh
  python3 script.py
  ```
3. The script will listen for cmd_vel messages and control the robot's motor accordingly.

## Script Details

- The script initializes GPIO pins for motor control.
- It subscribes to the cmd_vel topic and calculates motor speeds based on the linear and angular velocities received.
- Different methods are provided to control the robot's movements, such as forward, reverse, turns, and more.

## ROS Topics
Subscribed Topic: cmd_vel
- Message Type: geometry_msgs/Twist
- Description: Controls linear and angular velocities for robot movement

## Notes
- Adjust GPIO pin numbers and motor control settings as needed for your specific robot setup.
- This script assumes familiarity with ROS and Raspberry Pi.
- Customize the script based on your robot's configuration and requirements.
