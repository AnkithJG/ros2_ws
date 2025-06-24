# ROS 2 Lane Following Simulation with Turtlesim

## Overview

This project is a ROS 2 simulation of a lane-following controller using the Turtlesim package. It implements a mock lane sensor node that publishes lane offset data, and a lane follower node that uses a PID controller to adjust the turtle's velocity commands to stay centered in the lane.

Key features:
- ROS 2 Python-based publisher and subscriber nodes
- Simulation of lane offset sensor data
- PID controller implementation for lane keeping
- Periodic control commands published to `/turtle1/cmd_vel`

This project serves as a foundational platform for experimenting with decentralized behavioral planning and autonomous vehicle control in ROS 2.

## How It Works

- The **lane sensor node** simulates lateral offset data from a lane.
- The **lane follower node** subscribes to this data and computes steering commands using a PID controller.
- Steering commands are published as `geometry_msgs/Twist` messages to `/turtle1/cmd_vel` to control the turtle in Turtlesim.
- The control loop runs at 2 Hz (every 0.5 seconds) using ROS 2 timers.

### Prerequisites

- ROS 2 Humble or later installed
- `turtlesim` package installed
- Python 3 environment compatible with ROS 2
  
### Future Work and Expansion Plans

I plan to extend this project in the following ways to better align with real autonomous vehicle systems and research interests:

    Transition from Turtlesim to a realistic vehicle simulation, such as an Ackermann steering model in Gazebo.

    Integrate actual perception inputs by simulating or processing camera images for lane detection.

    Implement behavioral planning modules, including lane changes, intersection handling, and obstacle avoidance.

    Reimplement key controllers in C++ to improve performance and meet real-time constraints.

    Add comprehensive data logging and visualization tools for analyzing controller performance and vehicle behavior.

    Experiment with decentralized control algorithms and multi-agent scenarios to simulate cooperative autonomous vehicle behavior.
