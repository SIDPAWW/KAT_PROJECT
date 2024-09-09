Obstacle Avoidance
Project Overview

This project simulates a drone capable of navigating through a series of waypoints while avoiding obstacles. The drone has vertical takeoff and landing capabilities, and obstacle avoidance is managed using the 3D Vector Field Histogram (VFH) algorithm.

The project utilizes various MATLAB functions and Simulink models to control the UAV's movement and ensure safe navigation through complex environments.

Key Features

Waypoint Navigation: The drone follows predefined waypoints during its flight, adjusting its path in real time to avoid obstacles.

Vertical Takeoff and Landing (VTOL): The drone can take off and land vertically, ensuring smooth operations in restricted environments.

Obstacle Avoidance with 3D VFH: The 3D Vector Field Histogram algorithm helps the drone detect and steer around obstacles during flight.

Functions and Algorithms Used

This project uses several MATLAB functions for trajectory generation and control:

controllerVFH3D: Implements the 3D VFH algorithm for obstacle avoidance.

uavDubinsConnection: Establishes a connection using the Dubins path for UAV trajectory.

uavDubinsPathSegment: Generates segments of Dubins paths for UAV motion.

minsnappolytraj: Computes minimum-snap polynomial trajectories for UAV motion planning.

minjerkpolytraj: Generates minimum-jerk polynomial trajectories for smoother UAV movements.

waypointTrajectory: Manages waypoint following for the UAV.

polynomialTrajectory: Provides general polynomial trajectory generation for complex flight paths.

Setup Instructions

Requirements:

MATLAB (Recommended version: R2023a or later)

Simulink

Required Toolboxes:

UAV Toolbox (if applicable)

Navigation Toolbox (for path planning)

Robotics System Toolbox (for control and trajectory generation)

Installation

Clone this repository to your local machine:

bash
Copy code
git clone https://github.com/SIDPAWW/obstacle-avoidance-uav.git


Open MATLAB and navigate to the project folder.

Ensure the necessary toolboxes are installed (listed above).

Open the Simulink model file (uav_model.slx) and properly link all required blocks.

Running the Simulation
Define your set of waypoints and obstacles within the environment setup section of the code.
Open the Simulink model and run the simulation.
The UAV will autonomously navigate through the waypoints while avoiding any detected obstacles.

Usage

You can customize the waypoints and obstacles by editing the input arrays in the script.
Use the control panel in the Simulink model to adjust parameters such as lookahead distance, trajectory smoothness, and obstacle sensitivity.

Contributors

Siddhi Pawar (SIDPAWW)
