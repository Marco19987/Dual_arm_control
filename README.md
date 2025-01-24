# Dual Arm Control
In this repository, you will find the code for the online calibration and control of a dual-arm robotic system. 

## Overview

This project focuses on the precise calibration and control of dual-arm robots, leveraging advanced perception techniques. Each manipulator is equipped with hand-eye mounted cameras to estimate the object's pose relative to their base frame. An Extended Kalman Filter (EKF) processes these measurements, along with force data from the robot grippers, to jointly estimate the object's pose (position and quaternion) in the common base frame and the robots' relative pose.

## Key Features

- **Robust Perception**: The EKF can incorporate various measurements from the robots' cameras and handle potential occlusions. This ensures continuous operation whether using ArUco markers, deep learning-based 6DoF methods, or object trackers.
- **Accurate Estimation**: The EKF provides precise estimation of the object's pose and the relative pose of the robots, even in dynamic environments.

## Demonstration

The following video illustrates the estimation results of the calibration matrix between robots when the object is stationary and the cameras capture its pose through ArUco markers detection.

<div style="text-align: center;">
    <img src="matlab_simulations/additional_files/plots/EKF_estimation_video.gif" alt="Dual Arm Control Demo" width="1000">
</div>

## Simulation Results

In the following, there are the simulation results to demonstrate how the calibration estimate converges to the ground truth.

<div style="text-align: center;">
    <img src="matlab_simulations/additional_files/plots/EKF_position_calibration.png" alt="Dual Arm Control System" width="1000">
</div>


<div style="text-align: center;">
    <img src="matlab_simulations/additional_files/plots/EKF_orientation_calibration.png" alt="Dual Arm Control System" width="1000">
</div>



 


