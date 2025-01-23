# Dual Arm Control

In this repository, you will find the code for the online calibration and control of a dual-arm robotic system. 

Regarding perception, the two manipulators, equipped with hand-eye mounted cameras, estimate the object's pose w.r.t. their base frame. An Extended Kalman Filter (EKF) uses these measurements, along with the forces measured by the robot grippers, to jointly estimate the object's pose (position and quaternion), in the common base frame, and the robots' relative pose.
The EKF can incorporate various measurements from the robots' cameras and handle potential occlusions. This ensures that the filter continues to function regardless of whether the poses are estimated using ArUco markers, deep learning-based 6DoF methods, or object trackers.

The following video shows the estimation result of the calibration matrix between robots when the object is stationary and the cameras capturing its pose
through ArUco markers detection. 


<div style="text-align: center;">
    <img src="matlab_simulations/additional_files/plots/EKF_estimation_video.gif" alt="Dual Arm Control Demo" width="1000">
</div>

## Position Calibration

<div style="text-align: center;">
    <img src="matlab_simulations/additional_files/plots/EKF_position_calibration.png" alt="Dual Arm Control System" width="1000">
</div>


## Orientation Calibration

<div style="text-align: center;">
    <img src="matlab_simulations/additional_files/plots/EKF_orientation_calibration.png" alt="Dual Arm Control System" width="1000">
</div>







 


