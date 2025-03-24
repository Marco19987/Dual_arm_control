close all
clear variables
clc

rosbag_dir = "rosbag2_2025_03_24-13_05_22/";
bag = ros2bagreader(rosbag_dir);

% msgs = readMessages(bag);

time_duration = bag.EndTime - bag.StartTime;
start_time = bag.StartTime;
end_time = bag.EndTime;

% ekf object pose
ekf_object_pose_bag = select(bag,"Topic","/ekf/object_pose");
[time_vec_ekf_obj_pose,ekf_pose] = read_poseStamped(ekf_object_pose_bag);

% ekf object twist
ekf_object_twist_bag = select(bag,"Topic","/ekf/object_twist");
[time_vec_ekf_obj_twist,ekf_twist] = read_twistStamped(ekf_object_twist_bag);

% ekf b1Tb2
b1Tb2_pose_bag = select(bag,"Topic","/ekf/b1Tb2_filtered");
[time_vec_ekf_b1Tb2,b1Tb2_pose] = read_poseStamped(b1Tb2_pose_bag);

% wrench robot 1
wrench_robot1_bag_aft_piv = select(bag,"Topic","/iiwa/wsg50/wrench_rotated_after_pivoting");
[time_vec_wrench_robot1,wrench_robot1] = read_wrenchStamped(wrench_robot1_bag_aft_piv);

% wrench robot 2
wrench_robot2_bag_aft_piv = select(bag,"Topic","/yaskawa/wsg32/wrench_rotated_after_pivoting");
[time_vec_wrench_robot2,wrench_robot2] = read_wrenchStamped(wrench_robot2_bag_aft_piv);

% aruco pose measures - robot1
side1_T_object_robot1_bag = select(bag,"Topic","/robot1/resin_block_1/side1_T_object/pose");
[time_vec_side1_T_object_robot1,side1_T_object_robot1] = read_poseStamped(side1_T_object_robot1_bag);

side2_T_object_robot1_bag = select(bag,"Topic","/robot1/resin_block_1/side2_T_object/pose");
[time_vec_side2_T_object_robot1,side2_T_object_robot1] = read_poseStamped(side2_T_object_robot1_bag);

side3_T_object_robot1_bag = select(bag,"Topic","/robot1/resin_block_1/side3_T_object/pose");
[time_vec_side3_T_object_robot1,side3_T_object_robot1] = read_poseStamped(side3_T_object_robot1_bag);

side4_T_object_robot1_bag = select(bag,"Topic","/robot1/resin_block_1/side4_T_object/pose");
[time_vec_side4_T_object_robot1,side4_T_object_robot1] = read_poseStamped(side4_T_object_robot1_bag);

side5_T_object_robot1_bag = select(bag,"Topic","/robot1/resin_block_1/side5_T_object/pose");
[time_vec_side5_T_object_robot1,side5_T_object_robot1] = read_poseStamped(side5_T_object_robot1_bag);

side6_T_object_robot1_bag = select(bag,"Topic","/robot1/resin_block_1/side6_T_object/pose");
[time_vec_side6_T_object_robot1,side6_T_object_robot1] = read_poseStamped(side6_T_object_robot1_bag);
 
% aruco pose measures - robot2

side1_T_object_robot2_bag = select(bag,"Topic","/robot2/resin_block_1/side1_T_object/pose");
[time_vec_side1_T_object_robot2,side1_T_object_robot2] = read_poseStamped(side1_T_object_robot2_bag);

side2_T_object_robot2_bag = select(bag,"Topic","/robot2/resin_block_1/side2_T_object/pose");
[time_vec_side2_T_object_robot2,side2_T_object_robot2] = read_poseStamped(side2_T_object_robot2_bag);

side3_T_object_robot2_bag = select(bag,"Topic","/robot2/resin_block_1/side3_T_object/pose");
[time_vec_side3_T_object_robot2,side3_T_object_robot2] = read_poseStamped(side3_T_object_robot2_bag);

side4_T_object_robot2_bag = select(bag,"Topic","/robot2/resin_block_1/side4_T_object/pose");
[time_vec_side4_T_object_robot2,side4_T_object_robot2] = read_poseStamped(side4_T_object_robot2_bag);

side5_T_object_robot2_bag = select(bag,"Topic","/robot2/resin_block_1/side5_T_object/pose");
[time_vec_side5_T_object_robot2,side5_T_object_robot2] = read_poseStamped(side5_T_object_robot2_bag);

side6_T_object_robot2_bag = select(bag,"Topic","/robot2/resin_block_1/side6_T_object/pose");
[time_vec_side6_T_object_robot2,side6_T_object_robot2] = read_poseStamped(side6_T_object_robot2_bag);










