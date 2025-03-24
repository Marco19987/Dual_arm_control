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

%% Align vectors
SampleTime = 0.03;
time_vec = 0:SampleTime:time_duration;
numSteps = length(time_vec);

wrench_robots = zeros(12,numSteps);

[time_unique,IA,~] = unique(time_vec_wrench_robot1);
wrench_robots(1:6,:) = interp1(time_unique,wrench_robot1(:,IA)',time_vec)';

[time_unique,IA,~] = unique(time_vec_wrench_robot2);
wrench_robots(7:12,:) = interp1(time_unique,wrench_robot2(:,IA)',time_vec)';

measurements = zeros(7*6*2,numSteps);
measure_occlusion = ones(2*6, numSteps+1);


[time_unique,IA,~] = unique(time_vec_side1_T_object_robot1);
if(~isempty(time_unique))
    measurements(1:7,:) = interp1(time_unique,side1_T_object_robot1(:,IA)',time_vec)';
    measure_occlusion(1,IA) = 0;
end
[time_unique,IA,~] = unique(time_vec_side2_T_object_robot1);
if(~isempty(time_unique))
    measurements(8:14,:) = interp1(time_unique,side2_T_object_robot1(:,IA)',time_vec)';
    measure_occlusion(2,IA) = 0;
end

[time_unique,IA,~] = unique(time_vec_side3_T_object_robot1);
if(~isempty(time_unique))
    measurements(15:21,:) = interp1(time_unique,time_vec_side3_T_object_robot1(:,IA)',time_vec)';
    measure_occlusion(3,IA) = 0;
end

[time_unique,IA,~] = unique(time_vec_side4_T_object_robot1);
if(~isempty(time_unique))
    measurements(22:28,:) = interp1(time_unique,side4_T_object_robot1(:,IA)',time_vec)';
    measure_occlusion(4,IA) = 0;
end

[time_unique,IA,~] = unique(time_vec_side5_T_object_robot1);
if(~isempty(time_unique))
    measurements(29:35,:) = interp1(time_unique,side5_T_object_robot1(:,IA)',time_vec)';
    measure_occlusion(5,IA) = 0;    
end

[time_unique,IA,~] = unique(time_vec_side6_T_object_robot1);
if(~isempty(time_unique))
    measurements(36:42,:) = interp1(time_unique,side6_T_object_robot1(:,IA)',time_vec)';
    measure_occlusion(6,IA) = 0;
end

[time_unique,IA,~] = unique(time_vec_side1_T_object_robot2);
if(~isempty(time_unique))
    measurements(43:49,:) = interp1(time_unique,side1_T_object_robot2(:,IA)',time_vec)';
    measure_occlusion(7,IA) = 0;
end

[time_unique,IA,~] = unique(time_vec_side2_T_object_robot2);
if(~isempty(time_unique))
    measurements(50:56,:) = interp1(time_unique,side2_T_object_robot2(:,IA)',time_vec)';
    measure_occlusion(8,IA) = 0;
end

[time_unique,IA,~] = unique(time_vec_side3_T_object_robot2);
if(~isempty(time_unique))
    measurements(57:63,:) = interp1(time_unique,side3_T_object_robot2(:,IA)',time_vec)';
    measure_occlusion(9,IA) = 0;
end

[time_unique,IA,~] = unique(time_vec_side4_T_object_robot2);
if(~isempty(time_unique))
    measurements(64:70,:) = interp1(time_unique,side4_T_object_robot2(:,IA)',time_vec)';
    measure_occlusion(10,IA) = 0;
end

[time_unique,IA,~] = unique(time_vec_side5_T_object_robot2);
if(~isempty(time_unique))
    measurements(71:77,:) = interp1(time_unique,side5_T_object_robot2(:,IA)',time_vec)';
    measure_occlusion(11,IA) = 0;
end

[time_unique,IA,~] = unique(time_vec_side6_T_object_robot2);
if(~isempty(time_unique))
    measurements(78:84,:) = interp1(time_unique,side6_T_object_robot2(:,IA)',time_vec)';    
    measure_occlusion(12,IA) = 0;
end

















