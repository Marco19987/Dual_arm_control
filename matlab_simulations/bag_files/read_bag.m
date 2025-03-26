close all
clear variables
clc

% rosbag_dir = "rosbag2_2025_03_25-16_33_10_calib_only";
% rosbag_dir = "rosbag2_2025_03_25-16_38_53";
rosbag_dir = "rosbag2_2025_03_25-14_36_38_only_calib"
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
t_init_max = max([time_vec_ekf_obj_pose(1) time_vec_ekf_b1Tb2(1) time_vec_ekf_obj_twist(1)]);
t_final_min = min([time_vec_ekf_obj_pose(end) time_vec_ekf_b1Tb2(end) time_vec_ekf_obj_twist(end)]);
time_vec = t_init_max:SampleTime:t_final_min;
numSteps = length(time_vec);
n_pose_measures = 6;

wrench_robots = zeros(12,numSteps);

% [time_unique,IA,~] = unique(time_vec_wrench_robot1);
wrench_robots(1:6,:) = interp1(time_vec_wrench_robot1,wrench_robot1(:,:)',time_vec,'previous','extrap')';

% [time_unique,IA,~] = unique(time_vec_wrench_robot2);
wrench_robots(7:12,:) = interp1(time_vec_wrench_robot2,wrench_robot2(:,:)',time_vec,'previous','extrap')';

measurements = zeros(7*n_pose_measures*2,numSteps);
measure_occlusion = ones(2*n_pose_measures, numSteps+1);

for(i=0:n_pose_measures-1)
    measurements(i*7+4,:) = 1;
    measurements((i+n_pose_measures)*7+4,:) = 1;
end

% put aruco measures in a struct
time_vec_aruco_struct{1} = time_vec_side1_T_object_robot1;
time_vec_aruco_struct{2} = time_vec_side2_T_object_robot1;
time_vec_aruco_struct{3} = time_vec_side3_T_object_robot1;
time_vec_aruco_struct{4} = time_vec_side4_T_object_robot1;
time_vec_aruco_struct{5} = time_vec_side5_T_object_robot1;
time_vec_aruco_struct{6} = time_vec_side6_T_object_robot1;
time_vec_aruco_struct{7} = time_vec_side1_T_object_robot2;
time_vec_aruco_struct{8} = time_vec_side2_T_object_robot2;
time_vec_aruco_struct{9} = time_vec_side3_T_object_robot2;
time_vec_aruco_struct{10} = time_vec_side4_T_object_robot2;
time_vec_aruco_struct{11} = time_vec_side5_T_object_robot2;
time_vec_aruco_struct{12} = time_vec_side6_T_object_robot2;

aruco_struct{1} = side1_T_object_robot1;
aruco_struct{2} = side2_T_object_robot1;
aruco_struct{3} = side3_T_object_robot1;
aruco_struct{4} = side4_T_object_robot1;
aruco_struct{5} = side5_T_object_robot1;
aruco_struct{6} = side6_T_object_robot1;
aruco_struct{7} = side1_T_object_robot2;
aruco_struct{8} = side2_T_object_robot2;
aruco_struct{9} = side3_T_object_robot2;
aruco_struct{10} = side4_T_object_robot2;
aruco_struct{11} = side5_T_object_robot2;
aruco_struct{12} = side6_T_object_robot2;


%
for measure_index = 1:n_pose_measures*2
    for j=1:numSteps
        t = time_vec(j);
        index = find(time_vec_aruco_struct{measure_index}<t & time_vec_aruco_struct{measure_index}>(t-SampleTime));
        
        if(isempty(index))
           continue
        end
        if(numel(index)~=1)
            error("numel ~= 1")
        end
    
        measure_occlusion(measure_index,j+1) = 0;
        measurements((measure_index-1)*7+1:(measure_index-1)*7+7,j) = aruco_struct{measure_index}(:,index); 
         
    end
end


ekf_pose_aligned = interp1(time_vec_ekf_obj_pose,ekf_pose',time_vec,'previous','extrap')';

ekf_twist_aligned = interp1(time_vec_ekf_obj_twist,ekf_twist',time_vec,'previous','extrap')';

ekf_b1Tb2_pose = interp1(time_vec_ekf_b1Tb2,b1Tb2_pose',time_vec,'previous','extrap')';

time_vec = time_vec - time_vec(1);













