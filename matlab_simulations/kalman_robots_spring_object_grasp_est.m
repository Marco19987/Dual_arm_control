clear variables
close all
clc

%% define parameters of the robots-object model
M = 1*eye(3);        % Mass matrix
I = 0.1*eye(3);      % Inertia matrix
Bm = blkdiag(M,I);
viscous_friction = 0.001*blkdiag(eye(3)*0.1, eye(3)*0.1); % viscous friction object-air

% stiffness and damping robot 1 - espressed in the grasp 1 frame
Klin_1 = 1000*eye(3);
Ktau_1 = 1*eye(3);
K_1 = 1*blkdiag(Klin_1,Ktau_1);

Blin_1 = 10*eye(3);
Btau_1 = 0.1*eye(3);
B_1 = 1*blkdiag(Blin_1,Btau_1); 

% stiffness and damping robot 2 - expressed in the grasp 2 frame
Klin_2 = 1000*eye(3);
Ktau_2 = 1*eye(3);
K_2 = 1*blkdiag(Klin_2,Ktau_2);

Blin_2 = 10*eye(3);
Btau_2 = 0.1*eye(3);
B_2 = 1*blkdiag(Blin_2,Btau_2);


n_pose_measures = 6;
bg = [0;0;-9.8*1];

opg1 = [0;0.1;0];
opg2 = [0;-0.1;0];
oRg1 = eye(3);
oRg2 = eye(3);
oTg1 = Helper.transformation_matrix(opg1,rotm2quat(oRg1));
oTg2 = Helper.transformation_matrix(opg2,rotm2quat(oRg2));

% bTo = Helper.transformation_matrix([0.8,0.01,0.15]',[0 0.92 0.38 0]');
bTo = Helper.transformation_matrix([0.8,0.01,0.15]',[1 0 0 0]');


bTb1 = eye(4);
b1Tb2 = [quat2rotm([ 0.7071,0,0,0.7071]) [1.63;-0.00; 0.0]; 0 0 0 1];

b2Tb1 = inv(b1Tb2);

% compute initial pose robots given grasp frames and object pose
b1Te1 = inv(bTb1) * bTo * oTg1;
b2Te2 = b2Tb1 * inv(bTb1) * bTo * oTg2;



% Example usage of the KalmanFilter class
initialState = [bTo(1:3,4)' rotm2quat(bTo(1:3,1:3)) 0 0 0 0 0 0 ...
                b1Te1(1:3,4)' rotm2quat(b1Te1(1:3,1:3)) b2Te2(1:3,4)' rotm2quat(b2Te2(1:3,1:3)) ...
                 b2Tb1(1:3,4)' rotm2quat(b2Tb1(1:3,1:3)) ...
                 oTg1(1:3,4)' rotm2quat(oTg1(1:3,1:3)) ...
                 oTg2(1:3,4)' rotm2quat(oTg2(1:3,1:3))]';
% 
% load("initial_state_grasp.mat")
% initialState = x0';

sizeState = 48;
sizeOutput = 2 * n_pose_measures * 7 + 12 + 14;
SampleTime = 0.002;
continuous_system = RobotsSpringObjectSystem(initialState(1:27),27, sizeOutput, Bm,bg,oTg1,oTg2,n_pose_measures ...
                                            ,b2Tb1,bTb1,viscous_friction,K_1,B_1,K_2,B_2);
continuous_system_ext = RobotsSpringObjectSystemExt(initialState(1:34),continuous_system);
continuous_system_ext_grasp = RobotsSpringObjectSystemExtGraspEst(initialState,continuous_system_ext);

system = EulerIntegrator(SampleTime, continuous_system_ext_grasp);

W_k = eye(sizeState) * 1e-7; % Updated covariance noise matrix for state transition
W_k(4:7,4:7) = eye(4) * 1e-12; %bQo
W_k(1:3,1:3) = eye(3) * 1e-12; %bpo
W_k(8:13,8:13) = eye(6) * 1e-5; % otwisto
W_k(28:34,28:34) = 1*diag([ones(1,3)*1e-8 ones(1,4)*1e-8]); %b2Tb1
W_k(35:41,35:41) = 1*diag([ones(1,3)*1e-8 ones(1,4)*1e-5]); %oTg1
W_k(42:48,42:48) = 1*diag([ones(1,3)*1e-8 ones(1,4)*1e-5]); %oTg2





V_k_1_diag = [ones(1,3)*1e-4 ones(1,4)*1e-6];
V_k_1_diag_npose = repmat(V_k_1_diag,1,n_pose_measures);
V_k_2_diag = [ones(1,3)*1e-4 ones(1,4)*1e-6];
V_k_2_diag_npose = repmat(V_k_2_diag,1,n_pose_measures);
V_k = diag([V_k_1_diag_npose V_k_2_diag_npose]);
% V_k = 100*V_k;

V_k_force = eye(12)*1e-2;
% V_k_force = eye(12)*1e10;

V_k_fkine = eye(14)*1e-5;

V_k_force_default = V_k_force;
V_k_fkine_default = V_k_fkine;



b2Tb1_perturbed = b2Tb1;
b2Tb1_perturbed(1:3,4) = b2Tb1(1:3,4)*1;
b2Tb1_perturbed(1:3,1:3) = b2Tb1(1:3,1:3)*eul2rotm([deg2rad(0*[1 1 1])]);

% initialState_perturbed = [initialState(1:3);rotm2quat(Helper.my_quat2rotm(initialState(4:7)')*rotz(0*pi/2))'; 
%     initialState(8:13); b1Te1(1:3,4);rotm2quat(b1Te1(1:3,1:3))';b2Te2(1:3,4);rotm2quat(b2Te2(1:3,1:3))';
%     b2Tb1_perturbed(1:3,4); rotm2quat(b2Tb1_perturbed(1:3,1:3))'];
initialState_perturbed = initialState;

b1Te1_pert = Helper.transformation_matrix(initialState_perturbed(14:16),initialState_perturbed(17:20));
b2Te2_pert = Helper.transformation_matrix(initialState_perturbed(21:23),initialState_perturbed(24:27));
b2Tb1_perturbed = Helper.transformation_matrix(initialState_perturbed(28:30),initialState_perturbed(31:34));
bTo_pert = Helper.transformation_matrix(initialState_perturbed(1:3),initialState_perturbed(4:7));
oTg1_pert = inv(bTo_pert) * bTb1 * b1Te1_pert; 
oTg2_pert = inv(bTo_pert) * bTb1 * inv(b2Tb1_perturbed) *  b2Te2_pert;
oTg1_pert(1:3,4) = oTg1_pert(1:3,4)*0.9;
oTg2_pert(1:3,4) = oTg2_pert(1:3,4)*0.9;
oTg1_pert(1:3,1:3) = oTg1_pert(1:3,1:3)*rotx(5);
oTg2_pert(1:3,1:3) = oTg2_pert(1:3,1:3)*roty(5);

initialState_perturbed(35:48) = [oTg1_pert(1:3,4)' rotm2quat(oTg1_pert(1:3,1:3)) ...
                 oTg2_pert(1:3,4)' rotm2quat(oTg2_pert(1:3,1:3))]';


continuous_system_perturbed = RobotsSpringObjectSystem(initialState_perturbed(1:27),27, sizeOutput, Bm,bg,oTg1_pert,oTg2_pert,n_pose_measures ...
                                            ,b2Tb1_perturbed,bTb1,viscous_friction,K_1,B_1,K_2,B_2);
continuous_system_perturbed_ext = RobotsSpringObjectSystemExt(initialState_perturbed(1:34),continuous_system_perturbed);
continuous_system_perturbed_ext_grasp = RobotsSpringObjectSystemExtGraspEst(initialState_perturbed,continuous_system_perturbed_ext);


system_perturbed = EulerIntegrator(SampleTime, continuous_system_perturbed_ext_grasp);

kf = KalmanFilter(system_perturbed, W_k, blkdiag(V_k,V_k_force,V_k_fkine));

kf.system.updateState(initialState_perturbed);
kf.system.continuous_system.update_b2Tb1(b2Tb1_perturbed);


% Simulation parameters
tf = 1;
time_vec = 0:SampleTime:tf-SampleTime;
numSteps = length(time_vec);

u_k_fixed = 0.0*[1 0.0 0.0 0 0 0 0 -0.0 0 0 0 0.0]'; % twist of the robots

% Storage for results
trueStates = zeros(sizeState, numSteps);
measurements = zeros(sizeOutput, numSteps);
filteredStates = zeros(sizeState, numSteps);
filteredMeasurements = zeros(sizeOutput, numSteps);

filteredState = initialState;

% occlusion and multi rate simulation

aruco_measure_rate = 30;
measure_occlusion = ones(2*n_pose_measures, numSteps+1); % vector simulating the occlusion of arucos, 0 = visible, 1 = occluded
% last_pose_vector = zeros(2*7*n_pose_measures,1); % vector to store the last measured pose of the i-th aruco

measure_occlusion(:,1) = 1; % initially the markers are occluded
dt_aruco_measure_samples = round(1/(SampleTime*aruco_measure_rate));
measure_occlusion(1:end,2:dt_aruco_measure_samples:end) = 0;
measure_occlusion(1:end,2:dt_aruco_measure_samples:end) = randi([0 1],[n_pose_measures*2 size([2:dt_aruco_measure_samples:size(measure_occlusion,2)])]);
% measure_occlusion(:) = 0;

force_measure_rate = 100;
force_occlusion = ones(2, numSteps+1); % vector simulating the occlusion of force measures, 0 = visible, 1 = occluded
last_force_measure = zeros(12,1);
force_occlusion(:,1) = 1;
dt_force_measure_samples = round(1/(SampleTime*force_measure_rate));
force_occlusion(1:2,2:dt_force_measure_samples:end) = 0;


fkine_measure_rate = [200 50]'; % rate [Hz] measure fkine from robots
fkine_occlusion = ones(2, numSteps+1); % vector simulating the occlusion of fkine measures, 0 = visible, 1 = occluded
last_fkine_vecor = zeros(14,1);
fkine_occlusion(:,1) = 1;
dt_fkine1_measure_samples = round(1/(SampleTime*fkine_measure_rate(1)));
fkine_occlusion(1,2:dt_fkine1_measure_samples:end) = 0;
dt_fkine2_measure_samples = round(1/(SampleTime*fkine_measure_rate(2)));
fkine_occlusion(2,2:dt_fkine2_measure_samples:end) = 0;








% covariance update rule parameters
alpha_occlusion = 1.5;      % multiplicative factor for che covariance matrix in case of occlusion
saturation_occlusion = 15;
factor_occlusion = ones(2*n_pose_measures,1); % element used to saturate the multiplication factor 
V_ki_default = V_k(1:7,1:7);

V_k = eye(size(V_k))*1e10;
factor_occlusion = factor_occlusion * round(saturation_occlusion/alpha_occlusion);

estimated_b2Tb1 = zeros(4,4,numSteps);
for k = 1:numSteps
    disp(k*SampleTime)

    
    u_k = 0*u_k_fixed*(k<numSteps/4) + u_k_fixed*(k>=numSteps/4)*(k<numSteps/2) - u_k_fixed*(k>=numSteps/2);
    % u_k = 1*[0 0.0 0.0 0 0 0 0 0.0 0 pi/6 0 0.0]'; % twist of the robots

    % u_k = u_k_fixed*sin(k*SampleTime/(2*pi));
    % Simulate the true system
    prevState = system.getState();
    trueState = system.state_fcn(prevState, u_k);
    measurement = system.output_fcn(trueState, u_k);
    
    % add noise aruco
    % measurement(1:2*7*n_pose_measures) = measurement(1:2*7*n_pose_measures) + 0.0005*repmat([randn(3, 1)' 1*randn(4, 1)']',2*n_pose_measures,1); % Add measurement noise

    % add noise force measure
    % measurement(2*7*n_pose_measures+1:2*7*n_pose_measures+12) = measurement(2*7*n_pose_measures+1:2*7*n_pose_measures+12)+randn(12,1)*0.1;
    % measurement(2*7*n_pose_measures+1:2*7*n_pose_measures+12) = measurement(2*7*n_pose_measures+1:2*7*n_pose_measures+12) + 0.1*ones(12,1);
    
    % add noise fkine measure
    % measurement(2*7*n_pose_measures+13:2*7*n_pose_measures+13+13) = measurement(2*7*n_pose_measures+13:2*7*n_pose_measures+13+13)+randn(14,1)*0.001;




    for i=1:2*n_pose_measures
        measure_occluded = measure_occlusion(i,k);
        if(measure_occluded)
            measurement(1+(i-1)*7:(i-1)*7+7) = 0;
            V_k(1+(i-1)*7:(i-1)*7+7,1+(i-1)*7:(i-1)*7+7) = 1e10*eye(7);
        end 
        if not(measure_occluded)
            % reset the covariance matrix
            V_k(1+(i-1)*7:(i-1)*7+7,1+(i-1)*7:(i-1)*7+7) = V_ki_default;
        end 
    end 
    
    for i=1:2
        measure_occluded = force_occlusion(i,k);      
        if(measure_occluded)
           measurement(14*n_pose_measures + (i-1)*6+1:14*n_pose_measures +(i-1)*6+6) = 0;
           V_k_force((i-1)*6+1:(i-1)*6+6,(i-1)*6+1:(i-1)*6+6) = 1e10*eye(6);
        end 
        if not(measure_occluded)
            % reset the covariance matrix
            V_k_force((i-1)*6+1:(i-1)*6+6,(i-1)*6+1:(i-1)*6+6) = V_k_force_default((i-1)*6+1:(i-1)*6+6,(i-1)*6+1:(i-1)*6+6);
        end 
    end 

    for i=1:2
        measure_occluded = fkine_occlusion(i,k);
        if(measure_occluded)
           measurement(14*n_pose_measures + +12 + (i-1)*7+1:14*n_pose_measures + 12 +(i-1)*7+7) = 0;
           V_k_fkine((i-1)*7+1:(i-1)*7+7,(i-1)*7+1:(i-1)*7+7) = 1e10*eye(7);
        end 
        if not(measure_occluded)
            % reset the covariance matrix
            V_k_fkine((i-1)*7+1:(i-1)*7+7,(i-1)*7+1:(i-1)*7+7) =  V_k_fkine_default((i-1)*7+1:(i-1)*7+7,(i-1)*7+1:(i-1)*7+7);
        end 
    end 
    
    % Apply the Kalman filter
    [filtered_measurement,filteredState] = kf.kf_apply(u_k,measurement, W_k,  blkdiag(V_k,V_k_force,V_k_fkine));   
    filteredState(4:7) = filteredState(4:7)/(norm(filteredState(4:7)));

    estimated_b2Tb1(1:4,1:4,k) = Helper.transformation_matrix(filteredState(28:30), filteredState(31:34));
    filteredState(17:20) = filteredState(17:20)/(norm(filteredState(17:20)));
    filteredState(24:27) = filteredState(24:27)/(norm(filteredState(24:27)));
    filteredState(31:34) = filteredState(31:34)/(norm(filteredState(31:34)));
    filteredState(38:41) = filteredState(38:41)/(norm(filteredState(38:41)));
    filteredState(45:48) = filteredState(45:48)/(norm(filteredState(45:48)));


    
    % update system state
    system.updateState(trueState);
    kf.system.updateState(filteredState);
    kf.system.continuous_system.update_b2Tb1(estimated_b2Tb1(1:4,1:4,k)); % update estimated b1Tb2 in the base system 
    

    % Store results
    trueStates(:, k) = trueState;
    measurements(:, k) = measurement;
    filteredMeasurements(:,k) = filtered_measurement;
    filteredStates(:, k) = filteredState;
    % true_wrenches(:,k) = u_k;
end

%% Plot results
time_vec = 0:SampleTime:tf-SampleTime;

figure;
line_width = 1.5;
subplot(3, 1, 1);
plot(time_vec, trueStates(1:3, :), 'r',time_vec, filteredStates(1:3, :), 'b',"LineWidth", line_width);
legend('True State', 'Filtered State');
title('Position');
grid on


subplot(3, 1, 2);
plot(time_vec, trueStates(8:10,:), 'r',time_vec, filteredStates(8:10, :), 'b',"LineWidth", line_width);
legend('True State', 'Filtered State');
title('Linear Velocity');
grid on

subplot(3, 1, 3);
plot(time_vec, trueStates(11:13, :), 'r',time_vec, filteredStates(11:13, :), 'b',"LineWidth", line_width);
legend('True State', 'Filtered State');
title('Angular Velocity');
grid on

figure
plot(time_vec, trueStates(4:7, :), 'r',time_vec, filteredStates(4:7, :), 'b',"LineWidth", line_width);
legend('True State', 'Filtered State');
title('Quaternion Components');
grid on

figure 
subplot(3, 1, 1);
plot(time_vec, measurements([1:3], :),'r', time_vec, filteredMeasurements([1 2 3], :),'b',"LineWidth", line_width);
legend('Measurements', 'Filtered Measurements');
title('Object position base1 frane');
grid on

subplot(3, 1, 2);
index_aruco_measure_2 = n_pose_measures*7 + 1;
plot(time_vec, measurements([index_aruco_measure_2:index_aruco_measure_2+2], :),'r', time_vec, filteredMeasurements([index_aruco_measure_2:index_aruco_measure_2+2], :),'b',"LineWidth", line_width);
legend('Measurements', 'Filtered Measurements');
title('Object position base2 frane');
grid on


%% robot calibration results
figure 
subplot(2,1,1)
plot(time_vec, repmat(b2Tb1(1:3,4),1,numSteps), 'r', time_vec, squeeze(estimated_b2Tb1(1:3,4,:)), 'b', "LineWidth",line_width);
legend('x','y','z','x_hat','y_hat','z_hat');
grid on

quaternion_real = rotm2quat(b2Tb1(1:3,1:3))';
quaternion_estimated_b2Tb1 = zeros(4,numSteps);
for i=1:numSteps
   quaternion_estimated_b2Tb1(1:4,i) = rotm2quat(estimated_b2Tb1(1:3,1:3,i));
end

subplot(2,1,2)
plot(time_vec, repmat(quaternion_real,1,numSteps), 'r', time_vec, quaternion_estimated_b2Tb1(1:4,:), 'b', "LineWidth",line_width);
legend('real quaternion between robots','estimated');
grid on

%% Frames animation
figure

bTobj = Helper.transformation_matrix(trueStates(1:3,1), trueStates(4:7,1));
bT1 =  bTb1*Helper.transformation_matrix(trueStates(14:16,1), trueStates(17:20,1));
bT2 =  bTb1*inv(b2Tb1)*Helper.transformation_matrix(trueStates(21:23,1), trueStates(24:27,1));

frame_obj = Frame(bTobj,"frame",'obj', 'color', 'b','text_opts', {'FontSize', 10, 'FontWeight', 'bold'});
hold on
frame_robot1 = Frame(bT1,"frame",'robot1', 'color', 'b','text_opts', {'FontSize', 10, 'FontWeight', 'bold'});
hold on
frame_robot2 = Frame(bT2,"frame",'robot2', 'color', 'b','text_opts', {'FontSize', 10, 'FontWeight', 'bold'});

for i=1:numSteps
    bTobj = Helper.transformation_matrix(trueStates(1:3,i), trueStates(4:7,i));
    bT1 =  bTb1*Helper.transformation_matrix(trueStates(14:16,i), trueStates(17:20,i));
    bT2 =  bTb1*inv(b2Tb1)*Helper.transformation_matrix(trueStates(21:23,i), trueStates(24:27,i));

    frame_obj.move(bTobj);
    frame_robot1.move(bT1);
    frame_robot2.move(bT2);
    pause(SampleTime*10)
end 


