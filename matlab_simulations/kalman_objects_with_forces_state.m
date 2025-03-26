clear variables
close all
clc

%% define parameters of the robots-object model
M = 0.280*eye(3);        % Mass matrix
I = 0.1*eye(3);      % Inertia matrix
Bm = blkdiag(M,I);
viscous_friction = 1*blkdiag(eye(3)*0.1, eye(3)*0.1); % viscous friction object-air
n_pose_measures = 2;
bg = [0;0;-9.8*0];
opg1 = [0;0.1;0];
opg2 = [0;-0.1;0];
oRg1 = eye(3);
oRg2 = eye(3);
oTg1 = Helper.transformation_matrix(opg1,rotm2quat(oRg1));
oTg2 = Helper.transformation_matrix(opg2,rotm2quat(oRg2));

bTo = Helper.transformation_matrix([0.8,0.01,0.15]',[0 0.92 0.38 0]');
% bTo = Helper.transformation_matrix([1.2,0.1,0.0,0.0]',[0 0 0 1]);%rotm2quat(eul2rotm([deg2rad([1 50 31])])));

% bpb1 = [0.8;0.0;0.31];
% bQb1 = rotm2quat([0 1 0; -1 0 0; 0 0 1])';
% bTb1 = Helper.transformation_matrix(bpb1,bQb1); % well known
% bTb2 = [eye(3) [-0.8;0.0; 0.34]; 0 0 0 1];


bpb1 = [0;0.0;0];
bQb1 = rotm2quat(eye(3))';
bTb1 = Helper.transformation_matrix(bpb1,bQb1); % well known
bTb2 = [quat2rotm([0.72,0.69,-0.02,-0.02]) [1.63;-0.007; 0.04]; 0 0 0 1];

b1Tb2 = bTb1 \ bTb2;    % roughly known
b2Tb1 = inv(b1Tb2);



% Example usage of the KalmanFilter class
initialState = [bTo(1:3,4)' rotm2quat(bTo(1:3,1:3)) 0 0 0 0 0 0 b2Tb1(1:3,4)' rotm2quat(b2Tb1(1:3,1:3))]';

sizeState = 32;
sizeOutput = 2 * n_pose_measures * 7;
SampleTime = 0.05;
base_system = RobotsObjectSystem(initialState(1:13), 13, sizeOutput,SampleTime, Bm,bg,oTg1,oTg2,n_pose_measures ...
                                            ,b2Tb1,bTb1,viscous_friction);
system = RobotsObjectSystemExt(initialState,base_system);

W_k = eye(sizeState) * 1e-5; % Updated covariance noise matrix for state transition
W_k(14:20,14:20) = 1*diag([ones(1,3)*1e-7 ones(1,4)*1e-10]); % calibration b2Tb1 state
force_cov = eye(3)*1e-2;
torque_cov = eye(3)*1e-2;
W_k(21:32,21:32) = blkdiag(force_cov,torque_cov,force_cov,torque_cov); % force state

% V_k = eye(sizeOutput) * 0.01; % Updated covariance noise matrix for output
V_k_1_diag = [ones(1,3)*1e-4 ones(1,4)*1e-6];
V_k_1_diag_npose = repmat(V_k_1_diag,1,n_pose_measures);
V_k_2_diag = [ones(1,3)*1e-4 ones(1,4)*1e-6];
V_k_2_diag_npose = repmat(V_k_2_diag,1,n_pose_measures);
V_k = diag([V_k_1_diag_npose V_k_2_diag_npose]);

V_k_force = eye(12)*1e-2;
% W_k = 0.1*W_k;


b2Tb1_perturbed = b2Tb1;
b2Tb1_perturbed(1:3,4) = b2Tb1(1:3,4)*1;
b2Tb1_perturbed(1:3,1:3) = b2Tb1(1:3,1:3)*eul2rotm([deg2rad(0*[1 1 1])]);

initialState_perturbed = [initialState(1:3);rotm2quat(Helper.my_quat2rotm(initialState(4:7)')*rotz(0*pi/2))'; 
    initialState(8:13); b2Tb1_perturbed(1:3,4); rotm2quat(b2Tb1_perturbed(1:3,1:3))';zeros(12,1)];

system_forces_measured = RobotsObjectSystemExtForces(initialState_perturbed,system); % to be used by the EKF

kf = KalmanFilter(system_forces_measured, W_k, blkdiag(V_k,V_k_force));

kf.system.updateState(initialState_perturbed);
kf.system.base_system.update_b2Tb1(b2Tb1_perturbed);


% Simulation parameters
tf = 50;
time_vec = 0:SampleTime:tf-SampleTime;
numSteps = length(time_vec);

u_k_fixed = 2*[0.0 0.1 0.0 0 0 0 0 -0.01 0 0 0 0]'; % Wrench applied by the robots in the grasp frames

% Storage for results
trueStates = zeros(20, numSteps);
measurements = zeros(sizeOutput, numSteps);
filteredStates = zeros(sizeState, numSteps);
filteredMeasurements = zeros(sizeOutput+12, numSteps);

filteredState = initialState;

measure_occlusion = zeros(2*n_pose_measures, numSteps+1); % vector simulating the occlusion of arucos, 0 = occluded, 1 = visible
last_pose_vector = zeros(sizeOutput,1); % vector to store the last measured pose of the i-th aruco

measure_occlusion = round(rand(2*n_pose_measures, numSteps+1))*1 + 0*1;
% measure_occlusion(1,round(numSteps/2):end) = 1;
% measure_occlusion(2,round(numSteps/2):end) = 1;
% measure_occlusion(3,round(numSteps/2):end) = 1;
% measure_occlusion(4,round(numSteps/2):end) = 1;
% measure_occlusion(2,1:end) = 1;
% measure_occlusion(4,1:end) = 1;



% covariance update rule parameters
alpha_occlusion = 1.5;      % multiplicative factor for che covariance matrix in case of occlusion
saturation_occlusion = 15;
factor_occlusion = ones(2*n_pose_measures,1); % element used to saturate the multiplication factor 
V_ki_default = V_k(1:7,1:7);

estimated_b2Tb1 = zeros(4,4,numSteps);
for k = 1:numSteps
    disp(k*SampleTime)

    
    u_k = 0*u_k_fixed*(k<numSteps/4) + u_k_fixed*(k>=numSteps/4)*(k<numSteps/2) - u_k_fixed*(k>=numSteps/2);
    %u_k = 0.1 * ones(12,1) * (-1 + round(rand(1))*2);

    % Simulate the true system
    prevState = system.getState();
    trueState = system.state_fcn(prevState, u_k);
    measurement = system.output_fcn(trueState, u_k);
    % measurement = measurement + 0.005*repmat([randn(3, 1)' 1*randn(4, 1)']',2*n_pose_measures,1); % Add measurement noise

    % simulate occlusion of estimators
    for i=1:2*n_pose_measures
        measure_was_occluded = measure_occlusion(i,k);
        measure_occluded = measure_occlusion(i,k+1);
        if(measure_occluded && not(measure_was_occluded))
            % transition not occluded -> occluded
            last_pose_vector(1+(i-1)*7:(i-1)*7+7) = measurement((1+(i-1)*7:(i-1)*7+7))';
        end 
        if not(measure_occluded)
            % the marker is not occluded so the last pose is the measurement
            last_pose_vector(1+(i-1)*7:(i-1)*7+7) = measurement((1+(i-1)*7:(i-1)*7+7))';
        end 
        % if the marker was occluded and is still occluded do nothing

    end 
    measurement = last_pose_vector;


    % update V_k in correspondence of the occlusions detected
    for i=1:2*n_pose_measures
        measure_occluded = measure_occlusion(i,k+1);
        if(measure_occluded)
            % increase the covariance of this measure
            alpha_i = factor_occlusion(i)*alpha_occlusion;
            if saturation_occlusion > alpha_i
                factor_occlusion(i) = factor_occlusion(i) + 1;
                V_k(1+(i-1)*7:(i-1)*7+7,1+(i-1)*7:(i-1)*7+7) = alpha_i*V_k(1+(i-1)*7:(i-1)*7+7,1+(i-1)*7:(i-1)*7+7);
            end
        end 
        if not(measure_occluded)
            % reset the covariance matrix
            factor_occlusion(i) = 1;
            V_k(1+(i-1)*7:(i-1)*7+7,1+(i-1)*7:(i-1)*7+7) = V_ki_default;
        end 
    end 
    
    % add bias/noise on the measurements 
    % u_k_biased = u_k + 1*0.1*[0.1,0.2,0.1,0.01,0.02,0.01,0.01,0.1,0.3,0.01,0.01,0.001]';
    % u_k_noised = 0*u_k_biased + 1*1*[0.1*randn(3,1); 0.01*randn(3,1); 0.1*randn(3,1); 0.01*randn(3,1)];
    u_k_noised = u_k

     % Apply the Kalman filter
    [filtered_measurement,filteredState] = kf.kf_apply(zeros(12,1),[measurement;u_k_noised], W_k,  blkdiag(V_k,V_k_force));   
    filteredState(4:7) = filteredState(4:7)/(norm(filteredState(4:7)));

    estimated_b2Tb1(1:4,1:4,k) = Helper.transformation_matrix(filteredState(14:16), filteredState(17:20));


    % filteredState(17:20) = filteredState(17:20)/(norm(filteredState(17:20)));
    
    % update system state
    system.updateState(trueState);
    kf.system.updateState(filteredState);
    kf.system.base_system.update_b2Tb1(estimated_b2Tb1(1:4,1:4,k)); % update estimated b1Tb2 in the base system 
    
    % Store results
    trueStates(:, k) = trueState;
    measurements(:, k) = measurement;
    filteredMeasurements(:,k) = filtered_measurement;
    filteredStates(:, k) = filteredState;
    true_wrenches(:,k) = u_k;
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
plot(time_vec, measurements([end-6:end-4], :),'r', time_vec, filteredMeasurements([end-12-6:end-12-4], :),'b',"LineWidth", line_width);
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

%% 
figure
plot(time_vec, true_wrenches(1:3,:)')
hold on
plot(time_vec, filteredStates(21:23,:)')
