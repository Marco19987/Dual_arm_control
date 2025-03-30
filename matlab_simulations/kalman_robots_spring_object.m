clear variables
close all
clc

%% define parameters of the robots-object model
M = 0.280*eye(3);        % Mass matrix
I = 0.1*eye(3);      % Inertia matrix
Bm = blkdiag(M,I);
viscous_friction = 0.001*blkdiag(eye(3)*0.1, eye(3)*0.1); % viscous friction object-air

% stiffness and damping robot 1 - espressed in the grasp 1 frame
Klin_1 = 1000*eye(3);
Ktau_1 = 500*eye(3);
K_1 = blkdiag(Klin_1,Ktau_1);

Blin_1 = 50*eye(3);
Btau_1 = 0.5*eye(3);
B_1 = blkdiag(Blin_1,Btau_1); 

% stiffness and damping robot 2 - expressed in the grasp 2 frame
Klin_2 = 1000*eye(3);
Ktau_2 = 500*eye(3);
K_2 = blkdiag(Klin_2,Ktau_2);

Blin_2 = 50*eye(3);
Btau_2 = 0.5*eye(3);
B_2 = blkdiag(Blin_2,Btau_2);


n_pose_measures = 2;
bg = [0;0;-9.8*0];

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
                 b2Tb1(1:3,4)' rotm2quat(b2Tb1(1:3,1:3))]';

sizeState = 27;
sizeOutput = 2 * n_pose_measures * 7 + 12 + 14;
SampleTime = 0.001;
system = RobotsSpringObjectSystem(initialState(1:27),27, sizeOutput,SampleTime, Bm,bg,oTg1,oTg2,n_pose_measures ...
                                            ,b2Tb1,bTb1,viscous_friction,K_1,B_1,K_2,B_2);
% system = RobotsObjectSystemExt(initialState,base_system);

W_k = eye(sizeState) * 1e-5; % Updated covariance noise matrix for state transition
% W_k(14:20,14:20) = 1*diag([ones(1,3)*1e-7 ones(1,4)*1e-10]); % calibration b2Tb1 state


V_k_1_diag = [ones(1,3)*1e-4 ones(1,4)*1e-6];
V_k_1_diag_npose = repmat(V_k_1_diag,1,n_pose_measures);
V_k_2_diag = [ones(1,3)*1e-4 ones(1,4)*1e-6];
V_k_2_diag_npose = repmat(V_k_2_diag,1,n_pose_measures);
V_k = diag([V_k_1_diag_npose V_k_2_diag_npose]);

V_k_force = eye(12)*1e-2;
V_k_fkine = eye(14)*1e-5;



b2Tb1_perturbed = b2Tb1;
b2Tb1_perturbed(1:3,4) = b2Tb1(1:3,4)*1;
b2Tb1_perturbed(1:3,1:3) = b2Tb1(1:3,1:3)*eul2rotm([deg2rad(0*[1 1 1])]);

initialState_perturbed = [initialState(1:3);rotm2quat(Helper.my_quat2rotm(initialState(4:7)')*rotz(0*pi/2))'; 
    initialState(8:13); b1Te1(1:3,4);rotm2quat(b1Te1(1:3,1:3))';b2Te2(1:3,4);rotm2quat(b2Te2(1:3,1:3))';
    b2Tb1_perturbed(1:3,4); rotm2quat(b2Tb1_perturbed(1:3,1:3))'];


system_perturbed = RobotsSpringObjectSystem(initialState(1:27),27, sizeOutput,SampleTime, Bm,bg,oTg1,oTg2,n_pose_measures ...
                                            ,b2Tb1,bTb1,viscous_friction,K_1,B_1,K_2,B_2);
% system_forces_measured = RobotsObjectSystemExtForces(initialState_perturbed,system_perturbed); % to be used by the EKF

kf = KalmanFilter(system_perturbed, W_k, blkdiag(V_k,V_k_force,V_k_fkine));

kf.system.updateState(initialState_perturbed(1:27));
% kf.system.base_system.update_b2Tb1(b2Tb1_perturbed);


% Simulation parameters
tf = 1;
time_vec = 0:SampleTime:tf-SampleTime;
numSteps = length(time_vec);

u_k_fixed = 1*[1 0.0 0.0 0 0 0.0 0 -0.0 0 0 0 0.0]'; % twist of the robots

% Storage for results
trueStates = zeros(27, numSteps);
measurements = zeros(sizeOutput, numSteps);
filteredStates = zeros(sizeState, numSteps);
filteredMeasurements = zeros(sizeOutput, numSteps);

filteredState = initialState;

measure_occlusion = zeros(2*n_pose_measures, numSteps+1); % vector simulating the occlusion of arucos, 0 = occluded, 1 = visible
last_pose_vector = zeros(sizeOutput,1); % vector to store the last measured pose of the i-th aruco

% measure_occlusion = round(rand(2*n_pose_measures, numSteps+1))*1 + 0*1;
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

    
    % u_k = 0*u_k_fixed*(k<numSteps/4) + u_k_fixed*(k>=numSteps/4)*(k<numSteps/2) - u_k_fixed*(k>=numSteps/2);
    u_k = u_k_fixed*(k<numSteps/2);
    % Simulate the true system
    prevState = system.getState();
    trueState = system.state_fcn(prevState, u_k);
    measurement = system.output_fcn(trueState, u_k);
    % measurement = measurement + 0.005*repmat([randn(3, 1)' 1*randn(4, 1)']',2*n_pose_measures,1); % Add measurement noise
    measurement = measurement + randn(length(measurement),1)*0.01;

    % simulate occlusion of estimators
    for i=1:2*n_pose_measures
        measure_was_occluded = measure_occlusion(i,k);
        measure_occluded = measure_occlusion(i,k+1);
        if(measure_occluded && not(measure_was_occluded))
            % transition not occluded -> occluded
            last_pose_vector(1+(i-1)*7:(i-1)*7+7) = measurements((1+(i-1)*7:(i-1)*7+7),k-1)';
        end 
        if not(measure_occluded)
            % the marker is not occluded so the last pose is the measurement
            last_pose_vector(1+(i-1)*7:(i-1)*7+7) = measurement((1+(i-1)*7:(i-1)*7+7))';
        end 
        % if the marker was occluded and is still occluded do nothing

    end 
    % measurement = last_pose_vector;


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
    [filtered_measurement,filteredState] = kf.kf_apply(u_k_noised,measurement, W_k,  blkdiag(V_k,V_k_force,V_k_fkine));   
    filteredState(4:7) = filteredState(4:7)/(norm(filteredState(4:7)));

    % estimated_b2Tb1(1:4,1:4,k) = Helper.transformation_matrix(filteredState(14:16), filteredState(17:20));
    filteredState(17:20) = filteredState(17:20)/(norm(filteredState(17:20)));
    filteredState(24:27) = filteredState(24:27)/(norm(filteredState(24:27)));

    
    % update system state
    system.updateState(trueState);
    kf.system.updateState(filteredState);
    % kf.system.base_system.update_b2Tb1(estimated_b2Tb1(1:4,1:4,k)); % update estimated b1Tb2 in the base system 
    % 
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
    pause(SampleTime)
end 


