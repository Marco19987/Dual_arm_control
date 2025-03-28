clear variables
close all
clc

%% define parameters of the robots-object model
M = 0.280*eye(3);        % Mass matrix
I = 0.01*eye(3);      % Inertia matrix
Bm = blkdiag(M,I);
viscous_friction = 0.0001*blkdiag(eye(3)*0.1, eye(3)*0.01); % viscous friction object-air
n_pose_measures = 2;
bg = [0;0;-9.8*0];
opg1 = [0;0.1;0];
opg2 = [0;-0.1;0];
oRg1 = eye(3);
oRg2 = eye(3);
oTg1 = Helper.transformation_matrix(opg1,rotm2quat(oRg1));
oTg2 = Helper.transformation_matrix(opg2,rotm2quat(oRg2));

bTo = Helper.transformation_matrix([0 0 0]',[0 0 1 0]);%rotm2quat(eul2rotm([deg2rad([1 50 31])])));

bpb1 = [0.8;0.0;0.31];
bQb1 = rotm2quat([0 1 0; -1 0 0; 0 0 1])';
bTb1 = Helper.transformation_matrix(bpb1,bQb1); % well known
bTb2 = [eye(3) [-0.8;0.0; 0.34]; 0 0 0 1];

b1Tb2 = bTb1 \ bTb2;    % roughly known
b2Tb1 = inv(b1Tb2);

b1Tg1 = bTb1 \ bTo * oTg1; 
b2Tg2 = bTb2 \ bTo * oTg2; 


% Example usage of the KalmanFilter class
initialState = [bTo(1:3,4)' rotm2quat(bTo(1:3,1:3)) 0 0 0 0 0 0]';
sizeState = 13;
sizeOutput = 2 * n_pose_measures * 7;
SampleTime = 0.05;
system = RobotsObjectSystem(initialState, sizeState, sizeOutput,SampleTime, Bm,bg,oTg1,oTg2,n_pose_measures ...
                                            ,b2Tb1,bTb1,viscous_friction);

W_k = eye(sizeState) * 1e-6; % Updated covariance noise matrix for state transition
% V_k = eye(sizeOutput) * 0.000001; % Updated covariance noise matrix for output

V_k_1_diag = [ones(1,3)*1e-4 ones(1,4)*1e-6];
V_k_1_diag_npose = repmat(V_k_1_diag,1,n_pose_measures);
V_k_2_diag = [ones(1,3)*1e-4 ones(1,4)*1e-6];
V_k_2_diag_npose = repmat(V_k_2_diag,1,n_pose_measures);
V_k = diag([V_k_1_diag_npose V_k_2_diag_npose]);


kf = KalmanFilter(system, W_k, V_k); 

initialState_perturbed = [initialState(1:3)*1;rotm2quat(Helper.my_quat2rotm(initialState(4:7)')*rotz(0*pi/2))'; initialState(8:end)];
kf.system.updateState(initialState_perturbed);

b2Tb1_perturbed = b2Tb1;
b2Tb1_perturbed(1:3,4) = b2Tb1(1:3,4)*1;
b2Tb1_perturbed(1:3,1:3) = b2Tb1(1:3,1:3)*eul2rotm([deg2rad(0*[1 1 1])]);
kf.system.update_b2Tb1(b2Tb1_perturbed);

% Simulation parameters
tf = 10;
time_vec = 0:SampleTime:tf-SampleTime;
numSteps = length(time_vec);

u_k_fixed = 10*[0.0 0.11 0 0.0 0 0 0.0 -0.1 0 0 0 0]'; % Wrench applied by the robots in the grasp frames

% Storage for results
trueStates = zeros(sizeState, numSteps);
measurements = zeros(sizeOutput, numSteps);
filteredStates = zeros(sizeState, numSteps);
filteredMeasurements = zeros(sizeOutput, numSteps);

filteredState = initialState;

measure_occlusion = zeros(2*n_pose_measures, numSteps+1); % vector simulating the occlusion of arucos, 0 = occluded, 1 = visible
last_pose_vector = zeros(sizeOutput,1); % vector to store the last measured pose of the i-th aruco

% measure_occlusion = round(rand(2*n_pose_measures, numSteps+1));
% measure_occlusion(1,round(numSteps/2):end) = 1;
% measure_occlusion(2,round(numSteps/2):end) = 1;
% measure_occlusion(3,round(numSteps/2):end) = 1;
% measure_occlusion(4,round(numSteps/2):end) = 1;


% covariance update rule parameters
alpha_occlusion = 1.5;      % multiplicative factor for che covariance matrix in case of occlusion
saturation_occlusion = 15;
factor_occlusion = ones(2*n_pose_measures,1); % element used to saturate the multiplication factor 
V_ki_default = V_k(1:7,1:7);%1*eye(7,7);


for k = 1:numSteps
    disp(k)

    u_k = u_k_fixed*(k<numSteps/2) - u_k_fixed*(k>=numSteps/2);

    % Simulate the true system
    prevState = system.getState();
    trueState = system.state_fcn(prevState, u_k);
    measurement = system.output_fcn(trueState, u_k);
    measurement = measurement + 0.005*repmat([randn(3, 1)' 0.0001*randn(4, 1)']',2*n_pose_measures,1); % Add measurement noise



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
    
     % Apply the Kalman filter
    % u_k = u_k + randn(12,1)*0.01;
    % u_k = u_k + [0.1 0.1 0.1 0.01 0.01 0.01 0.1 0.1 0.1 0.01 0.01 0.01]';

    [filtered_measurement,filteredState] = kf.kf_apply(u_k, measurement, W_k, V_k);   
    filteredState(4:7) = filteredState(4:7)/(norm(filteredState(4:7)));
    
    % update system state
    system.updateState(trueState);
    kf.system.updateState(filteredState);
    
    % Store results
    trueStates(:, k) = trueState;
    measurements(:, k) = measurement;
    filteredMeasurements(:,k) = filtered_measurement;
    filteredStates(:, k) = filteredState;
end

%% Plot results
time_vec = 0:SampleTime:tf-SampleTime;

figure;
line_width = 1;
subplot(3, 1, 1);
plot(time_vec, trueStates(1:3, :), 'g',time_vec, filteredStates(1:3, :), 'b',"LineWidth", line_width);
legend('True State', 'Filtered State');
title('Position');
grid on


subplot(3, 1, 2);
plot(time_vec, trueStates(8:10,:), 'g',time_vec, filteredStates(8:10, :), 'b',"LineWidth", line_width);
legend('True State', 'Filtered State');
title('Linear Velocity');
grid on

subplot(3, 1, 3);
plot(time_vec, trueStates(11:13, :), 'g',time_vec, filteredStates(11:13, :), 'b',"LineWidth", line_width);
legend('True State', 'Filtered State');
title('Angular Velocity');
grid on

figure
plot(time_vec, trueStates(4:7, :), 'g',time_vec, filteredStates(4:7, :), 'b',"LineWidth", line_width);
legend('True State', 'Filtered State');
title('Quaternion Components');
grid on

figure 
subplot(3, 1, 1);
plot(time_vec, measurements([1:3], :),'g', time_vec, filteredMeasurements([1 2 3], :),'b',"LineWidth", line_width);
legend('Measurements', 'Filtered Measurements');
title('Object position base1 frane');
grid on

subplot(3, 1, 2);
plot(time_vec, measurements([15:17], :),'g', time_vec, filteredMeasurements([15:17], :),'b',"LineWidth", line_width);
legend('Measurements', 'Filtered Measurements');
title('Object position base2 frane');
grid on

