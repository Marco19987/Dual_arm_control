% clear variables
close all
clc

addpath('..');

% {\displaystyle I_{h}={\frac {1}{12}}m\left(w^{2}+d^{2}\right)}
% I w = 1 12 m ( h 2 + d 2 ) {\displaystyle I_{w}={\frac {1}{12}}m\left(h^{2}+d^{2}\right)}
% I d = 1 12 m ( h 2 + w 2 ) {\displaystyle I_{d}={\frac {1}{12}}m\left(h^{2}+w^{2}\right)}

%% define parameters of the robots-object model
M = 0.280*eye(3);        % Mass matrix
I = 0.01*eye(3);      % Inertia matrix
Bm = blkdiag(M,I);
viscous_friction = 0.1*blkdiag(eye(3)*0.1, eye(3)*0.01); % viscous friction object-air
n_pose_measures = 6;
bg = [0;0;-9.8*0];
opg1 = [0.06, -0.025, -0.0]';
opg2 = [-0.056, -0.005, 0.0]';
oRg1 = quat2rotm([0.0,-0.3827,0.9239, 0.0]);
oRg2 = quat2rotm([ 0.9239, 0.0, 0.0, -0.3827]);
oTg1 = Helper.transformation_matrix(opg1,rotm2quat(oRg1));
oTg2 = Helper.transformation_matrix(opg2,rotm2quat(oRg2));

bTo = Helper.transformation_matrix(ekf_pose_aligned(1:3,1),ekf_pose_aligned(4:7,1));


bTb1 = eye(4);
% b1Tb2 = [quat2rotm([ 0.7071,0,0,0.7071]) [1.63;-0.00; 0.0]; 0 0 0 1];
b1Tb2 = Helper.transformation_matrix(ekf_b1Tb2_pose(1:3,1),ekf_b1Tb2_pose(4:7,1)); % start from in the same conditions of the experiments
b2Tb1 = inv(b1Tb2);

initialState = [bTo(1:3,4)' rotm2quat(bTo(1:3,1:3)) 0 0 0 0 0 0 b2Tb1(1:3,4)' rotm2quat(b2Tb1(1:3,1:3))]';

sizeState = 32;
sizeOutput = 2 * n_pose_measures * 7;
base_system = RobotsObjectSystem(initialState(1:13), 13, sizeOutput,SampleTime, Bm,bg,oTg1,oTg2,n_pose_measures ...
                                            ,b2Tb1,bTb1,viscous_friction);
system = RobotsObjectSystemExt(initialState,base_system);

W_k = eye(sizeState) * 1e-5; % Updated covariance noise matrix for state transition
% W_k(14:20,14:20) = 1*diag([ones(1,3)*1e-11 ones(1,4)*1e-11]); % calibration b2Tb1 state
% force_cov = eye(3)*1e-3;
% torque_cov = eye(3)*1e-3;
% W_k(21:32,21:32) = blkdiag(force_cov,torque_cov,force_cov,torque_cov); % force state

%0.0000000001, 0.00000000001, 0.000000001, 0.000000001, 0.0000000001, 0.00000000000001,0.001,0.001
W_k(1:3,1:3) = eye(3)*1e-10;
W_k(4:7,4:7) = eye(4)*1e-11;
W_k(8:10,8:10) = eye(3)*1e-9;
W_k(11:13,11:13) = eye(3)*1e-9;
W_k(14:16,14:16) = eye(3)*1e-10;
W_k(17:20,17:20) = eye(4)*1e-14;
W_k(21:32,21:32) = eye(12)*1e-3;

%[0.0000001, 0.0000001, 0.0000001, 0.000000001, 0.0000000001, 0.000000001, 0.000000001,0.000001,0.000001]

% V_k = eye(sizeOutput) * 0.01; % Updated covariance noise matrix for output
% V_k_1_diag = [ones(1,3)*1e-8 ones(1,4)*1e-8];
V_k_1_diag = [0.0000001, 0.0000001, 0.0000001, 0.000000001, 0.0000000001, 0.000000001, 0.000000001];
V_k_1_diag_npose = repmat(V_k_1_diag,1,n_pose_measures);
% V_k_2_diag = [ones(1,3)*1e-8 ones(1,4)*1e-8];
V_k_2_diag = [0.0000001, 0.0000001, 0.0000001, 0.000000001, 0.0000000001, 0.000000001, 0.000000001];
V_k_2_diag_npose = repmat(V_k_2_diag,1,n_pose_measures);
V_k = diag([V_k_1_diag_npose V_k_2_diag_npose]);

V_k_force = eye(12)*1e-6;
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




% Storage for results
filteredStates = zeros(sizeState, numSteps);
filteredMeasurements = zeros(sizeOutput+12, numSteps);

filteredState = initialState;

% measure_occlusion = zeros(2*n_pose_measures, numSteps+1); % vector simulating the occlusion of arucos, 1 = occluded, 0 = visible
last_pose_vector = measurements(:,1); % vector to store the last measured pose of the i-th aruco


% covariance update rule parameters
alpha_occlusion = 1.15;      % multiplicative factor for che covariance matrix in case of occlusion
saturation_occlusion = 15;
factor_occlusion = ones(2*n_pose_measures,1); % element used to saturate the multiplication factor 
V_ki_default = V_k(1:7,1:7);

V_k = eye(size(V_k))*1e1;
factor_occlusion = factor_occlusion * round(saturation_occlusion/alpha_occlusion);

estimated_b2Tb1 = zeros(4,4,numSteps);
b2Tb1_convergence_status = false;
b2Tb1_convergence_index = 0;
for k = 1:numSteps
    disp(k*SampleTime)

    u_k = 0*wrench_robots(:,k);
    measurement = measurements(:,k);


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
    u_k_noised = u_k;

     % Apply the Kalman filter
    [filtered_measurement,filteredState] = kf.kf_apply(zeros(12,1),[measurement;u_k_noised], W_k,  blkdiag(V_k,V_k_force));   
    filteredState(4:7) = filteredState(4:7)/(norm(filteredState(4:7)));
    filteredState(17:20) = filteredState(17:20)/(norm(filteredState(17:20)));

    % ensure quaternion continuity with measures
    % for(i=0:n_pose_measures-1)
    %     measurements(i*7+4:i*7+7,k+1) = Helper.quaternion_continuity(measurements(i*7+4:i*7+7,k+1),filteredState(4:7));
    %     measurements((i+n_pose_measures)*7+4:(i+n_pose_measures)*7+7,k+1) = Helper.quaternion_continuity(measurements((i+n_pose_measures)*7+4:(i+n_pose_measures)*7+7,k+1),filteredState(4:7));
    % end

    
    estimated_b2Tb1(1:4,1:4,k) = Helper.transformation_matrix(filteredState(14:16), filteredState(17:20));
    estimated_b1Qb2(1:4,k) = Helper.quaternion_inverse(filteredState(17:20));

    % control b2Tb1 convergence
    if(~b2Tb1_convergence_status)
        index_measure_1 = find(measure_occlusion(1:n_pose_measures,k)==0,1)-1;
        index_measure_2 = find(measure_occlusion(n_pose_measures+1:end,k)==0,1)-1;
    
    
    
        if(~isempty(index_measure_1) & ~isempty(index_measure_2))
            b1To_filt = Helper.transformation_matrix(measurement(index_measure_1*7+1:index_measure_1*7+3), measurement(index_measure_1*7+4:index_measure_1*7+7));
            b2To_filt = Helper.transformation_matrix(measurement((n_pose_measures+index_measure_2)*7+1:(n_pose_measures+index_measure_2)*7 + 3), measurement((index_measure_2+n_pose_measures)*7+4:(index_measure_2+n_pose_measures)*7 + 7));
            b1To_b2 = inv(estimated_b2Tb1(1:4,1:4,k))*b2To_filt;
        
            error_calib = inv(b1To_filt)*b1To_b2;
        
            norm_error_pos(k) = norm(error_calib(1:3,4));
            axisangle_error = rotm2axang(error_calib(1:3,1:3));
            error_orientation(k) = axisangle_error(4);
            if(norm_error_pos(k)<0.07 && error_orientation(k)<0.01)
                W_k(14:20,14:20) = zeros(7);
                kf.setW(W_k);
                kf.setP(W_k);
                b2Tb1_convergence_status = true;
                b2Tb1_convergence_index = k;
            end 
        end 
    end
 


    
    % update system state
    kf.system.updateState(filteredState);
    kf.system.base_system.update_b2Tb1(estimated_b2Tb1(1:4,1:4,k)); % update estimated b1Tb2 in the base system 
    
    % Store results
    filteredMeasurements(:,k) = filtered_measurement;
    filteredStates(:, k) = filteredState;
end

%% Plot results
% 
% figure;
line_width = 1.5;
% subplot(3, 1, 1);
% plot(time_vec, filteredStates(1:3, :), 'b',"LineWidth", line_width);
% legend('Filtered State');
% title('Position');
% grid on
% 
% 
% subplot(3, 1, 2);
% plot(time_vec, filteredStates(8:10, :), 'b',"LineWidth", line_width);
% legend('Filtered State');
% title('Linear Velocity');
% grid on
% 
% subplot(3, 1, 3);
% plot(time_vec, filteredStates(11:13, :), 'b',"LineWidth", line_width);
% legend('True State', 'Filtered State');
% title('Angular Velocity');
% grid on
% 
% figure
% plot(time_vec, filteredStates(4:7, :), 'b',"LineWidth", line_width);
% legend('Filtered State');
% title('Quaternion Components');
% grid on
% 
% figure 
% subplot(3, 1, 1);
% plot(time_vec, measurements([1:3], :),'r', time_vec, filteredMeasurements([1 2 3], :),'b',"LineWidth", line_width);
% legend('Measurements', 'Filtered Measurements');
% title('Object position base1 frane');
% grid on
% 
% subplot(3, 1, 2);
% plot(time_vec, measurements([end-6:end-4], :),'r', time_vec, filteredMeasurements([end-12-6:end-12-4], :),'b',"LineWidth", line_width);
% legend('Measurements', 'Filtered Measurements');
% title('Object position base2 frane');
% grid on


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
hold on, grid on
plot(time_vec, filteredStates(21:23,:)');
legend('estimated forces');

%% plot pose measures
figure
hold on, grid on, title("aruco positions robot1")
for(i=0:n_pose_measures-1)
    plot(time_vec,measurements(i*7+1:i*7+3,:));
    
    % measurement((i+n_pose_measures)*7+4:(i+n_pose_measures)*7+7) = Helper.quaternion_continuity(measurement((i+n_pose_measures)*7+4:(i+n_pose_measures)*7+7),filteredState(4:7));
end

figure
hold on, grid on, title("aruco positions robot2")
for(i=0:n_pose_measures-1)
    plot(time_vec,measurements((i+n_pose_measures)*7+1:(i+n_pose_measures)*7+3,:));
    hold on
end

%% plot comparison ekf c and ekf matlab
figure;
subplot(2,2, 1);
plot(time_vec, ekf_pose_aligned(1:3, :)', 'r',"LineWidth", line_width);
hold on
plot(time_vec, filteredStates(1:3, :), 'b',"LineWidth", line_width);
legend({'ekf cpp','','','ekf matlab','',''});
title('Object Position');
grid on

subplot(2, 2, 2);
plot(time_vec, ekf_pose_aligned(4:7, :)', 'r',"LineWidth", line_width);
hold on
plot(time_vec, filteredStates(4:7, :), 'b',"LineWidth", line_width);
legend({'ekf cpp','','','','ekf matlab'});
title('Object Quaternion');
grid on

subplot(2,2, 3);
plot(time_vec, ekf_twist_aligned(1:3, :)', 'r',"LineWidth", line_width);
hold on
plot(time_vec, filteredStates(8:10, :), 'b',"LineWidth", line_width);
legend({'ekf cpp','','','ekf matlab'});
title('Object Linear Twist');
grid on

subplot(2,2, 4);
plot(time_vec, ekf_twist_aligned(4:6, :)', 'r',"LineWidth", line_width);
hold on
plot(time_vec, filteredStates(11:13, :), 'b',"LineWidth", line_width);
legend({'ekf cpp','','','ekf matlab'});
title('Object Angular Twist');
grid on

figure 
subplot(2,1,1)
plot(time_vec, ekf_b1Tb2_pose(1:3, :)', 'r',"LineWidth", line_width);
hold on
plot(time_vec, filteredStates(14:16, :), 'b',"LineWidth", line_width);
legend({'ekf cpp','','','ekf matlab'});
title('b1Tb2 Position Estimate');
grid on

subplot(2,1,2)
plot(time_vec, ekf_b1Tb2_pose(4:7, :)', 'r',"LineWidth", line_width);
hold on
plot(time_vec, estimated_b1Qb2(1:4, :), 'b',"LineWidth", line_width);
legend({'ekf cpp','','','','ekf matlab'});
title('b1Tb2 Quaternion Estimate');
grid on


