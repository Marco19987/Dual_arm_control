clear variables
close all
clc

%% define parameters of the robots-object model
M = 1*eye(3);        % Mass matrix
I = 0.1*eye(3);      % Inertia matrix
Bm = blkdiag(M,I);
viscous_friction = blkdiag(eye(3)*0.1, eye(3)*0.001); % viscous friction object-air
n_pose_measures = 2;
bg = [0;0;-9.8*0];
opg1 = [0;0.1;0];
opg2 = [0;-0.1;0];
oRg1 = eye(3);
oRg2 = eye(3);
oTg1 = Helper.transformation_matrix(opg1,rotm2quat(oRg1));
oTg2 = Helper.transformation_matrix(opg2,rotm2quat(oRg2));

bpb1 = [0.8;0.0;0.31];
bQb1 = rotm2quat([0 1 0; -1 0 0; 0 0 1])';
bTb2 = [eye(3) [-0.8;0.0; 0.34]; 0 0 0 1];
bpb2 = bTb2(1:3,4);
bQb2 = rotm2quat(bTb2(1:3,1:3))';

bTo = Helper.transformation_matrix([0 0 0]',[1 0 0 0]');

bTb1 = Helper.transformation_matrix(bpb1,bQb1);

b1Tb2 = bTb1 \ bTb2;
b1pb2 = b1Tb2(1:3,4);
b1Qb2 = rotm2quat(b1Tb2(1:3,1:3))';

b1Tg1 = bTb1 \ bTo * oTg1; 
b2Tg2 = bTb2 \ bTo * oTg2; 
b1pg1 = b1Tg1(1:3,4);
b1Qg1 = rotm2quat(b1Tg1(1:3,1:3))';
b2pg2 = b2Tg2(1:3,4);
b2Qg2 = rotm2quat(b2Tg2(1:3,1:3))';


% Example usage of the KalmanFilter class
initialState = [bTo(1:3,4)' rotm2quat(bTo(1:3,1:3)) 0 0 0 0 0 0]';
sizeState = 13;
sizeOutput = 2 * n_pose_measures * 7;
sampleTime = 0.1;
system = RobotsObjectSystem(initialState, sizeState, sizeOutput,sampleTime,Bm,bg,opg1,opg2,oRg1,oRg2,n_pose_measures ...
                                            ,b1pg1, b1Qg1,b2pg2,b2Qg2,b1pb2,b1Qb2,bpb1,bQb1,viscous_friction);

W = eye(sizeState); % Initial covariance noise matrix for state transition
V = eye(sizeOutput); % Initial covariance noise matrix for output

kf = KalmanFilter(system, W, V);

% Simulation parameters
tf = 10;
time_vec = 0:sampleTime:tf-sampleTime;
numSteps = length(time_vec);

trueState = [0 0 0 1 0 0 0 0 0 0 0 0 0]';
u_k_fixed = [0.1 0.1 0.1 0 0 0 0.1 0 0 0 0 0]'; % Wrench applied by the robots in the grasp frames
W_k = eye(sizeState) * 0.1; % Updated covariance noise matrix for state transition
V_k = eye(sizeOutput) * 1; % Updated covariance noise matrix for output


% Storage for results
trueStates = zeros(sizeState, numSteps);
measurements = zeros(sizeOutput, numSteps);
filteredStates = zeros(sizeState, numSteps);
filteredMeasurements = zeros(sizeOutput, numSteps);

kf.system.updateState(initialState);
filteredState = initialState;


for k = 1:numSteps
    disp(k)

    u_k = u_k_fixed*(k<numSteps/2) - u_k_fixed*(k>=numSteps/2);
    % Simulate the true system
    prevState = system.getState();
    trueState = system.state_fcn(prevState, u_k);
    measurement = system.output_fcn(trueState, u_k) + randn(sizeOutput, 1) * 0.00 + 0.01*repmat([randn(3, 1)' 0.0*randn(4, 1)']',2*n_pose_measures,1); % Add measurement noise

    %simulate occlusion of estimators
    if(k>numSteps/2)
        measurement(1:7) = zeros(7,1);
        measurement(8:14) = zeros(7,1);
        measurement(15:21) = zeros(7,1); 
    end



    % update V_k in correspondence of the occlusions detected
    if(k>numSteps/2)
        V_k(1:7,1:7) = eye(7)*100; 
        V_k(8:14,8:14) = eye(7)*100; 
        V_k(15:21,15:21) = eye(7)*100; 
        % V_k(22:28,22:28) = eye(7)*0.01; 
    end

    
    
    [filtered_measurement,filteredState] = kf.kf_apply(u_k, measurement, W_k, V_k);

    % Apply the Kalman filter
    b1Tg1 = bTb1 \ Helper.transformation_matrix(trueState(1:3),trueState(4:7)) * oTg1;
    b1pg1 = b1Tg1(1:3,4);
    b1Qg1 = rotm2quat(b1Tg1(1:3,1:3))';
    b2Tg2 = bTb2 \ Helper.transformation_matrix(trueState(1:3),trueState(4:7)) * oTg2;

    b2pg2 = b2Tg2(1:3,4);
    b2Qg2 = rotm2quat(b2Tg2(1:3,1:3))';
    system.update_b1Tg1(b1pg1,b1Qg1); % added to remember that they need to be updated from the outside
    system.update_b2Tg2(b2pg2,b2Qg2); % added to remember that they need to be updated from the outside


    % g1To_filtered = Helper.transformation_matrix(filtered_measurement(1:3),filtered_measurement(4:7));
    % g2To_filtered = Helper.transformation_matrix(filtered_measurement(end-6:end-4),filtered_measurement(end-3:end));
    filteredState(4:7) = filteredState(4:7)/(norm(filteredState(4:7)));
    b1Tg1 = bTb1 \ Helper.transformation_matrix(filteredState(1:3),filteredState(4:7)) * oTg1;
    b1pg1 = b1Tg1(1:3,4);
    b1Qg1 = rotm2quat(b1Tg1(1:3,1:3))';
   
    b2Tg2 = bTb2 \ Helper.transformation_matrix(filteredState(1:3),filteredState(4:7)) * oTg2;
    b2pg2 = b2Tg2(1:3,4);
    b2Qg2 = rotm2quat(b2Tg2(1:3,1:3))';

    kf.system.update_b1Tg1(b1pg1,b1Qg1); % added to remember that they need to be updated from the outside
    kf.system.update_b2Tg2(b2pg2,b2Qg2); % added to remember that they need to be updated from the outside

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
figure;
line_width = 1;
subplot(3, 1, 1);
plot(1:numSteps, trueStates(1:3, :), 'g',1:numSteps, filteredStates(1:3, :), 'b',"LineWidth", line_width);
legend('True State', 'Filtered State');
title('Position');
grid on


subplot(3, 1, 2);
plot(1:numSteps, trueStates(8:10,:), 'g',1:numSteps, filteredStates(8:10, :), 'b',"LineWidth", line_width);
legend('True State', 'Filtered State');
title('Linear Velocity');
grid on

subplot(3, 1, 3);
plot(1:numSteps, trueStates(11:13, :), 'g',1:numSteps, filteredStates(11:13, :), 'b',"LineWidth", line_width);
legend('True State', 'Filtered State');
title('Angular Velocity');
grid on

figure
plot(1:numSteps, trueStates(4:7, :), 'g',1:numSteps, filteredStates(4:7, :), 'b',"LineWidth", line_width);
legend('True State', 'Filtered State');
title('Quaternion Components');
grid on

figure 
subplot(3, 1, 1);
plot(1:numSteps, measurements([1 2 3 8 9 10], :), 1:numSteps, filteredMeasurements([1 2 3 8 9 10], :),"LineWidth", line_width);
legend('Measurements', 'Filtered Measurements');
title('Object position');
grid on
