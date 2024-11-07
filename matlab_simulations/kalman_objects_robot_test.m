clear variables
close all
clc

% define parameters of the robots-object model
Bm = eye(6);
bg = [0;0;-9.8*0];
opg1 = [0;0.1;0];
opg2 = [0;-0.1;0];
oRg1 = eye(3);
oRg2 = eye(3);
n_pose_measures = 2;
b1pg1 = [0;0;0.5];
b1Qg1 = [1 0 0 0]';
b2pg2 = [0;0;0.5];
b2Qg2 = [1 0 0 0]';
b1pb2 = [0;1;0];
b1Qb2 = [1 0 0 0]';
bpb1 = [0 0 0]';
bQb1 = [1 0 0 0];

% Example usage of the KalmanFilter class
initialState = [0.1 0 0 1 0 0 0 0 0 0 0 0 0]';
sizeState = 13;
sizeOutput = 2 * n_pose_measures * 7;
sampleTime = 0.1;
system = RobotsObjectSystem(initialState, sizeState, sizeOutput,sampleTime,Bm,bg,opg1,opg2,oRg1,oRg2,n_pose_measures ...
                                            ,b1pg1, b1Qg1,b2pg2,b2Qg2,b1pb2,b1Qb2,bpb1,bQb1);

W = eye(sizeState); % Initial covariance noise matrix for state transition
V = eye(sizeOutput); % Initial covariance noise matrix for output

kf = KalmanFilter(system, W, V);

% Simulation parameters
numSteps = 100;
trueState = [0 0 0 1 0 0 0 0 0 0 0 0 0]';
u_k = [0.1 0 0 0 0 0.0 0 0 0 0 0 0]'; % Wrench applied by the robots in the grasp frames
W_k = eye(sizeState) * 0.01; % Updated covariance noise matrix for state transition
V_k = eye(sizeOutput) * 1; % Updated covariance noise matrix for output


% Storage for results
trueStates = zeros(sizeState, numSteps);
measurements = zeros(sizeOutput, numSteps);
filteredStates = zeros(sizeState, numSteps);
filteredMeasurements = zeros(sizeOutput, numSteps);


for k = 1:numSteps

    u_k = u_k*(k<numSteps/2) -u_k*(k>numSteps/2);
    % Simulate the true system
    trueState = system.state_fcn(trueState, u_k);
    measurement = system.output_fcn(trueState, u_k) + randn(sizeOutput, 1) * 0.1; % Add measurement noise
    
    % Apply the Kalman filter
    system.update_b1Tg1(b1pg1,b1Qg1); % added to remember that they need to be updated from the outside
    system.update_b2Tg2(b2pg2,b2Qg2); % added to remember that they need to be updated from the outside

    [filtered_measurement,filteredState] = kf.kf_apply(u_k, measurement, W_k, V_k);
    
    % Store results
    trueStates(:, k) = trueState;
    measurements(:, k) = measurement;
    filteredMeasurements(:,k) = filtered_measurement;
    filteredStates(:, k) = filteredState;
end

%% Plot results
figure;
subplot(3, 1, 1);
plot(1:numSteps, trueStates(1:3, :), 'g',1:numSteps, filteredStates(1:3, :), 'b');
legend('True State', 'Filtered State');
title('Position');


subplot(3, 1, 2);
plot(1:numSteps, trueStates(8:10, :), 'g',1:numSteps, filteredStates(8:10, :), 'b');
legend('True State', 'Filtered State');
title('Linear Velocity');

subplot(3, 1, 3);
plot(1:numSteps, trueStates(11:13, :), 'g',1:numSteps, filteredStates(11:13, :), 'b');
legend('True State', 'Filtered State');
title('Angular Velocity');

figure 
subplot(3, 1, 1);
plot(1:numSteps, measurements([1 2 3 8 9 10], :), 1:numSteps, filteredMeasurements([1 2 3 8 9 10], :));
legend('Measurements', 'Filtered Measurements');
title('Object position');
