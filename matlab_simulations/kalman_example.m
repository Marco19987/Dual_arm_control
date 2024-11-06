clear variables
close all
clc


% Example usage of the KalmanFilter class
initialState = [10,1]';
sizeState = 2;
sizeOutput = 1;
sampleTime = 0.1;
system = SimpleSystem(initialState, sizeState, sizeOutput,sampleTime);

W = eye(sizeState); % Initial covariance noise matrix for state transition
V = eye(sizeOutput); % Initial covariance noise matrix for output

kf = KalmanFilter(system, W, V);

% Simulation parameters
numSteps = 100;
trueState = 0.1*initialState;
u_k = [0; 0]; % Example input
W_k = eye(sizeState) * 1; % Updated covariance noise matrix for state transition
V_k = eye(sizeOutput) * 100; % Updated covariance noise matrix for output


% Storage for results
trueStates = zeros(sizeState, numSteps);
measurements = zeros(sizeOutput, numSteps);
filteredStates = zeros(sizeState, numSteps);
filteredMeasurements = zeros(sizeOutput, numSteps);


for k = 1:numSteps

    u_k = sin((k-1)*0.1)*[1;1]*1;
    % Simulate the true system
    trueState = system.state_fcn(trueState, u_k);
    measurement = system.output_fcn(trueState, u_k) + randn(sizeOutput, 1) * 1; % Add measurement noise
    
    % Apply the Kalman filter
    [filtered_measurement,filteredState] = kf.kf_apply(u_k, measurement, W_k, V_k);
    
    % Store results
    trueStates(:, k) = trueState;
    measurements(:, k) = measurement;
    filteredMeasurements(:,k) = filtered_measurement;
    filteredStates(:, k) = filteredState;
end

% Plot results
figure;
subplot(3, 1, 1);
plot(1:numSteps, trueStates(1, :), 'g',1:numSteps, filteredStates(1, :), 'b');
legend('True State', 'Filtered State');
title('State 1');


subplot(3, 1, 2);
plot(1:numSteps, trueStates(2, :), 'g',1:numSteps, filteredStates(2, :), 'b');
legend('True State', 'Filtered State');
title('State 2');

subplot(3, 1, 3);
plot(1:numSteps, measurements(1, :), 'g', 1:numSteps, filteredMeasurements(1, :), 'b');
legend('Measurements', 'Filtered Measurements');
title('Output 1');
