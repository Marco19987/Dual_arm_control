clear all
close all
clc


% simplified planar model
K1 = 5000*eye(2); % stiffness robot1
K2 = 5000*eye(2); % stiffness robot2

B1 = 1000*eye(2); % damping
B2 = 1000*eye(2);

delta_p1 = [0.1;0];   % spring 1 offset
delta_p2 = [-0.1;0];   % spring 2 offset

M = 10*eye(2); % Mass matrix
I = 0.01;      % Inertia
g = [0; -9.8]; 

% simulation parameters
tf = 10;    % final time 
dt = 0.001; % step size
time_vec = 0:dt:tf;
N_sample = length(time_vec);

x0 = [0 0 0 0 0 0]'; % [x y theta x_dot y_dot theta_dot]';
Nx = length(x0); % size x state
u = repmat(1*[delta_p1;delta_p2] + [0.0;0.01;0;-0.01],1,N_sample); % [p1, p2]' robots positions in the world frame 
udot = (diff(u')/dt)';
udot = [udot, udot(:,end)];

u_dot_single = [0.01;0;0;0];
udot = repmat(u_dot_single,1,N_sample);
u = zeros(4,N_sample+1);
u(:,1) = 1*[delta_p1;delta_p2] + [0.0;0.01;0;-0.01];
for i=1:N_sample
    u(:,i+1) = u(:,i) + udot(:,i)*dt;
end


x = zeros(Nx,N_sample);
x(:,1) = x0;

for i=1:N_sample

    theta = x(3,i);
    bRo = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    bp1 = u(1:2,i); 
    bp2 = u(3:4,i); 
    bpo = x(1:2,i); % object position world frame
    op1 = bRo'*(bp1-bpo);
    op2 = bRo'*(bp2-bpo);
    opo = bRo'*(bpo-bpo); % should be always 0

    bpo_dot = x(4:5,i); % object vel in the base frame
    opo_dot = bRo' * bpo_dot;

    bp1_dot = udot(1:2,i);
    bp2_dot = udot(3:4,i);
    op1_dot = bRo' * bp1_dot;
    op2_dot = bRo' * bp2_dot;

    omega_r_1 = skew([0,0,x(6,i)])*[delta_p1;0];
    omega_r_2 =  skew([0,0,x(6,i)])*[delta_p2;0];


    oF1_b = B1*(-(opo_dot + omega_r_1(1:2)) + op1_dot); % viscous force
    oF2_b = B2*(-(opo_dot + omega_r_2(1:2)) + op2_dot);

    oF1 = K1*(op1 - opo - delta_p1); % elastic forces robot 1
    oF2 = K2*(op2 - opo - delta_p2);

    % angular acceleration
    delta_1_cross_F1 = cross([delta_p1;0], [oF1;0]);
    delta_2_cross_F2 = cross([delta_p2;0], [oF2;0]);
    delta_1_cross_F1_B = cross([delta_p1;0], [oF1_b;0]);
    delta_2_cross_F2_B = cross([delta_p2;0], [oF2_b;0]);


    bp_o_dot_dot = bRo*inv(M)*(oF1 + oF2) + bRo*inv(M)*(oF1_b + oF2_b) + g;
    btheta_o_dot_dot = inv(I)*(delta_1_cross_F1(3) + delta_2_cross_F2(3) + delta_1_cross_F1_B(3) + delta_2_cross_F2_B(3));

    
    x(:,i+1) = x(:,i) + dt*[x(4:end,i);[bp_o_dot_dot;btheta_o_dot_dot]];
    
    % xdot = planar_spring_model(x(:,i),u(:,i),udot(:,i), K1, K2, B1, B2, delta_p1, delta_p2, M, I, g);
    % x(:,i+1) = x(:,i) + dt*xdot;

end




