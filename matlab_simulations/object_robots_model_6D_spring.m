clear variables
close all
clc

% stiffness and damping robot 1 - espressed in the grasp 1 frame
Klin_1 = 1000*eye(3);
Ktau_1 = 500*eye(3);
K_1 = blkdiag(Klin_1,Ktau_1);

Blin_1 = 50*eye(3);
Btau_1 = 50*eye(3);
B_1 = blkdiag(Blin_1,Btau_1); 

% stiffness and damping robot 2 - expressed in the grasp 2 frame
Klin_2 = 1000*eye(3);
Ktau_2 = 500*eye(3);
K_2 = blkdiag(Klin_2,Ktau_2);

Blin_2 = 50*eye(3);
Btau_2 = 50*eye(3);
B_2 = blkdiag(Blin_2,Btau_2);

% Object parameters
M = 10*eye(3);        % Mass matrix
I = 0.01*eye(3);      % Inertia matrix
Bm = blkdiag(M,I); 

% simulation parameters
tf = 1;    % final time 
dt = 0.001; % step size
time_vec = 0:dt:tf;
N_sample = length(time_vec);
bg = 0*[0; 0; -9.8];      % gravity acceleration in the base frame
eul_choice = "XYZ";       % eul2rotm([0 0 0],eul_choice);


% pose = [x y z phi theta psi]'; 
% the orientations are expressed as roll pitch and yaw
% velocity = [x_dot y_dot z_dot omegax omegay omegaz]


% define the robots velocity trajectory 
bx1_dot = repmat([0.01 0.01 0.0 0 0 0]',1,N_sample); % robot 1 end effector trajectory - bx1 is the robot pose in the base frame
bx2_dot = repmat([0.01 -0.01 -0.0 0 0 0]',1,N_sample); % same as before

% Initial conditions - b stands for base frame
bxobj_0 = [0 0 0 0 0 0]';       % object pose
bxobj_dot_0 = [0 0 0 0 0 0]';   % object velocity
bx1_0 = [0.3 0 0 0 0 0]';       % end effector 1 initial pose
bx2_0 = [-0.3 0 0 0 0 0]';      % end effector 2 initial pose
bxg1_0 = bx1_0;                 % initially grasp frame g1 and ee1 are coincident
bxg2_0 = bx2_0;                 % initially grasp frame g2 and ee2 are coincident


% compute grasp frames in the object frame
bRo = eul2rotm(bxobj_0(4:6)',eul_choice); % object frame in the base frame
opg1 = bRo'*(bxg1_0(1:3) - bxobj_0(1:3)); % constant - o stands for object frame
opg2 = bRo'*(bxg2_0(1:3) - bxobj_0(1:3)); % constant
bRg1 = eul2rotm(bxg1_0(4:6)',eul_choice); 
bRg2 = eul2rotm(bxg2_0(4:6)',eul_choice); 
oRg1 = bRo'* bRg1; % constant 
oRg2 = bRo'* bRg2; % constant




% simulation variables
x = zeros(12+6+6,N_sample); % position and velocity of the object, positions of the end effectors
x(:,1) = [bxobj_0;bxobj_dot_0;bx1_0;bx2_0];
u = [bx1_dot; bx2_dot];
u(:,round(length(u)/2):end) = 0;

for i=1:N_sample
    
    bpo = x(1:3,i);
    bRo = eul2rotm(x(4:6,i)',eul_choice);
    bp1 = x(13:15,i);
    bp2 = x(19:21,i);
    bpg1 = bpo + bRo * opg1;
    bpg2 = bpo + bRo * opg2;

    bpo_dot = x(7:9,i);
    bomegao = x(10:12,i);
    bp1_dot = u(1:3,i);
    bp2_dot = u(7:9,i);

    bpg1_dot = bpo_dot + skew(bomegao) * bRo * opg1;
    bpg2_dot = bpo_dot + skew(bomegao) * bRo * opg2;


    % wrench grasp frame 1 expressed in g1 : g1hg1
    g1Rb = oRg1' * bRo';
    g1fe_1 = K_1(1:3,1:3) * g1Rb * (- bpg1 + bp1); % elastic force 1 wrt g1
    g1fbeta_1 = B_1(1:3,1:3) * g1Rb * (- bpg1_dot + bp1_dot); % viscous force 1 wrt g1
    g1hg1 = [g1fe_1 + g1fbeta_1; [0 0 0]'];


    % wrench grasp frame 2 expressed in g2 : g2hg2
    g2Rb = oRg2' * bRo';
    g2fe_2 = K_2(1:3,1:3) * g2Rb * (- bpg2 + bp2); % elastic force 1 wrt g1
    g2fbeta_2 = B_2(1:3,1:3) * g1Rb * (- bpg2_dot + bp2_dot); % viscous force 1 wrt g1
    g2hg2 = [g2fe_2 + g2fbeta_2; [0 0 0]'];

    % compute grasp matrixs and Rbar matrix
    Wg1 = [eye(3), zeros(3); -skew(opg1)', eye(3)];
    Wg2 = [eye(3), zeros(3); -skew(opg2)', eye(3)];
    W = [Wg1,Wg2];

    Rbar = blkdiag(oRg1,oRg1,oRg2,oRg2);

    % object wrench
    oh = W * Rbar * [g1hg1;g2hg2];

    % xdot evaluation
    bxobj_dot_dot = blkdiag(bRo,bRo)*(Bm\oh) + [bg;0;0;0];
    Ta = rpy2jac(x(4:6,i)',eul_choice); % matrix relating omega_dot to the euler_angles_dot
    bxobj_dot = [x(7:9,i); Ta\x(10:12,i)];

    Ta_1 = rpy2jac(x(16:18,i)',eul_choice);
    bx1_dot = [bp1_dot; Ta_1\u(4:6,i)];

    Ta_2 = rpy2jac(x(22:24,i)',eul_choice);
    bx2_dot = [bp2_dot; Ta_2\u(10:12,i)];
    
    xdot = [bxobj_dot;bxobj_dot_dot;bx1_dot;bx2_dot];

    %xdot = spring_model(x(:,i),u(:,i), K_1, K_2, B_1, B_2, Bm, bg,eul_choice,opg1,opg2,oRg1,oRg2);   

    x(:,i+1) = x(:,i) + dt*xdot;


end

%% 
line_width = 1.5;
figure
subplot(2,2,1), plot(time_vec(1:end), x(1:3,1:end-1)',"LineWidth", line_width), title("Object Position"), legend("x","y","z"),grid on
subplot(2,2,2), plot(time_vec(1:end), x(4:6,1:end-1)',"LineWidth", line_width), title("Object Orientation"), legend("phi","theta","psi"),grid on
subplot(2,2,3), plot(time_vec(1:end), x(7:9,1:end-1)',"LineWidth", line_width), title("Object Velocity"), legend("vx","vy","vz"),grid on
subplot(2,2,4), plot(time_vec(1:end),  x(10:12,1:end-1)',"LineWidth", line_width), title("Object Angular Velocity"), legend("omegax","omegay","omegaz"),grid on

figure
subplot(2,2,1), plot(time_vec(1:end), x(13:15,1:end-1)',"LineWidth", line_width), title("Robot1 Position"), legend("x","y","z"),grid on
subplot(2,2,2), plot(time_vec(1:end), x(16:18,1:end-1)',"LineWidth", line_width), title("Robot1 Orientation"), legend("phi","theta","psi"),grid on
subplot(2,2,3), plot(time_vec(1:end), x(19:21,1:end-1)',"LineWidth", line_width), title("Robot2 Position"), legend("x","y","z"),grid on
subplot(2,2,4), plot(time_vec(1:end),  x(22:24,1:end-1)',"LineWidth", line_width), title("Robot2 Orientation"), legend("phi","theta","psi"),grid on


