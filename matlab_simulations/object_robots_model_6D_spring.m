clear variables
close all
clc

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

% Object parameters
M = 10*eye(3);        % Mass matrix
I = 0.01*eye(3);      % Inertia matrix
Bm = blkdiag(M,I); 

% simulation parameters
tf = 10;    % final time 
dt = 0.001; % step size
time_vec = 0:dt:tf;
N_sample = length(time_vec);
bg = 0*[0; 0; -9.8];      % gravity acceleration in the base frame
eul_choice = "XYZ";       % eul2rotm([0 0 0],eul_choice);


% pose = [x y z phi theta psi]'; 
% the orientations are expressed as roll pitch and yaw
% velocity = [x_dot y_dot z_dot omegax omegay omegaz]


% define the robots velocity trajectory 
bx1_dot = repmat([0 0.1 0 0 0.0 0]',1,N_sample); % robot 1 end effector trajectory - bx1 is the robot pose in the base frame
bx2_dot = repmat([0 -0.1 0 0 0 0]',1,N_sample); % same as before

% Initial conditions - b stands for base frame
bxobj_0 = [0 0 0 1 0 0 0]';       % object pose - quaternion - [qw qx qy qz] 
bxobj_dot_0 = [0 0 0 0 0 0]';   % object velocity - linear and angular velocity 
bx1_0 = [0.3 0 0 0 1 0 0]';       % end effector 1 initial pose
bx2_0 = [-0.3 0 0 1 0 0 0]';      % end effector 2 initial pose
bxg1_0 = bx1_0;                 % initially grasp frame g1 and ee1 are coincident
bxg2_0 = bx2_0;                 % initially grasp frame g2 and ee2 are coincident


% compute grasp frames in the object frame
% bRo = eul2rotm(bxobj_0(4:6)',eul_choice); % object frame in the base frame
bRo = quat2rotm(bxobj_0(4:7)'); % object frame in the base frame
opg1 = bRo'*(bxg1_0(1:3) - bxobj_0(1:3)); % constant - o stands for object frame
opg2 = bRo'*(bxg2_0(1:3) - bxobj_0(1:3)); % constant
bRg1 = quat2rotm(bxg1_0(4:7)'); 
bRg2 = quat2rotm(bxg2_0(4:7)'); 
oRg1 = bRo'* bRg1; % constant 
oRg2 = bRo'* bRg2; % constant




% simulation variables
x = zeros(7+6,N_sample); % pose and velocity of the object - pose has 7 elements (pos + quat), the velocity 6 (lin and ang vel)
x(:,1) = [bxobj_0;bxobj_dot_0];
u = [bx1_dot; bx2_dot];  % the robots are velocity controlled
u(:,round(length(u)/2):end) = 0;


% compute the corresponding robot poses to u
bx1 = zeros(7,N_sample);
bx2 = zeros(7,N_sample);
bx1(:,1) = bx1_0;
bx2(:,1) = bx2_0;
for i=1:N_sample
    bx1(:,i+1) = bx1(:,i) + dt*[u(1:3,i);Helper.quaternion_propagation(bx1(4:end,i),u(4:6,i))];
    bx2(:,i+1) = bx2(:,i) + dt*[u(7:9,i);Helper.quaternion_propagation(bx2(4:end,i),u(10:12,i))];
end 

oho = zeros(6, N_sample); 
for i=1:N_sample
    
    bpo = x(1:3,i);
    bRo = quat2rotm(x(4:7,i)');
    bp1 = bx1(1:3,i);
    bp2 = bx2(1:3,i);
    bpg1 = bpo + bRo * opg1;
    bpg2 = bpo + bRo * opg2;

    bpo_dot = x(8:10,i);
    bomegao = x(11:13,i);
    bp1_dot = u(1:3,i);
    bp2_dot = u(7:9,i);
    bomega1_dot = u(4:6,i);
    bomega2_dot = u(10:12,i);

    bpg1_dot = bpo_dot + skew(bomegao) * bRo * opg1;
    bpg2_dot = bpo_dot + skew(bomegao) * bRo * opg2;


    % wrench grasp frame 1 expressed in g1 : g1hg1
    g1Rb = oRg1' * bRo';
    g1fe_1 = K_1(1:3,1:3) * g1Rb * (- bpg1 + bp1); % elastic force 1 wrt g1
    g1fbeta_1 = B_1(1:3,1:3) * g1Rb * (- bpg1_dot + bp1_dot); % viscous force 1 wrt g1

    g1tau_beta_1 = B_1(4:6,4:6) * g1Rb * (- bomegao + bomega1_dot); % viscous torsional force 1 wrt g1

    g1hg1 = [g1fe_1 + g1fbeta_1; g1tau_beta_1 + [0 0 0]'];


    % wrench grasp frame 2 expressed in g2 : g2hg2
    g2Rb = oRg2' * bRo';
    g2fe_2 = K_2(1:3,1:3) * g2Rb * (- bpg2 + bp2); % elastic force 1 wrt g1
    g2fbeta_2 = B_2(1:3,1:3) * g2Rb * (- bpg2_dot + bp2_dot); % viscous force 1 wrt g1

    g2tau_beta_2 = B_2(4:6,4:6) * g2Rb * (- bomegao + bomega2_dot); % viscous torsional force 1 wrt g1

    g2hg2 = [g2fe_2 + g2fbeta_2; g2tau_beta_2 + [0 0 0]'];

    % compute grasp matrixs and Rbar matrix
    Wg1 = [eye(3), zeros(3); -skew(opg1)', eye(3)];
    Wg2 = [eye(3), zeros(3); -skew(opg2)', eye(3)];
    W = [Wg1,Wg2];

    Rbar = blkdiag(oRg1,oRg1,oRg2,oRg2);

    % object wrench
    oh = W * Rbar * [g1hg1;g2hg2];

    oho(:,i) = oh;
    % xdot evaluation
    bxobj_dot_dot = blkdiag(bRo,bRo)*(Bm\oh) + [bg;0;0;0];
    bxobj_dot = [bpo_dot; Helper.quaternion_propagation(x(4:7,i),bomegao)];
 
    xdot = [bxobj_dot;bxobj_dot_dot];

    %xdot = spring_model(x(:,i),u(:,i),bx1(:,i),bx2(:,i),K_1, K_2, B_1, B_2, Bm, bg,eul_choice,opg1,opg2,oRg1,oRg2);   

    x(:,i+1) = x(:,i) + dt*xdot;


end

%% 
line_width = 1.5;
figure
subplot(2,2,1), plot(time_vec(1:end), x(1:3,1:end-1)',"LineWidth", line_width), title("Object Position"), legend("x","y","z"),grid on
subplot(2,2,2), plot(time_vec(1:end), quat2eul(x(4:7,1:end-1)',eul_choice),"LineWidth", line_width), title("Object Orientation"), legend("phi","theta","psi"),grid on
subplot(2,2,3), plot(time_vec(1:end), x(8:10,1:end-1)',"LineWidth", line_width), title("Object Velocity"), legend("vx","vy","vz"),grid on
subplot(2,2,4), plot(time_vec(1:end),  x(11:13,1:end-1)',"LineWidth", line_width), title("Object Angular Velocity"), legend("omegax","omegay","omegaz"),grid on

figure
subplot(2,2,1), plot(time_vec(1:end), bx1(1:3,1:end-1)',"LineWidth", line_width), title("Robot1 Position"), legend("x","y","z"),grid on
subplot(2,2,2), plot(time_vec(1:end), quat2eul(bx1(4:7,1:end-1)', eul_choice),"LineWidth", line_width), title("Robot1 Orientation"), legend("phi","theta","psi"),grid on
subplot(2,2,3), plot(time_vec(1:end), bx2(1:3,1:end-1)',"LineWidth", line_width), title("Robot2 Position"), legend("x","y","z"),grid on
subplot(2,2,4), plot(time_vec(1:end),  quat2eul(bx2(4:7,1:end-1)', eul_choice),"LineWidth", line_width), title("Robot2 Orientation"), legend("phi","theta","psi"),grid on

%% Frames animation
figure
bTobj = Helper.transformation_matrix(x(1:3,1), x(4:7,1));
bT1 =  Helper.transformation_matrix(bx1(1:3,1), bx1(4:7,1));
bT2 =  Helper.transformation_matrix(bx2(1:3,1), bx2(4:7,1));

frame_obj = Frame(bTobj,"frame",'obj', 'color', 'b','text_opts', {'FontSize', 10, 'FontWeight', 'bold'});
hold on
frame_robot1 = Frame(bT1,"frame",'robot1', 'color', 'b','text_opts', {'FontSize', 10, 'FontWeight', 'bold'});
hold on
frame_robot2 = Frame(bT2,"frame",'robot2', 'color', 'b','text_opts', {'FontSize', 10, 'FontWeight', 'bold'});

for i=1:N_sample
    bTobj = Helper.transformation_matrix(x(1:3,i), x(4:7,i));
    bT1 =  Helper.transformation_matrix(bx1(1:3,i), bx1(4:7,i));
    bT2 =  Helper.transformation_matrix(bx2(1:3,i), bx2(4:7,i));

    frame_obj.move(bTobj);
    frame_robot1.move(bT1);
    frame_robot2.move(bT2);
    pause(dt)
end 

