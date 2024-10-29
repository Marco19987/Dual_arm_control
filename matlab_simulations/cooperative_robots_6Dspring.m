clear variables
close all
clc 

%% run the script create_robots.m to create the robots R1 and R2 in the workspace
create_robots

% set base frame and tools
bT_r1 = [[0 1 0; -1 0 0; 0 0 1] [0.8;0.0;0.31]; 0 0 0 1]; % transform between robot's frame 0 and base frame
R1.base = bT_r1;
r1_nT_ee1 = [eye(3) [0;0.0;0.145]; 0 0 0 1]; % transform between robot's last frame and tool
R1.tool = r1_nT_ee1;


bT_r2 = [eye(3) [-0.8;0.0; 0.34]; 0 0 0 1]; % transform between robot's frame 0 and base frame
R2.base = bT_r2;
r2_nT_ee2 = [eye(3) [0;0.0;0.19486]; 0 0 0 1]; % transform between robot's last frame and tool
R2.tool = r2_nT_ee2;


% plot
q0_1 = [-pi/2 pi/4 0 pi/4 0 pi/2 0];
R1.plot(q0_1)
hold on
q0_2 = [0 pi/4 0 deg2rad(121.3) 0 deg2rad(-75.6) 0];
R2.plot(q0_2)

Frame(eye(4), 'frame', 'base')

% create the cooperative_space_helper object to easily compute the
% cooperative space variables

coop_help = cooperative_space_helper(R1,R2);

% save end effectors pose wrt base frame 
pa_0 = coop_help.compute_pa(q0_1,q0_2);

% compute initial absolute rotation 
bRa_0 = coop_help.compute_Ra(q0_1,q0_2);


%% define the absolute and relative trajectories for the cooperative task space 
dt = 0.1;         % sample time [s]
tf = 10;           % trajectory duration [s]
Npoints = tf/dt;
time_vec = 0:dt:tf;
points_time = [0 tf]; % vector specifying the istants into which the waypoints of the trajectory have to be reached

% absolute trajectory
pa_f = pa_0 + [0*0.05;0*0.15;0];              % final position absolute frame wrt base frame
bRa_f = bRa_0*rotz(1*pi/4);           % final orientation absolute frame

bTa0 = [bRa_0 pa_0; 0 0 0 1];       % initial absolute pose 
bTaf = [bRa_0 pa_f; 0 0 0 1];       % final absolute pose

% absolute position trajectory
[bpa_traj, bpa_traj_dot, bpa_traj_dot_dot, pp] = quinticpolytraj([pa_0, pa_f],points_time,time_vec);

% absolute angular trajectory - see Siciliano, Robotica, pg.192-193
a0_R_af = bRa_0'*bRa_f;
[theta_0f, ra_0f] = tr2angvec(a0_R_af);
[theta_a_traj, theta_a_traj_dot, theta_a_traj_dot_dot, pp] = quinticpolytraj([0 theta_0f],points_time,time_vec);

b_omega_a_traj = zeros(3,Npoints); % vector of the desired angular velocities for the absolute frame
bRa_d_traj = zeros(3,3,Npoints);
for i=1:Npoints
    bRa_i = bRa_0*angvec2r(theta_a_traj(i), ra_0f);
    a_omega_i = theta_a_traj_dot(i) * ra_0f'; % angular velocity in the absolute frame
    b_omega_i = bRa_i * a_omega_i;            % angular velocity in the world frame
    b_omega_a_traj(1:3,i) = b_omega_i; % save the angular trajectory
    bRa_d_traj(:,:,i) = bRa_i;           % save the rotations
end 

% relative trajectory
bpr_0 = coop_help.compute_pr(q0_1,q0_2);          % relative position coordinate in the base frame
apr_0 = bRa_0' * bpr_0;    % better define the relative trajectory in the absolute frame and then convert it in the base frame
R12_0 = coop_help.compute_Rr(q0_1,q0_2); 

% aTr0 = [R12_0 [0;0.0;0.5]; 0 0 0 1];   % defined wrt absolute frame, to convert in the base frame. 
% aTrf = aTr0;                            % chosen costant -> tight grasp
% 
apr_d_traj = repmat(apr_0,1,Npoints);        % desired relative position trajectory for the robots in the absolute frame 
Rr_d_traj = repmat(R12_0,1,1,Npoints);
% 
% bTr_d_traj = aTr_d_traj*0;    % rotation in the base frame
% for i=1:Npoints
%     bTr_d_traj(:,:,i) = bTa_d_traj(:,:,i)*aTr_d_traj(:,:,i);
% end

% for the moment define the relative trajectory between the end effectors
% to be constant -> the grasp is tight 
bpr_traj_dot = zeros(3, Npoints);
b_omega_r_traj = zeros(3, Npoints);



%% IVERSE KINEMATIC
q = zeros(R1.n+R2.n, Npoints+1);
q_dot = zeros(R1.n+R2.n, Npoints);

q(:,1) = [q0_1'; q0_2'];

v = [bpa_traj_dot(:,1:end-1); b_omega_a_traj; bpr_traj_dot; b_omega_r_traj]; % desired velocities - the bpr_traj should contain a term depending on omega_a


K = 1*(1/dt)*diag(ones(1,12)); % CLIK gains

error_pa  = zeros(3,Npoints);
error_oa =  zeros(3,Npoints);
error_pr = zeros(3,Npoints);
error_or = zeros(3,Npoints);

% parameters for object simulation
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
eul_choice = "XYZ";       % eul2rotm([0 0 0],eul_choice);
bg = 0*[0; 0; -9.8];      % gravity acceleration in the base frame

% Initial conditions - b stands for base frame
bxobj_0 = [pa_0(1:3)',rotm2quat(bRa_0)]';       % object pose - quaternion - [qw qx qy qz] 
bxobj_dot_0 = [0 0 0 0 0 0]';                   % object velocity
bx1_0 = [R1.fkine(q0_1).t' rotm2quat(R1.fkine(q0_1).R)]';      % end effector 1 initial pose
bx2_0 = [R2.fkine(q0_2).t' rotm2quat(R2.fkine(q0_2).R)]';      % end effector 2 initial pose
bxg1_0 = bx1_0;                 % initially grasp frame g1 and ee1 are coincident
bxg2_0 = bx2_0;                 % initially grasp frame g2 and ee2 are coincident

% compute grasp frames in the object frame
bRo = quat2rotm(bxobj_0(4:7)'); % object frame in the base frame
opg1 = bRo'*(bxg1_0(1:3) - bxobj_0(1:3)); % constant - o stands for object frame
opg2 = bRo'*(bxg2_0(1:3) - bxobj_0(1:3)); % constant
bRg1 = quat2rotm(bxg1_0(4:7)'); 
bRg2 = quat2rotm(bxg2_0(4:7)'); 
oRg1 = bRo'* bRg1; % constant 
oRg2 = bRo'* bRg2; % constant


% grasp matrix
Wg1 = [eye(3), zeros(3); -skew(opg1)', eye(3)];
Wg2 = [eye(3), zeros(3); -skew(opg2)', eye(3)];
W = [Wg1,Wg2];

% Rbar matrix 
Rbar = blkdiag(oRg1,oRg1,oRg2,oRg2);

% simulation variables
x_obj = zeros(7+6,Npoints);             % pose and velocity of the object - pose has 7 elements (pos + quat), the velocity 6 (lin and ang vel)
x_obj(:,1) = [bxobj_0;bxobj_dot_0];  % initial condition
h_wrenches = zeros(12,Npoints);         %wrenches measured by the sensors in the grasp frames g1 and g2
oh_wrenches = h_wrenches;
oh_int = zeros(12,Npoints);             % internal forces in the grasp frames
vrd_int_wrench = zeros(6,Npoints);      % relative velocity control resulting by the control loop on the internal forces

Kc = blkdiag(1e-3*eye(3), 0.008*eye(3)); % internal wrench gain matrix 

% robot end effectors pose and velocity
bx1_dot = zeros(6,Npoints);
bx2_dot = zeros(6,Npoints);
bx1 = zeros(7,Npoints);
bx2 = zeros(7,Npoints);

for i=1:Npoints

    J1 = R1.jacob0(q(1:R1.n,i));
    J2 = R2.jacob0(q(R2.n+1:end,i));

    Ja = 0.5 * [J1, J2];    % absolute jacobian
    Jr = [-J1 J2];          % relative jacobian

    J = [Ja; Jr];

    % ERROR COMPUTATION - see Chiacchio96 - Direct and Inverse Kinematics for Coordinated IVIotion Tasl<s of a Two-Manipulator System 
    % absolute error
    bpa_i = coop_help.compute_pa(q(1:R1.n,i),q(R2.n+1:end,i));
    ep_a = bpa_traj(:,i) - bpa_i;

    bRa_i = coop_help.compute_Ra(q(1:R1.n,i),q(R2.n+1:end,i));
    bRa_d_i = bRa_d_traj(:,:,i);           % desired absolute orientation
    eo_a = 0.5*(skew(bRa_i(:,1))*bRa_d_i(:,1) + skew(bRa_i(:,2))*bRa_d_i(:,2) + skew(bRa_i(:,3))*bRa_d_i(:,3)); % absolute orientation error

    % relative error
    bpr_i = coop_help.compute_pr(q(1:R1.n,i),q(R2.n+1:end,i));
    ep_r = bRa_i*apr_d_traj(:,i) - bpr_i;

    bR1 = R1.fkine(q(1:R1.n)).R;
    Rr_i = coop_help.compute_Rr(q(1:R1.n,i),q(R2.n+1:end,i));
    Rr_d = Rr_d_traj(:,:,i);
    eo_r = 0.5*bR1*(skew(Rr_i(:,1))*Rr_d(:,1) + skew(Rr_i(:,2))*Rr_d(:,2) + skew(Rr_i(:,3))*Rr_d(:,3)); % absolute orientation error

    % desired absolute velocity
    vad = v(1:6,i);

    % desired relative velocity
    if (i==1)
        omega_a = zeros(3,1);
    else
        va = Ja*q_dot(:,i-1);
        omega_a = va(4:end);
    end
    vrd = [v(7:9,i)+skew(omega_a)*bRa_i*apr_d_traj(:,i);v(10:end,i)];


    % robot - object interaction simulation
    if (i==1)
        q_dot1 = zeros(R1.n,1);
        q_dot2 = zeros(R2.n,1);
    else
        q_dot1 = q_dot(1:R1.n,i-1);
        q_dot2 = q_dot(R2.n+1:end,i-1);
    end
    bx1_dot(:,i) = J1*q_dot1;
    bx2_dot(:,i) = J2*q_dot2;
    bx1(:,i) = [R1.fkine(q(1:R1.n,i)).t; rotm2quat(R1.fkine(q(1:R1.n,i)).R)'];
    bx2(:,i) = [R2.fkine(q(R2.n+1:end,i)).t; rotm2quat(R2.fkine(q(R2.n+1:end,i)).R)'];

    [x_obj_dot,h_wrenches(:,i)] = spring_model(x_obj(:,i),[bx1_dot(:,i);bx2_dot(:,i)],bx1(:,i),bx2(:,i),K_1, K_2, B_1, B_2, Bm, bg,eul_choice,opg1,opg2,oRg1,oRg2);

    tk = 0;
    x_obj_micro = x_obj(:,i);
    xdot_obj_rob_micro = x_obj_dot;
    micro_step = dt/100;
    while tk < dt
        tk = tk + micro_step;
        [xdot_obj_rob_micro,h_wrenches(:,i)] = spring_model(x_obj_micro,[bx1_dot(:,i);bx2_dot(:,i)],bx1(:,i),bx2(:,i), K_1, K_2, B_1, B_2, Bm, bg,eul_choice,opg1,opg2,oRg1,oRg2);
        x_obj_micro = x_obj_micro + micro_step*xdot_obj_rob_micro;
    end
    x_obj(:,i+1) = x_obj_micro;

    % internal forces computation
    oh_wrenches(:,i) = Rbar*h_wrenches(:,i);
    oh_int(:,i) = (eye(12) - pinv(W)*W)*oh_wrenches(:,i);
    %oh_int(:,i) = Rbar*oh_int(:,i); % internal wrenches rotated in the object frame
    bRo_i = quat2rotm(x_obj(4:7,i)');
    bRobar = blkdiag(bRo_i,bRo_i);
    vrd_int_wrench(:,i) = bRobar*1*Kc*(-(oh_int(7:end,i)-oh_int(1:6,i)));
    
    %vrd = [vrd_int_wrench(1:3,i)+skew(omega_a)*bRa_i*apr_d_traj(:,i);vrd_int_wrench(4:6,i)];
    vrd = vrd +  1*vrd_int_wrench(:,i);
    %vrd =  1*vrd_int_wrench(:,i);
    
    vad = vad*0 + 1*(i<Npoints/2)*[0.0 0.01 0 0 0.1 0]';

    % if (i>Npoints/2)
    %     i
    % end 

    q_dot(:,i) = pinv(J) * [vad;vrd];      
    %q_dot(:,i) = pinv(J) * ([vad;vrd] + K*[ep_a;eo_a;ep_r;eo_r]); 


    q(:,i+1) = q(:,i) + dt*q_dot(:,i);

    % save variables for plots
    error_pa(:,i) = ep_a;
    error_oa(:,i) = eo_a;
    error_pr(:,i) = ep_r;
    error_or(:,i) = eo_r;

    disp(dt*i)

end 



%% plot results

% for i=1:1:Npoints
%     pa = coop_help.compute_pa(q(1:R1.n,i)',q(R2.n+1:end,i));
%     error_pa(:,i) = bpa_traj(:,i) - pa;
% 
%     bRa_i = coop_help.compute_Ra(q(1:R1.n,i),q(R2.n+1:end,i));
%     bRa_d_i = bRa_d_traj(:,:,i);           % desired absolute orientation
%     error_oa(:,i) = 0.5*(skew(bRa_i(:,1))*bRa_d_i(:,1) + skew(bRa_i(:,2))*bRa_d_i(:,2) + skew(bRa_i(:,3))*bRa_d_i(:,3)); % absolute orientation error
% 
%     bpr_i = coop_help.compute_pr(q(1:R1.n,i),q(R2.n+1:end,i));
%     error_pr(:,i) = bRa_i*apr_d_traj(:,i) - bpr_i;
% 
% end
figure
subplot(2,2,1), plot(time_vec(1:end-1), error_pa), title("pa error"), grid on
subplot(2,2,2), plot(time_vec(1:end-1), error_oa), title("absolute frame orientation error"), grid on
subplot(2,2,3), plot(time_vec(1:end-1), error_pr), title("pr error"), grid on
subplot(2,2,4), plot(time_vec(1:end-1), error_or), title("relative orientation error"), grid on



%%
close all
R1.plot(q(1:R1.n,1)');
hold on
R2.plot(q(R2.n+1:end,1)');

for i=1:5:Npoints+1
    R1.plot(q(1:R1.n,i)');
    R2.plot(q(R2.n+1:end,i)');

end

%% plot forces and object position
line_width = 1.5;
figure
subplot(2,2,1), plot(time_vec(1:end-1), oh_wrenches(1:6,:)',"LineWidth", line_width), title("Elastic Force 1 (obj frame)"), legend("fx","fy","fz","tau_x", "tau_y", "tau_z"),grid on
subplot(2,2,2), plot(time_vec(1:end-1), oh_wrenches(7:12,:)',"LineWidth", line_width), title("Elastic Force 2 (obj frame)"), legend("fx","fy","fz","tau_x", "tau_y", "tau_z"),grid on
subplot(2,2,3), plot(time_vec(1:end-1), oh_int(1:6,:)',"LineWidth", line_width), title("Internal Forces 1"), legend("fx","fy","fz","tau_x", "tau_y", "tau_z"),grid on
subplot(2,2,4), plot(time_vec(1:end-1),  oh_int(7:end,:)',"LineWidth", line_width), title("Internal Force s 2"), legend("fx","fy","fz","tau_x", "tau_y", "tau_z"),grid on
figure, plot(time_vec(1:end-1),  vrd_int_wrench(:,:)',"LineWidth", line_width), title("relative velocity"), legend("vx","vy","vz","omega_x", "omega_y", "omega_z"),grid on

figure
subplot(2,2,1), plot(time_vec(1:end), x_obj(1:3,1:end)',"LineWidth", line_width), title("Object Position"), legend("x","y","z"),grid on
subplot(2,2,2), plot(time_vec(1:end), quat2eul(x_obj(4:7,1:end)',eul_choice),"LineWidth", line_width), title("Object Orientation"), legend("phi","theta","psi"),grid on
subplot(2,2,3), plot(time_vec(1:end), x_obj(8:10,1:end)',"LineWidth", line_width), title("Object Velocity"), legend("vx","vy","vz"),grid on
subplot(2,2,4), plot(time_vec(1:end),  x_obj(11:13,1:end)',"LineWidth", line_width), title("Object Angular Velocity"), legend("omegax","omegay","omegaz"),grid on

figure
subplot(2,2,1), plot(time_vec(1:end-1), bx1(1:3,1:end)',"LineWidth", line_width), title("Robot1 Position"), legend("x","y","z"),grid on
subplot(2,2,2), plot(time_vec(1:end-1), quat2eul(bx1(4:7,1:end)', eul_choice),"LineWidth", line_width), title("Robot1 Orientation"), legend("phi","theta","psi"),grid on
subplot(2,2,3), plot(time_vec(1:end-1), bx2(1:3,1:end)',"LineWidth", line_width), title("Robot2 Position"), legend("x","y","z"),grid on
subplot(2,2,4), plot(time_vec(1:end-1),  quat2eul(bx2(4:7,1:end)', eul_choice),"LineWidth", line_width), title("Robot2 Orientation"), legend("phi","theta","psi"),grid on

%% Frames animation
figure
bTobj = Helper.transformation_matrix(x_obj(1:3,1), x_obj(4:7,1));
bT1 =  Helper.transformation_matrix(bx1(1:3,1), bx1(4:7,1));
bT2 =  Helper.transformation_matrix(bx2(1:3,1), bx2(4:7,1));

frame_obj = Frame(bTobj,"frame",'obj', 'color', 'b','text_opts', {'FontSize', 10, 'FontWeight', 'bold'});
hold on
frame_robot1 = Frame(bT1,"frame",'robot1', 'color', 'b','text_opts', {'FontSize', 10, 'FontWeight', 'bold'});
hold on
frame_robot2 = Frame(bT2,"frame",'robot2', 'color', 'b','text_opts', {'FontSize', 10, 'FontWeight', 'bold'});

for i=1:Npoints
    bTobj = Helper.transformation_matrix(x_obj(1:3,i), x_obj(4:7,i));
    bT1 =  Helper.transformation_matrix(bx1(1:3,i), bx1(4:7,i));
    bT2 =  Helper.transformation_matrix(bx2(1:3,i), bx2(4:7,i));

    frame_obj.move(bTobj);
    frame_robot1.move(bT1);
    frame_robot2.move(bT2);
    pause(dt)
end 


