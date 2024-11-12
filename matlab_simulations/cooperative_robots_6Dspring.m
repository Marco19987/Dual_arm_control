clear variables
close all
clc 

%% run the script create_robots.m to create the robots R1 and R2 in the workspace
create_robots

% set base frame and tools
bTb1 = [[0 1 0; -1 0 0; 0 0 1] [0.8;0.0;0.31]; 0 0 0 1]; % transform between robot's frame 0 and base frame
R1.base = bTb1;
r1_nT_ee1 = [eye(3) [0;0.0;0.145]; 0 0 0 1]; % transform between robot's last frame and tool
R1.tool = r1_nT_ee1;


bTb2 = [eye(3) [-0.8;0.0; 0.34]; 0 0 0 1]; % transform between robot's frame 0 and base frame
R2.base = bTb2;
r2_nT_ee2 = [eye(3) [0;0.0;0.19486]; 0 0 0 1]; % transform between robot's last frame and tool
R2.tool = r2_nT_ee2;

b1Tb2 = bTb1\(bTb2); % relative transform between robots


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
pa_f = pa_0 + [0*0.05;1*0.15;0];              % final position absolute frame wrt base frame
bRa_f = bRa_0*rotz(1*pi/4);           % final orientation absolute frame

bTa0 = [bRa_0 pa_0; 0 0 0 1];       % initial absolute pose 
bTaf = [bRa_0 pa_f; 0 0 0 1];       % final absolute pose

% absolute position trajectory
[bpa_traj, bpa_traj_dot, bpa_traj_dot_dot, pp] = quinticpolytraj([pa_0, pa_f],points_time,time_vec);

% absolute angular trajectory - see Siciliano, Robotica, pg.192-193
% a0_R_af = bRa_0'*bRa_f;
% [theta_0f, ra_0f] = tr2angvec(a0_R_af);
% [theta_a_traj, theta_a_traj_dot, theta_a_traj_dot_dot, pp] = quinticpolytraj([0 theta_0f],points_time,time_vec);
% 
% b_omega_a_traj = zeros(3,Npoints); % vector of the desired angular velocities for the absolute frame
% bRa_d_traj = zeros(3,3,Npoints);
% for i=1:Npoints
%     bRa_i = bRa_0*angvec2r(theta_a_traj(i), ra_0f);
%     a_omega_i = theta_a_traj_dot(i) * ra_0f'; % angular velocity in the absolute frame
%     b_omega_i = bRa_i * a_omega_i;            % angular velocity in the world frame
%     b_omega_a_traj(1:3,i) = b_omega_i; % save the angular trajectory
%     bRa_d_traj(:,:,i) = bRa_i;           % save the rotations
% end 
[b_omega_a_traj,bRa_d_traj] = Helper.axis_angle_trajectory(bRa_0,bRa_f,points_time,time_vec,Npoints);

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

% parameters for object trajectory control
bxobj_0 = [pa_0(1:3)',rotm2quat(bRa_0)]';       % object initial pose - quaternion - [qw qx qy qz] 
bxobj_dot_0 = [0 0 0 0 0 0]';                   % object initial velocity

bpobj_f = pa_0(1:3) + 1*[0,0.1,0]';                        % object desired final position
bRobj_f = bRa_0*rotx(0*pi/2);                               % object desired final orientation
bxobj_f = [bpobj_f; rotm2quat(bRobj_f)'];

bxobj_traj = zeros(7,Npoints);                  % object trajectory
[bpobj_traj, bpobj_traj_dot, bpobj_traj_dot_dot, pp] = quinticpolytraj([bxobj_0(1:3), bxobj_f(1:3)],points_time,time_vec);
bxobj_traj(1:3,:) = bpobj_traj(:,1:end-1);
[b_omega_obj_traj,bRobj_traj] = Helper.axis_angle_trajectory(quat2rotm(bxobj_0(4:7)'),quat2rotm(bxobj_f(4:7)'),points_time,time_vec,Npoints);
bxobj_traj(4:7,:) = rotm2quat(bRobj_traj)';


% Initial conditions - b stands for base frame
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
oTg1 = [oRg1, opg1; 0 0 0 1];
oTg2 = [oRg2, opg2; 0 0 0 1];



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


% object traking gain
Kot = blkdiag(1*eye(3),1*eye(3));


% KALMAN FILTER
% parameters for object pose estimation from mutliple pose
% and force measures 
n_pose_measures = 2;
b1Tg1 = bTb1 \ Helper.transformation_matrix(bxg1_0(1:3),bxg1_0(4:7));
b1pg1 = b1Tg1(1:3,4);
b1Qg1 = rotm2quat(b1Tg1(1:3,1:3))';
b2Tg2 = bTb2 \ Helper.transformation_matrix(bxg2_0(1:3),bxg2_0(4:7));
b2pg2 = b2Tg2(1:3,4);
b2Qg2 = rotm2quat(b2Tg2(1:3,1:3))';
b1pb2 = b1Tb2(1:3,4);
b1Qb2 = rotm2quat(b1Tb2(1:3,1:3))';
bpb1 = bTb1(1:3,4);
bQb1 =  rotm2quat(bTb1(1:3,1:3))';


sizeState = length(x_obj(:,1)); 
initialCondition = x_obj(:,1);
sizeOutput = 2 * n_pose_measures * 7;

viscous_friction = blkdiag(eye(3)*0.1, eye(3)*0.001); % viscous friction object-air


kalman_system = RobotsObjectSystem(initialCondition, sizeState, sizeOutput,dt,Bm,bg,opg1,opg2,oRg1,oRg2,n_pose_measures ...
                                            ,b1pg1, b1Qg1,b2pg2,b2Qg2,b1pb2,b1Qb2,bpb1,bQb1,viscous_friction);
% this system is used by the kalman filter to estimate the object pose and
% twist starting from force and pose measures

W_k = eye(sizeState)*1; % Initial covariance noise matrix for state transition
V_k = eye(sizeOutput)*1; % Initial covariance noise matrix for output

kf = KalmanFilter(kalman_system, W_k, V_k); 

x_obj_sim_kalman = zeros(sizeState,Npoints);
x_obj_sim_kalman(:,1) = initialCondition;
filtered_measurements = zeros(sizeOutput,Npoints);
filteredStates = zeros(sizeState,Npoints);

%%%%%%
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
    micro_step = 0.001;%dt/20;
    while tk < dt
        tk = tk + micro_step;
        [xdot_obj_rob_micro] = spring_model(x_obj_micro,[bx1_dot(:,i);bx2_dot(:,i)],bx1(:,i),bx2(:,i), K_1, K_2, B_1, B_2, Bm, bg,eul_choice,opg1,opg2,oRg1,oRg2);
        x_obj_micro = x_obj_micro + micro_step*xdot_obj_rob_micro;
    end
    x_obj(:,i+1) = x_obj_micro;


    % KALMAN FILTER SIMULATION
    kalman_system.getState()
    x_obj_sim_kalman(:,i+1) = kalman_system.state_fcn(kalman_system.getState(), h_wrenches(:,i)); % simulate the system again starting from the forces exerted by the springs. This should be done in a better way. x_obj and x_obj_sim_kalman should be the same
    measurement = kalman_system.output_fcn(x_obj_sim_kalman(:,i+1), h_wrenches(:,i)) + randn(sizeOutput, 1) * 0.0; % Add measurement noise
    
    

    [filtered_measurements(:,i),filteredStates(:,i)] = kf.kf_apply(h_wrenches(:,i), measurement, W_k, V_k);

    g1T1 = eye(4);
    b1Tg1 = inv(bTb1) * Helper.transformation_matrix(bx1(1:3,i),bx1(4:7,i)) * inv(g1T1);
    %b1Tg1 = bTb1 \ Helper.transformation_matrix(x_obj_sim_kalman(1:3,i+1),x_obj_sim_kalman(4:7,i+1)) * oTg1;
    b1pg1 = b1Tg1(1:3,4);
    b1Qg1 = rotm2quat(b1Tg1(1:3,1:3))';
    g2T2 = eye(4);
    b2Tg2 = inv(bTb2) * Helper.transformation_matrix(bx2(1:3,i),bx2(4:7,i)) * inv(g2T2);
    %b2Tg2 = bTb2 \ Helper.transformation_matrix(x_obj_sim_kalman(1:3,i+1),x_obj_sim_kalman(4:7,i+1)) * oTg2;
    b2pg2 = b2Tg2(1:3,4);
    b2Qg2 = rotm2quat(b2Tg2(1:3,1:3))';

    kalman_system.update_b1Tg1(b1pg1,b1Qg1); % added to remember that they need to be updated from the outside
    kalman_system.update_b2Tg2(b2pg2,b2Qg2); % added to remember that they need to be updated from the outside
    kalman_system.updateState(x_obj_sim_kalman(:,i+1)); % update the internal state of the system

    % INTERNAL FORCES COMPUTATION
    oh_wrenches(:,i) = Rbar*h_wrenches(:,i);
    oh_int(:,i) = (eye(12) - pinv(W)*W)*oh_wrenches(:,i);
    bRo_i = quat2rotm(x_obj(4:7,i)');
    bRobar = blkdiag(bRo_i,bRo_i);
    vrd_int_wrench(:,i) = bRobar*10*Kc*(-(oh_int(7:end,i)-oh_int(1:6,i)));
    
    
    vrd = 1*vrd +  0*vrd_int_wrench(:,i) + 0*[0 0 0 0 0 0.1]';
    %vrd =  1*vrd_int_wrench(:,i);
    
    vad = vad*0 + 0*(i<Npoints/2)*[0.0 0.01 0 0 0.0 0.1]';


    % OBJECT TRAJECTORY TRAKING CONTROL
    bTo_d = Helper.transformation_matrix(bxobj_traj(1:3,i),bxobj_traj(4:7,i)); % desired object pose in the base frame
    bTa = Helper.transformation_matrix(bpa_i, rotm2quat(bRa_i));
    bTo = Helper.transformation_matrix(x_obj(1:3,i),x_obj(4:7,i));
    oTa = bTo\bTa; 
    bTa_d = bTo_d*oTa;
    vad(1:3) = Kot(1:3,1:3)*(bTa_d(1:3,4)-bTa(1:3,4));
    aRa_d = bTa(1:3,1:3)'*bTa_d(1:3,1:3);
    [atheta_aa_d, ar_aa_d] = tr2angvec(aRa_d./vecnorm(aRa_d)');
    a_error_aa_d = atheta_aa_d * ar_aa_d'; % error a-a_d in the absolute frame
    b_error_aa_d = bRa_i*a_error_aa_d;
    vad(4:6) = Kot(4:6,4:6)*(b_error_aa_d);
    
    
    
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
frame_to_show = x_obj_sim_kalman(1:7,:);%x_obj;
figure
R1.plot(q(1:R1.n,1)');
hold on
R2.plot(q(R2.n+1:end,1)');
bTobj = Helper.transformation_matrix(frame_to_show(1:3,1), frame_to_show(4:7,1));
frame_obj = Frame(bTobj,"frame",'obj', 'color', 'b','text_opts', {'FontSize', 10, 'FontWeight', 'bold'});
hold on

for i=1:5:Npoints+1
    R1.plot(q(1:R1.n,i)');
    R2.plot(q(R2.n+1:end,i)');
    bTobj = Helper.transformation_matrix(frame_to_show(1:3,i), frame_to_show(4:7,i));
    frame_obj.move(bTobj);
    pause(dt)

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


