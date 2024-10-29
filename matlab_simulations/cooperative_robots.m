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
pa_f = pa_0 + [0;0*0.15;0];              % final position absolute frame wrt base frame
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

% spring stiffness
K_f = 1000*eye(3); % linear stiffness
K_tau = 1000*eye(3); % torsional stiffness
bh_i = zeros(6,Npoints); % vector to store the internal forces 

% internal wrench gain matrix 
Kc = blkdiag(1e-3*eye(3), 0.008*eye(3)); 
bh_d = zeros(6,1); % desired internal wrench. It sholud be 12 dim????
vrd_int_wrench = zeros(6,Npoints);

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


    % Internal wrench simulation with a 6D spring
    f_bi = K_f * (bpr_i - bRa_i*apr_0); % spring linear force - bRa_i*apr_0 is the spring rest position in the base frame
    tau_bi = K_tau * (0.5*bR1*(skew(Rr_i(:,1))*R12_0(:,1) + skew(Rr_i(:,2))*R12_0(:,2) + skew(Rr_i(:,3))*R12_0(:,3))); % spring torsional force 
    bh_i(:,i) = [f_bi;tau_bi];
    vrd_int_wrench(:,i) = Kc*(bh_d-bh_i(:,i));

    %q_dot(:,i) = pinv(J) * [vad;vrd];      
    q_dot(:,i) = pinv(J) * ([vad;vrd] + K*[ep_a;eo_a;ep_r;eo_r]); 


    q(:,i+1) = q(:,i) + dt*q_dot(:,i);

    % save variables for plots
    error_pa(:,i) = ep_a;
    error_oa(:,i) = eo_a;
    error_pr(:,i) = ep_r;
    error_or(:,i) = eo_r;

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