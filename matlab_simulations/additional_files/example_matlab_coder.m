
x = zeros(27,1);
u = zeros(12,1);
K_1 = eye(6);
Transform = eye(4);
bpo_b = x(1:3);       % object's position in the base frame 
bQo = x(4:7);       % object's quaternion in the base frame  
ovo = x(8:10);      % object's linear velocity in the object frame
oomegao = x(11:13); % object's angular velocity in the object frame
b1pe1_b1 = x(14:16); % robot 1 end-effector position in the robot 1 base frame
b1Qe1 = x(17:20);    % robot 1 end-effector quaternion in the robot 1 base frame
b2pe2_b2 = x(21:23); % robot 2 end-effector position in the robot 2 base frame
b2Qe2 = x(24:27);    % robot 2 end-effector quaternion in the robot 2 base frame
b2pb1 = Transform(1:3,4);
b2Qb1 = rotm2quat(Transform(1:3,1:3))';

b1pe1_dot = u(1:3);  % robot 1 end-effector linear velocity in the robot 1 base frame
b1_omega_e1 = u(4:6); % robot 1 end-effector angular velocity in the robot 1 base frame
b2pe2_dot = u(7:9);  % robot 2 end-effector linear velocity in the robot 2 base frame
b2_omega_e2 = u(10:12); % robot 2 end-effector angular velocity in the robot 2 base frame
bpb1 = Transform(1:3,4); 
bQb1 = rotm2quat(Transform(1:3,1:3))'; 
opg1 = Transform(1:3,4);
oQg1 = rotm2quat(Transform(1:3,1:3))';
opg2 = Transform(1:3,4);
oQg2 = rotm2quat(Transform(1:3,1:3))';
K_1_diag = diag(K_1);
B_1_diag = diag(K_1);
K_2_diag = diag(K_1);
B_2_diag = diag(K_1);

jacobian = jacobian_h_to_x_state_not_ext(bpo_b,bQo,ovo,oomegao,b1pe1_b1,b1Qe1,b2pe2_b2,b2Qe2,b2pb1,b2Qb1, ...
                                b1pe1_dot,b1_omega_e1,b2pe2_dot,...
                        b2_omega_e2,bpb1, bQb1, opg1,oQg1,  opg2,oQg2,K_1_diag,B_1_diag,K_2_diag,B_2_diag);


jacobian = jacobian_h_to_b2Tb1(bpo_b,bQo,ovo,oomegao,b1pe1_b1,b1Qe1,b2pe2_b2,b2Qe2,b2pb1,b2Qb1, ...
                                b1pe1_dot,b1_omega_e1,b2pe2_dot,...
                        b2_omega_e2,bpb1, bQb1, opg1,oQg1,  opg2,oQg2,K_1_diag,B_1_diag,K_2_diag,B_2_diag);