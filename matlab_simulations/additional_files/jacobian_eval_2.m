% common variables
syms bpo [3 1] real
syms bQo [4 1] real % [qw qx qy qz]
syms ovo [3 1] real
syms oomegao [3 1] real
syms bg [3 1] real % gravity vector
syms Bm [6 6] real % inertia matrix
syms WR [6 12] real % grasp matrix dot R_bar
syms g1hg1 [6 1] real % wrench robot1
syms g2hg2 [6 1] real % wrench robot2
syms beta [6 6] real % air viscous friction matrix
syms b1pe1_b1 [3 1] real % robot 1 end-effector position in the robot 1 base frame
syms b1Qe1 [4 1] real    % robot 1 end-effector quaternion in the robot 1 base frame
syms b2pe2_b2 [3 1] real % robot 2 end-effector position in the robot 2 base frame
syms b2Qe2 [4 1] real    % robot 2 end-effector quaternion in the robot 2 base frame
syms b1pe1_dot [3 1] real  % robot 1 end-effector linear velocity in the robot 1 base frame
syms b1_omega_e1 [3 1] real % robot 1 end-effector angular velocity in the robot 1 base frame
syms b2pe2_dot [3 1] real  % robot 2 end-effector linear velocity in the robot 2 base frame
syms b2_omega_e2 [3 1] real % robot 2 end-effector angular velocity in the robot 2 base frame



bQo_ = UnitQuaternion(bQo);
bRo = bQo_.R;
h = [g1hg1;g2hg2];
bRo_bar = blkdiag(bRo,bRo);
otwisto = [ovo;oomegao];



%% jacobian bpo_dot -> bQo
bpo_dot = bRo*ovo;

Jacobian_bpo_dot_to_bQo = jacobian(bpo_dot(:), bQo);
disp(Jacobian_bpo_dot_to_bQo);
% ccode(Jacobian_bpo_dot_to_bQo)
% matlabFunction(Jacobian_bpo_dot_to_bQo,"File","Jacobian_bpo_dot_to_bQo","Vars",{[bQo],[ovo]})

%% jacobian bQo_dot
E_oomegao = [0 oomegao'; -oomegao skew(oomegao)];
bQo_dot = -0.5 * E_oomegao * bQo;

% bQo_dot -> bQo
Jacobian_bQo_dot_to_bQo = jacobian(bQo_dot(:), bQo);
disp(Jacobian_bQo_dot_to_bQo);
% matlabFunction(Jacobian_bQo_dot_to_bQo,"File","Jacobian_bQo_dot_to_bQo","Vars",{[oomegao]})

% bQo_dot -> oomegao
Jacobian_bQo_dot_to_oomegao = jacobian(bQo_dot(:), oomegao);
disp(Jacobian_bQo_dot_to_oomegao);
% matlabFunction(Jacobian_bQo_dot_to_oomegao,"File","Jacobian_bQo_dot_to_oomegao","Vars",{[bQo]})

%% Jacobian otwisto_dot 
double_skew = [skew(oomegao), zeros(3); skew(ovo),skew(oomegao)];

gravity_term = bRo_bar'*[bg;0;0;0];
otwisto_skew_term = -double_skew*Bm*otwisto; % remember to pre-multiply Bm^-1
otwisto_viscous_term = - beta*otwisto;          % remember to pre-multiply Bm^-1

% otwisto_dot = Bm\(WR*h) + gravity_term + Bm\(otwisto_viscous_term + otwisto_skew_term);

% otwisto_dot -> bQo 
Jacobian_otwisto_dot_to_bQo = jacobian(gravity_term(:), bQo);
disp(Jacobian_otwisto_dot_to_bQo);
% matlabFunction(Jacobian_otwisto_dot_to_bQo,"File","Jacobian_otwisto_dot_to_bQo","Vars",{[bQo],[bg]})

% otwisto_dot -> otwisto skew term
Jacobian_otwisto_dot_to_otwisto_skew_term = jacobian(otwisto_skew_term(:), otwisto);
disp(Jacobian_otwisto_dot_to_otwisto_skew_term);
% matlabFunction(Jacobian_otwisto_dot_to_otwisto_skew_term,"File","Jacobian_otwisto_dot_to_otwisto_skew_term","Vars",{[Bm],[otwisto]})

%% Jacobian product between two quaternions
syms Q1 [4 1] real % [qw qx qy qz]
syms Q2 [4 1] real 

Q1_ = UnitQuaternion(Q1);
Q2_ = UnitQuaternion(Q2);
fun = Q1_*Q2_;

Jacobian_quaternion_product_right = jacobian([fun.s fun.v], Q2);
disp(Jacobian_quaternion_product_right);
% matlabFunction(Jacobian_quaternion_product_right,"File","Jacobian_quaternion_product_right","Vars",{[Q1]})

Jacobian_quaternion_product_left = jacobian([fun.s fun.v], Q1);
disp(Jacobian_quaternion_product_left);
% matlabFunction(Jacobian_quaternion_product_left,"File","Jacobian_quaternion_product_left","Vars",{[Q2]})

%% Spring Wrench Jacobian gihgi
syms opgi [3 1] real
syms oQgi [4 1] real % [qw qx qy qz]
syms bpb1 [3 1] real
syms bQb1 [4 1] real % [qw qx qy qz]
syms b2pb1 [3 1] real
syms b2Qb1 [4 1] real % [qw qx qy qz]
syms b2pei_b2 [3 1] real

b1Qe1_ = UnitQuaternion(b1Qe1);
b2Qe2_ = UnitQuaternion(b2Qe2);

bQb1_ = UnitQuaternion(bQb1);
bRb1 = bQb1_.R;
bTb1 = [bRb1 bpb1; 0 0 0 1];

oQgi_ = UnitQuaternion(oQgi);
oRgi = oQgi_.R;
oTgi = [oRgi opgi; 0 0 0 1];

b2Qb1_ = UnitQuaternion(b2Qb1);
b2Rb1 = b2Qb1_.R;
b2Tb1 = [b2Rb1 b2pb1; 0 0 0 1];
b1Tb2 = inv(b2Tb1);


% bpe1_b = bTb1(1:3,4) + bTb1(1:3,1:3) * (b1pe1_b1);
bpei_b = bTb1(1:3,4) + bTb1(1:3,1:3) * (b1Tb2(1:3,4) + b1Tb2(1:3,1:3)*b2pei_b2); % when b1Tb2 = eye(4) => b1pe1

bpgi_b = bpo + bRo * oTgi(1:3,4);
gifgi_e =  bRo' * (bpei_b-bpgi_b); % elastic force wrt gi 
% IMPORTANT!!!   omissing K*oRgi' remember to multiply for K*oRgi' the
% resulting Jacobians
% the resulting jacobians can be used for both g1hg1 and g2hg2 but, when
% evaluating g1hg1 the b2Tb1 (b2Qb1 identity) parameter of the functions must be set to
% eye(4)!!!

syms b2pei_dot [3 1] real 
bpei_dot = bTb1(1:3,1:3)*b1Tb2(1:3,1:3)*b2pei_dot;
bpgi_dot = bRo*ovo + skew(bRo*oomegao) * bRo * oTgi(1:3,4);
gifgi_beta =  bRo' * (bpei_dot-bpgi_dot); % viscous force wrt gi
% IMPORTANT!!!   omissing B*oRgi' remember to multiply for B*oRgi' the
% resulting Jacobians
% the resulting jacobians can be used for both g1hg1 and g2hg2 but, when
% evaluating g1hg1 the b2Tb1 (b2Qb1 identity) parameter of the functions must be set to
% eye(4)!!!

syms b2Qei [4 1] real
b2Qei_ = UnitQuaternion(b2Qei);
giQb = oQgi_.inv * bQo_.inv;
giQei = giQb * bQb1_ * b2Qb1_.inv * b2Qei_;
gi_tau_gi_e = giQei.v'; % elastic torsional force wrt gi
% IMPORTANT!!!   omissing K(4:6,4:6) remember to multiply for K(4:6,4:6) the
% resulting Jacobians
% the resulting jacobians can be used for both g1hg1 and g2hg2 but, when
% evaluating g1hg1 the b2Tb1 (b2Qb1 identity) parameter of the functions must be set to
% eye(4)!!!

syms b2_omega_ei [3 1] real
gi_tau_gi_beta = bRo' * (bTb1(1:3,1:3)*b1Tb2(1:3,1:3)*b2_omega_ei - bRo*oomegao); % viscous torsional force wrt g1
% IMPORTANT!!!   omissing 
% B(4:6,4:6)*oRgi' remember to multiply for B_1(4:6,4:6)*oRgi' the
% resulting Jacobians
% the resulting jacobians can be used for both g1hg1 and g2hg2 but, when
% evaluating g1hg1 the b2Tb1 (b2Qb1 identity) parameter of the functions must be set to
% eye(4)!!!


gihgi = [gifgi_e + gifgi_beta ; gi_tau_gi_e + gi_tau_gi_beta ];

% jacobian gifgi_e-> bpo
Jacobian_gifgi_e_to_bpo = jacobian(gifgi_e(:), bpo); % -K*oRgi'*bRo'

% jacobian gifgi_e -> bQo
Jacobian_gifgi_e_to_bQo = jacobian(gifgi_e(:), bQo);
%matlabFunction(Jacobian_gifgi_e_to_bQo,"File","Jacobian_gifgi_e_to_bQo","Vars",{[b2Qb1],[b2pb1],[b2pei_b2],[bQb1],[bpb1],[bQo],[bpo],[opgi]})

% jacobian gifgi_e -> b2pei_b2 (b1pe1)
Jacobian_gifgi_e_to_b2pei = jacobian(gifgi_e(:), b2pei_b2);
%matlabFunction(Jacobian_gifgi_e_to_b2pei,"File","Jacobian_gifgi_e_to_b2pei","Vars",{[b2Qb1],[bQb1],[bQo]})

% Jacobian_gifgi_beta -> bQo
Jacobian_gifgi_beta_to_bQo = jacobian(gifgi_beta(:), bQo);
%matlabFunction(Jacobian_gifgi_beta_to_bQo,"File","Jacobian_gifgi_beta_to_bQo","Vars",{})

% Jacobian_gifgi_beta -> otwisto
Jacobian_gifgi_beta_to_otwisto = jacobian(gifgi_beta(:), otwisto);
%matlabFunction(Jacobian_gifgi_beta_to_otwisto,"File","Jacobian_gifgi_beta_to_otwisto","Vars",{})

% Jacobian gi_tau_gi_e -> bQo
Jacobian_gi_tau_gi_e_to_bQo = jacobian(gi_tau_gi_e(:), bQo);
% matlabFunction(Jacobian_gi_tau_gi_e_to_bQo,"File","Jacobian_gi_tau_gi_e_to_bQo","Vars",{})

% Jacobian gi_tau_gi_e -> b2Qei
Jacobian_gi_tau_gi_e_to_b2Qei = jacobian(gi_tau_gi_e(:), b2Qei);
% matlabFunction(Jacobian_gi_tau_gi_e_to_b2Qei,"File","Jacobian_gi_tau_gi_e_to_b2Qei","Vars",{})

% Jacobian gi_tau_gi_beta -> bQo
Jacobian_gi_tau_gi_beta_to_bQo = jacobian(gi_tau_gi_beta(:), bQo);
% matlabFunction(Jacobian_gi_tau_gi_beta_to_bQo,"File","Jacobian_gi_tau_gi_beta_to_bQo","Vars",{})

% Jacobian gi_tau_gi_beta -> otwisto
Jacobian_gi_tau_gi_beta_to_otwisto = jacobian(gi_tau_gi_beta(:), otwisto);
% matlabFunction(Jacobian_gi_tau_gi_beta_to_otwisto,"File","Jacobian_gi_tau_gi_beta_to_otwisto","Vars",{})





% sym_variables = [bpo; bQo; ovo; oomegao; b2pei_b2; b2Qei; b2pb1; b2Qb1; bpb1; bQb1; b2pei_dot;b2_omega_ei; opgi;oQgi];
% jacobian_gihgi_to_x_state = jacobian(gihgi(:),sym_variables);
% matlabFunction(jacobian_gihgi_to_x_state,"File","jacobian_gihgi_to_x_state","Vars",{bpo; bQo; ovo; oomegao; b2pei_b2; b2Qei; b2pb1; b2Qb1; bpb1; bQb1; b2pei_dot;b2_omega_ei; opgi;oQgi})






