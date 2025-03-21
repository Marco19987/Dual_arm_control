% common variables
syms bQo [4 1] real % [qw qx qy qz]
syms ovo [3 1] real
syms oomegao [3 1] real
syms bg [3 1] real % gravity vector
syms Bm [6 6] real % inertia matrix
syms WR [6 12] real % grasp matrix dot R_bar
syms g1hg1 [6 1] real % wrench robot1
syms g2hg2 [6 1] real % wrench robot2
syms beta [6 6] real % air viscous friction matrix


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



