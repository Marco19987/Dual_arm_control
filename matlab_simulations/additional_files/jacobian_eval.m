%% Quaternion dot jacobian
syms omega1 omega2 omega3 real
syms qw qx qy qz real

% Definire il vettore omega
omega = [omega1; omega2; omega3];

% Definire la matrice antisimmetrica skew(omega)
skew_omega = [  0,     -omega3,  omega2;
              omega3,     0,    -omega1;
             -omega2,  omega1,     0   ];

% Definire la matrice F
F = 0.5 * [1, omega1, omega2, omega3;
           omega1, 0, -omega3, omega2;
           omega2, omega3, 0, -omega1;
           omega3, -omega2, omega1, 0];

F = F * [qw,qx,qy,qz]';


% Calcolare la Jacobiana di F rispetto a omega
Jacobian_F = jacobian(F(:), omega);

% Visualizzare la Jacobiana
disp(Jacobian_F);


%% dinamycs jacobian
clear variables 
close all 
clc

% Define the symbolic quaternion components
syms qw qx qy qz real
syms a [6 1] real 
% Define the quaternion as a symbolic vector
quaternion = [qw; qx; qy; qz];

% Create the rotation matrix from the quaternion
R = [1 - 2*qy^2 - 2*qz^2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw;
     2*qx*qy + 2*qz*qw, 1 - 2*qx^2 - 2*qz^2, 2*qy*qz - 2*qx*qw;
     2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx^2 - 2*qy^2];

% Display the rotation matrix
disp('Rotation Matrix:');
disp(R);

F = [R zeros(3); zeros(3) R]*a;

Jacobian_F = jacobian(F(:), quaternion);

%% compute output jacobian
clear variables 
close all 
clc

% Define the symbolic quaternion components
syms p [3 1] real 
syms qw qx qy qz real
syms gkTb [4 4]
syms epsilon
% Define the quaternion as a symbolic vector
quaternion = [qw; qx; qy; qz];

% Create the rotation matrix from the quaternion
R = [1 - 2*qy^2 - 2*qz^2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw;
     2*qx*qy + 2*qz*qw, 1 - 2*qx^2 - 2*qz^2, 2*qy*qz - 2*qx*qw;
     2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx^2 - 2*qy^2];

bTo = [R p; 0 0 0 1];

gkTo = gkTb * bTo;
gkRo = gkTo(1:3,1:3);

trace_rotm = trace(gkRo);
qw_tmp = 0.5 * sqrt(1 + trace_rotm);
qx_tmp = (gkRo(3,2) - gkRo(2,3)) / (4 * qw_tmp + epsilon);
qy_tmp = (gkRo(1,3) - gkRo(3,1)) / (4 * qw_tmp + epsilon);
qz_tmp = (gkRo(2,1) - gkRo(1,2)) / (4 * qw_tmp + epsilon);

pose_from_T = [gkTo(1:3,4); qw_tmp; qx_tmp;qy_tmp;qz_tmp];

Jacobian_p = jacobian(pose_from_T(:), p);

Jacobian_Q = jacobian(pose_from_T(:), quaternion);


