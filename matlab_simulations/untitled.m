opg1 = [0.06, -0.025, -0.0]';
opg2 = [-0.056, -0.005, 0.0]';
oRg1 = quat2rotm([0.0,-0.3827,0.9239, 0.0]); 
oRg2 = quat2rotm( [0.9239,0.0, 0.0, -0.3827]); 
oTg1 = [oRg1, opg1; 0 0 0 1];
oTg2 = [oRg2, opg2; 0 0 0 1];



% grasp matrix
Wg1 = [eye(3), zeros(3); -skew(opg1)', eye(3)];
Wg2 = [eye(3), zeros(3); -skew(opg2)', eye(3)];
W = [Wg1,Wg2];

% Rbar matrix 
Rbar = blkdiag(oRg1,oRg1,oRg2,oRg2);

g1hg1 = -[0,-0.2,0.6 0.005 0.002 0.0]';
g2hg2 = -[0,-0.22,-0.3 0.002 0.0 -0.01]';

h = [g1hg1;g2hg2];

he = W * Rbar * h;