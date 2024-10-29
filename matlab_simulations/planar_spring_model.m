function [xdot, bFk]  = planar_spring_model(x,u,udot, K1, K2, B1, B2, delta_p1, delta_p2, M, I, g)
    
    theta = x(3);
    bRo = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    bp1 = u(1:2); 
    bp2 = u(3:4); 
    bpo = x(1:2); % object position world frame
    op1 = bRo'*(bp1-bpo);
    op2 = bRo'*(bp2-bpo);
    opo = bRo'*(bpo-bpo); % should be always 0

    bpo_dot = x(4:5); % object vel in the base frame
    opo_dot = bRo' * bpo_dot;

    bp1_dot = udot(1:2);
    bp2_dot = udot(3:4);
    op1_dot = bRo' * bp1_dot;
    op2_dot = bRo' * bp2_dot;

    omega_r_1 = skew([0,0,x(6)])*[delta_p1;0];
    omega_r_2 =  skew([0,0,x(6)])*[delta_p2;0];


    oF1_b = B1*(-(opo_dot + omega_r_1(1:2)) + op1_dot); % viscous force
    oF2_b = B2*(-(opo_dot + omega_r_2(1:2)) + op2_dot);

    oF1 = K1*(op1 - opo - delta_p1); % elastic forces robot 1
    oF2 = K2*(op2 - opo - delta_p2);

    % angular acceleration
    delta_1_cross_F1 = cross([delta_p1;0], [oF1;0]);
    delta_2_cross_F2 = cross([delta_p2;0], [oF2;0]);
    delta_1_cross_F1_B = cross([delta_p1;0], [oF1_b;0]);
    delta_2_cross_F2_B = cross([delta_p2;0], [oF2_b;0]);


    bp_o_dot_dot = bRo*inv(M)*(oF1 + oF2) + bRo*inv(M)*(oF1_b + oF2_b) + g;
    btheta_o_dot_dot = inv(I)*(delta_1_cross_F1(3) + delta_2_cross_F2(3) + delta_1_cross_F1_B(3) + delta_2_cross_F2_B(3));

    xdot = [x(4:end);[bp_o_dot_dot;btheta_o_dot_dot]];

    bFk = [oF1;oF2]; % elastic forces in the base frame
    
end

