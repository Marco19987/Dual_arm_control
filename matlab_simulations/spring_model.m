function [xdot, h]  = spring_model(x,u, K_1, K_2, B_1, B_2, Bm, bg,eul_choice,opg1,opg2,oRg1,oRg2)
    
    bpo = x(1:3);
    bRo = eul2rotm(x(4:6)',eul_choice);
    bp1 = x(13:15);
    bp2 = x(19:21);
    bpg1 = bpo + bRo * opg1;
    bpg2 = bpo + bRo * opg2;

    bpo_dot = x(7:9);
    bomegao = x(10:12);
    bp1_dot = u(1:3);
    bp2_dot = u(7:9);

    bpg1_dot = bpo_dot + skew(bomegao) * bRo * opg1;
    bpg2_dot = bpo_dot + skew(bomegao) * bRo * opg2;


    % wrench grasp frame 1 expressed in g1 : g1hg1
    g1Rb = oRg1' * bRo';
    g1fe_1 = K_1(1:3,1:3) * g1Rb * (- bpg1 + bp1); % elastic force 1 wrt g1
    g1fbeta_1 = B_1(1:3,1:3) * g1Rb * (- bpg1_dot + bp1_dot); % viscous force 1 wrt g1
    g1hg1 = [g1fe_1 + g1fbeta_1; [0 0 0]'];


    % wrench grasp frame 2 expressed in g2 : g2hg2
    g2Rb = oRg2' * bRo';
    g2fe_2 = K_2(1:3,1:3) * g2Rb * (- bpg2 + bp2); % elastic force 1 wrt g1
    g2fbeta_2 = B_2(1:3,1:3) * g1Rb * (- bpg2_dot + bp2_dot); % viscous force 1 wrt g1
    g2hg2 = [g2fe_2 + g2fbeta_2; [0 0 0]'];

    % compute grasp matrixs and Rbar matrix
    Wg1 = [eye(3), zeros(3); -skew(opg1)', eye(3)];
    Wg2 = [eye(3), zeros(3); -skew(opg2)', eye(3)];
    W = [Wg1,Wg2];

    Rbar = blkdiag(oRg1,oRg1,oRg2,oRg2);

    % object wrench
    oh = W * Rbar * [g1hg1;g2hg2];

    % xdot evaluation
    bxobj_dot_dot = blkdiag(bRo,bRo)*(Bm\oh) + [bg;0;0;0];
    Ta = rpy2jac(x(4:6)',eul_choice); % matrix relating omega_dot to the euler_angles_dot
    bxobj_dot = [x(7:9); Ta\x(10:12)];

    Ta_1 = rpy2jac(x(16:18)',eul_choice);
    bx1_dot = [bp1_dot; Ta_1\u(4:6)];

    Ta_2 = rpy2jac(x(22:24)',eul_choice);
    bx2_dot = [bp2_dot; Ta_2\u(10:12)];

    xdot = [bxobj_dot;bxobj_dot_dot;bx1_dot;bx2_dot];

    h = [g1hg1;g2hg2]; % wrenches in the grasp frames
    
end

