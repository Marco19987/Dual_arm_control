function [xdot, h]  = spring_model(x,u,bx1,bx2,K_1, K_2, B_1, B_2, Bm, bg,eul_choice,opg1,opg2,oRg1,oRg2)
    
    bpo = x(1:3);
    bRo = quat2rotm(x(4:7)');
    bp1 = bx1(1:3);
    bp2 = bx2(1:3);
    bpg1 = bpo + bRo * opg1;
    bpg2 = bpo + bRo * opg2;

    bpo_dot = x(8:10);
    bomegao = x(11:13);
    bp1_dot = u(1:3);
    bp2_dot = u(7:9);
    bomega1_dot = u(4:6);
    bomega2_dot = u(10:12);

    bpg1_dot = bpo_dot + skew(bomegao) * bRo * opg1;
    bpg2_dot = bpo_dot + skew(bomegao) * bRo * opg2;

    bR1 = quat2rotm(bx1(4:7)'); %robot 1 orientation
    bR2 = quat2rotm(bx2(4:7)'); %robot 2 orientation


    % wrench grasp frame 1 expressed in g1 : g1hg1
    g1Rb = oRg1' * bRo';
    g1fe_1 = K_1(1:3,1:3) * g1Rb * (- bpg1 + bp1); % elastic force 1 wrt g1
    g1fbeta_1 = B_1(1:3,1:3) * g1Rb * (- bpg1_dot + bp1_dot); % viscous force 1 wrt g1
    
    g1taue_1 = B_1(1:3,1:3) * rotm2eul(g1Rb*bR1,eul_choice)'; % elastic torsional force 1 wrt g1 - computed ad the euler angles of the matrix g1R1
    g1tau_beta_1 = B_1(4:6,4:6) * g1Rb * (- bomegao + bomega1_dot); % viscous torsional force 1 wrt g1

    g1hg1 = [g1fe_1 + g1fbeta_1; g1taue_1 + g1tau_beta_1 + [0 0 0]'];


    % wrench grasp frame 2 expressed in g2 : g2hg2
    g2Rb = oRg2' * bRo';
    g2fe_2 = K_2(1:3,1:3) * g2Rb * (- bpg2 + bp2); % elastic force 1 wrt g1
    g2fbeta_2 = B_2(1:3,1:3) * g2Rb * (- bpg2_dot + bp2_dot); % viscous force 1 wrt g1
    
    g2taue_2 = B_2(1:3,1:3) * rotm2eul(g2Rb*bR2,eul_choice)'; % elastic torsional force 2 wrt g2 - computed ad the euler angles of the matrix g2R2
    g2tau_beta_2 = B_2(4:6,4:6) * g2Rb * (- bomegao + bomega2_dot); % viscous torsional force 1 wrt g1

    g2hg2 = [g2fe_2 + g2fbeta_2; g2taue_2 + g2tau_beta_2 + [0 0 0]'];

    % compute grasp matrixs and Rbar matrix
    Wg1 = [eye(3), zeros(3); -skew(opg1)', eye(3)];
    Wg2 = [eye(3), zeros(3); -skew(opg2)', eye(3)];
    W = [Wg1,Wg2];

    Rbar = blkdiag(oRg1,oRg1,oRg2,oRg2);

    % object wrench
    oh = W * Rbar * [g1hg1;g2hg2];

    % xdot evaluation
    bxobj_dot_dot = blkdiag(bRo,bRo)*(Bm\oh) + [bg;0;0;0];
    bxobj_dot = [bpo_dot; Helper.quaternion_propagation(x(4:7),bomegao)];
 
    xdot = [bxobj_dot;bxobj_dot_dot];

    h = [g1hg1;g2hg2]; % wrenches in the grasp frames
    %h = [g1fe_1; [0 0 0]';g2fe_2;[0 0 0]' ];
end

