classdef RobotsObjectSystem < SimpleSystem
    % system describing the interaction between 2 robots and one object
    % the input to the system are the exerted wrenches by the robots
    % applied in the grasp frames and expressed in the same frame
    % the output of the system are the measured poses of the object in the 
    % robots base frame
    % the state is represented by the pose of the object in the inertial
    % frame an the twist
    % pose = [px py pz qw qx qy qz]' epressed in the base frame;
    % twist = [vx vy vz omegax omegay omegaz]' expressed in the object frame;

    properties      

        Bm     % object inertia matrix
        bg     % gravity vector in the base frame 
        oTg1   % transformation matrix of the grasp frame of robot1 expressed in the object frame
        oTg2   % transformation matrix of the grasp frame of robot2 expressed in the object frame

        n_pose_measures      % number pose estimators 
                             % => there will be 2*n_pose_measures measures of the 
                             % object pose (for each robot we have n_pose_measures)
                             % the poses measured by robot_i are expressed
                             % in the grasp_i frame for i=1,2
                            
        W      % grasp matrix
        Rbar   % rotation matrixs between grasp frames and object frame

        b1Tb2   % robot 2 base frame transformation matrix wrt the robot 1 base frame
        % This transformation matrix can be added as state variable of the filter for 
        % a further refinement

        bTb1   % transformation matrix of robot 1 wrt the base frame into wich express the object pose and twist
        % assumed to be constant and well known

        viscous_friction; % 6x6 matrix of the viscous friction coefficients for the object-air friction

    end
    methods
        function obj = RobotsObjectSystem(state, sizeState, sizeOutput,SampleTime, Bm,bg,oTg1,oTg2,n_pose_measures ...
                                            ,b1Tb2,bTb1,viscous_friction)
            % Call the constructor of the superclass
            obj@SimpleSystem(state, sizeState, sizeOutput, SampleTime);
            
            obj.Bm = Bm;
            obj.bg = bg;
            obj.oTg1 = oTg1;
            obj.oTg2 = oTg2;
            obj.n_pose_measures = n_pose_measures;
            obj.update_grasp_matrix()
            obj.update_Rbar()

            obj.b1Tb2 = b1Tb2;

            obj.bTb1 = bTb1;

            obj.viscous_friction = viscous_friction;
        end

        function update_b1Tb2(obj, b1Tb2)
            obj.b1Tb2 = b1Tb2;
        end 
      
        function update_grasp_matrix(obj)
            % compute grasp matrix 
            opg1 = obj.oTg1(1:3,4);
            opg2 = obj.oTg2(1:3,4);
            Wg1 = [eye(3), zeros(3); -skew(opg1)', eye(3)];
            Wg2 = [eye(3), zeros(3); -skew(opg2)', eye(3)];
            obj.W = [Wg1,Wg2];
        end 

        function update_Rbar(obj)
            oRg1 = obj.oTg1(1:3,1:3);
            oRg2 = obj.oTg2(1:3,1:3);
            obj.Rbar = blkdiag(oRg1,oRg1,oRg2,oRg2);
        end

        function xdot = eval_xdot(obj, x, u)
            x(4:7) = x(4:7)/norm(x(4:7));
            % implementation of xdot = f(x,u)
            ovo = x(8:10);      % object's linear velocity in the object frame
            oomegao = x(11:13); % object's angular velocity in the object frame
            bQo = x(4:7);       % object's quaternion in the base frame
            g1hg1 = u(1:6);     % robot1's grasp wrench in the grasp frame 1
            g2hg2 = u(7:12);    % robot2's grasp wrench in the grasp frame 2
            oh = obj.W * obj.Rbar * [g1hg1;g2hg2];  % resulting wrench in the object frame
            bRo = quat2rotm(bQo');
            bRo_bar = blkdiag(bRo,bRo);
            o_twist_o = [ovo;oomegao]; % object twist base frame
            double_skew = [skew(oomegao), zeros(3); skew(ovo),skew(oomegao)];

            o_twist_dot_o = obj.Bm\(oh + obj.Bm*bRo_bar'*[obj.bg;0;0;0] - obj.viscous_friction*o_twist_o ...
                - double_skew*obj.Bm*o_twist_o);

            xdot(1:3) = bRo*ovo;
            xdot(4:7) = Helper.quaternion_propagation(bQo,bRo*oomegao);        

            xdot(8:13) = o_twist_dot_o;

        end
        
        function newState = state_fcn(obj, x, u)
            % discretized version
            % xk+1 = xk + SampleTime * xdotk
            newState = x + obj.SampleTime * obj.eval_xdot(x, u)';
            newState(4:7) = newState(4:7)/norm(newState(4:7));   
            newState(4:7) = Helper.quaternion_continuity(newState(4:7),x(4:7));
        end
        
        function output = output_fcn(obj, x, u)
           % in the output there are the measures coming from the object pose
           % estimators expressed in the respective robot's base frame: bkTo
           % there is only a transformation involved in the relation 
           x(4:7) = x(4:7)/norm(x(4:7));
            
       
           output = zeros(7*obj.n_pose_measures*2,1); 
          
           bTo = Helper.transformation_matrix(x(1:3), x(4:7));
            
           b1To = obj.bTb1 \ bTo; % measured object pose from robot 1 with the k-th pose estimator in the base1 frame
           b2To = inv(obj.b1Tb2) * inv(obj.bTb1) * bTo; % measured object pose from robot 2 with the k-th pose estimator in the base2 frame
            
           b1Qo = Helper.quaternion_product(rotm2quat(obj.bTb1(1:3,1:3)'),x(4:7));
           b2Qo = Helper.quaternion_product(rotm2quat(obj.b1Tb2(1:3,1:3)' * obj.bTb1(1:3,1:3)'),x(4:7));

         
           b1qo = [b1To(1:3,4);b1Qo'];
           b2qo = [b2To(1:3,4);b2Qo'];
            
           output(1:7*obj.n_pose_measures) = repmat(b1qo,obj.n_pose_measures,1);
           output(7*obj.n_pose_measures+1:end) = repmat(b2qo,obj.n_pose_measures,1);

            
        end
        
        function jacobian = jacob_state_fcn(obj, x, u)
            x(4:7) = x(4:7)/norm(x(4:7));
            % Override the Jacobian of the state transition function
            bQo = x(4:7);
            ovo = x(8:10);  
            oomegao = x(11:13);
            bRo = quat2rotm(bQo');
            otwisto = [ovo;oomegao];

            jacobian = zeros(length(x));
           
            % linear velocity term
            jacobian(1:3,8:10) = bRo; 
            jacobian(1:3,4:7) = obj.Jacobian_bpo_dot_to_bQo(bQo,ovo); 
            
            % quaternion dot term
            jacobian(4:7,4:7) = obj.Jacobian_bQo_dot_to_bQo(oomegao); %  jacobian of 0.5*E[omega]*Q wrt Q
            jacobian(4:7,11:13) = obj.Jacobian_bQo_dot_to_oomegao(bQo);      % jacobian of 0.5*E[omega]*Q wrt omega

            % acceleration term 
            jacobian(8:13,4:7) = obj.Jacobian_otwisto_dot_to_bQo(bQo,obj.bg);
            jacobian(8:13,8:13) = obj.Bm\obj.Jacobian_otwisto_dot_to_otwisto_skew_term(obj.Bm,otwisto);

            % viscous force term depending from otwisto
            jacobian(8:13,8:13) = -obj.Bm\obj.viscous_friction;
            
            jacobian = eye(length(x)) + obj.SampleTime*jacobian;

        end

        function Jacobian_bpo_dot_to_bQo = Jacobian_bpo_dot_to_bQo(obj,bQo,ovo)
            %Jacobian_bpo_dot_to_bQo
            %    Jacobian_bpo_dot_to_bQo = Jacobian_bpo_dot_to_bQo(IN1,IN2)
            
            %    This function was generated by the Symbolic Math Toolbox version 23.2.
            %    17-Mar-2025 15:14:01
            
            bQo1 = bQo(1,:);
            bQo2 = bQo(2,:);
            bQo3 = bQo(3,:);
            bQo4 = bQo(4,:);
            ovo1 = ovo(1,:);
            ovo2 = ovo(2,:);
            ovo3 = ovo(3,:);
            t2 = bQo1.*ovo1.*2.0;
            t3 = bQo1.*ovo2.*2.0;
            t4 = bQo2.*ovo1.*2.0;
            t5 = bQo1.*ovo3.*2.0;
            t6 = bQo2.*ovo2.*2.0;
            t7 = bQo3.*ovo1.*2.0;
            t8 = bQo2.*ovo3.*2.0;
            t9 = bQo3.*ovo2.*2.0;
            t10 = bQo4.*ovo1.*2.0;
            t11 = bQo3.*ovo3.*2.0;
            t12 = bQo4.*ovo2.*2.0;
            t13 = bQo4.*ovo3.*2.0;
            Jacobian_bpo_dot_to_bQo = reshape([t11-t12,-t8+t10,t6-t7,t9+t13,-t5+t7-bQo2.*ovo2.*4.0,t3+t10-bQo2.*ovo3.*4.0,t5+t6-bQo3.*ovo1.*4.0,t4+t13,-t2+t12-bQo3.*ovo3.*4.0,-t3+t8-bQo4.*ovo1.*4.0,t2+t11-bQo4.*ovo2.*4.0,t4+t9],[3,4]);
        end

        function Jacobian_bQo_dot_to_bQo = Jacobian_bQo_dot_to_bQo(obj,oomegao)
            %Jacobian_bQo_dot_to_bQo
            %    Jacobian_bQo_dot_to_bQo = Jacobian_bQo_dot_to_bQo(IN1)
            
            %    This function was generated by the Symbolic Math Toolbox version 23.2.
            %    17-Mar-2025 15:31:44
            
            oomegao1 = oomegao(1,:);
            oomegao2 = oomegao(2,:);
            oomegao3 = oomegao(3,:);
            t2 = oomegao1./2.0;
            t3 = oomegao2./2.0;
            t4 = oomegao3./2.0;
            t5 = -t2;
            t6 = -t3;
            t7 = -t4;
            Jacobian_bQo_dot_to_bQo = reshape([0.0,t2,t3,t4,t5,0.0,t7,t3,t6,t4,0.0,t5,t7,t6,t2,0.0],[4,4]);
        end

        function Jacobian_bQo_dot_to_oomegao = Jacobian_bQo_dot_to_oomegao(obj,bQo)
            %Jacobian_bQo_dot_to_oomegao
            %    Jacobian_bQo_dot_to_oomegao = Jacobian_bQo_dot_to_oomegao(IN1)
            
            %    This function was generated by the Symbolic Math Toolbox version 23.2.
            %    17-Mar-2025 15:35:09
            
            bQo1 = bQo(1,:);
            bQo2 = bQo(2,:);
            bQo3 = bQo(3,:);
            bQo4 = bQo(4,:);
            t2 = bQo1./2.0;
            t3 = bQo2./2.0;
            t4 = bQo3./2.0;
            t5 = bQo4./2.0;
            t6 = -t3;
            t7 = -t4;
            t8 = -t5;
            Jacobian_bQo_dot_to_oomegao = reshape([t6,t2,t5,t7,t7,t8,t2,t3,t8,t4,t6,t2],[4,3]);
        end

        function Jacobian_otwisto_dot_to_bQo = Jacobian_otwisto_dot_to_bQo(obj,bQo,bg)
            %Jacobian_otwisto_dot_to_bQo
            %    Jacobian_otwisto_dot_to_bQo = Jacobian_otwisto_dot_to_bQo(IN1,IN2)
            
            %    This function was generated by the Symbolic Math Toolbox version 23.2.
            %    17-Mar-2025 16:01:45
            
            bQo1 = bQo(1,:);
            bQo2 = bQo(2,:);
            bQo3 = bQo(3,:);
            bQo4 = bQo(4,:);
            bg1 = bg(1,:);
            bg2 = bg(2,:);
            bg3 = bg(3,:);
            t2 = bQo1.*bg1.*2.0;
            t3 = bQo1.*bg2.*2.0;
            t4 = bQo2.*bg1.*2.0;
            t5 = bQo1.*bg3.*2.0;
            t6 = bQo2.*bg2.*2.0;
            t7 = bQo3.*bg1.*2.0;
            t8 = bQo2.*bg3.*2.0;
            t9 = bQo3.*bg2.*2.0;
            t10 = bQo4.*bg1.*2.0;
            t11 = bQo3.*bg3.*2.0;
            t12 = bQo4.*bg2.*2.0;
            t13 = bQo4.*bg3.*2.0;
            Jacobian_otwisto_dot_to_bQo = reshape([-t11+t12,t8-t10,-t6+t7,0.0,0.0,0.0,t9+t13,t5+t7-bQo2.*bg2.*4.0,-t3+t10-bQo2.*bg3.*4.0,0.0,0.0,0.0,-t5+t6-bQo3.*bg1.*4.0,t4+t13,t2+t12-bQo3.*bg3.*4.0,0.0,0.0,0.0,t3+t8-bQo4.*bg1.*4.0,-t2+t11-bQo4.*bg2.*4.0,t4+t9,0.0,0.0,0.0],[6,4]);
        end

        function Jacobian_otwisto_dot_to_otwisto_skew_term = Jacobian_otwisto_dot_to_otwisto_skew_term(obj,Bm,otwisto)
            %Jacobian_otwisto_dot_to_otwisto_skew_term
            %    Jacobian_otwisto_dot_to_otwisto_skew_term = Jacobian_otwisto_dot_to_otwisto_skew_term(IN1,IN2)
            
            %    This function was generated by the Symbolic Math Toolbox version 23.2.
            %    17-Mar-2025 16:12:48
            
            Bm1_1 = Bm(1);
            Bm1_2 = Bm(7);
            Bm1_3 = Bm(13);
            Bm1_4 = Bm(19);
            Bm1_5 = Bm(25);
            Bm1_6 = Bm(31);
            Bm2_1 = Bm(2);
            Bm2_2 = Bm(8);
            Bm2_3 = Bm(14);
            Bm2_4 = Bm(20);
            Bm2_5 = Bm(26);
            Bm2_6 = Bm(32);
            Bm3_1 = Bm(3);
            Bm3_2 = Bm(9);
            Bm3_3 = Bm(15);
            Bm3_4 = Bm(21);
            Bm3_5 = Bm(27);
            Bm3_6 = Bm(33);
            Bm4_1 = Bm(4);
            Bm4_2 = Bm(10);
            Bm4_3 = Bm(16);
            Bm4_4 = Bm(22);
            Bm4_5 = Bm(28);
            Bm4_6 = Bm(34);
            Bm5_1 = Bm(5);
            Bm5_2 = Bm(11);
            Bm5_3 = Bm(17);
            Bm5_4 = Bm(23);
            Bm5_5 = Bm(29);
            Bm5_6 = Bm(35);
            Bm6_1 = Bm(6);
            Bm6_2 = Bm(12);
            Bm6_3 = Bm(18);
            Bm6_4 = Bm(24);
            Bm6_5 = Bm(30);
            Bm6_6 = Bm(36);
            oomegao1 = otwisto(4,:);
            oomegao2 = otwisto(5,:);
            oomegao3 = otwisto(6,:);
            ovo1 = otwisto(1,:);
            ovo2 = otwisto(2,:);
            ovo3 = otwisto(3,:);
            t2 = Bm1_4.*oomegao1;
            t3 = Bm1_5.*oomegao2;
            t4 = Bm1_6.*oomegao3;
            t5 = Bm2_4.*oomegao1;
            t6 = Bm2_5.*oomegao2;
            t7 = Bm2_6.*oomegao3;
            t8 = Bm3_4.*oomegao1;
            t9 = Bm3_5.*oomegao2;
            t10 = Bm3_6.*oomegao3;
            t11 = Bm4_4.*oomegao1;
            t12 = Bm5_5.*oomegao2;
            t13 = Bm6_6.*oomegao3;
            t14 = Bm1_1.*ovo1;
            t15 = Bm1_2.*ovo2;
            t16 = Bm1_3.*ovo3;
            t17 = Bm2_1.*ovo1;
            t18 = Bm2_2.*ovo2;
            t19 = Bm2_3.*ovo3;
            t20 = Bm3_1.*ovo1;
            t21 = Bm3_2.*ovo2;
            t22 = Bm3_3.*ovo3;
            t23 = Bm4_1.*ovo1;
            t24 = Bm4_2.*ovo2;
            t25 = Bm4_3.*ovo3;
            t26 = Bm5_1.*ovo1;
            t27 = Bm5_2.*ovo2;
            t28 = Bm5_3.*ovo3;
            t29 = Bm6_1.*ovo1;
            t30 = Bm6_2.*ovo2;
            t31 = Bm6_3.*ovo3;
            t32 = -t2;
            t33 = -t3;
            t34 = -t6;
            t35 = -t7;
            t36 = -t8;
            t37 = -t10;
            t38 = -t14;
            t39 = -t15;
            t40 = -t18;
            t41 = -t19;
            t42 = -t20;
            t43 = -t22;
            mt1 = [Bm2_1.*oomegao3-Bm3_1.*oomegao2,-Bm1_1.*oomegao3+Bm3_1.*oomegao1,Bm1_1.*oomegao2-Bm2_1.*oomegao1,Bm5_1.*oomegao3-Bm6_1.*oomegao2+Bm2_1.*ovo3-Bm3_1.*ovo2,t8+t9+t10+t20.*2.0+t21+t22-Bm4_1.*oomegao3+Bm6_1.*oomegao1-Bm1_1.*ovo3,-t5-t17.*2.0+t34+t35+t40+t41+Bm4_1.*oomegao2-Bm5_1.*oomegao1+Bm1_1.*ovo2,Bm2_2.*oomegao3-Bm3_2.*oomegao2,-Bm1_2.*oomegao3+Bm3_2.*oomegao1,Bm1_2.*oomegao2-Bm2_2.*oomegao1,-t9-t21.*2.0+t36+t37+t42+t43+Bm5_2.*oomegao3-Bm6_2.*oomegao2+Bm2_2.*ovo3,-Bm4_2.*oomegao3+Bm6_2.*oomegao1-Bm1_2.*ovo3+Bm3_2.*ovo1,t2+t3+t4+t14+t15.*2.0+t16+Bm4_2.*oomegao2-Bm5_2.*oomegao1-Bm2_2.*ovo1,Bm2_3.*oomegao3-Bm3_3.*oomegao2,-Bm1_3.*oomegao3+Bm3_3.*oomegao1];
            mt2 = [Bm1_3.*oomegao2-Bm2_3.*oomegao1,t5+t6+t7+t17+t18+t19.*2.0+Bm5_3.*oomegao3-Bm6_3.*oomegao2-Bm3_3.*ovo2,-t4-t16.*2.0+t32+t33+t38+t39-Bm4_3.*oomegao3+Bm6_3.*oomegao1+Bm3_3.*ovo1,Bm4_3.*oomegao2-Bm5_3.*oomegao1+Bm1_3.*ovo2-Bm2_3.*ovo1,Bm2_4.*oomegao3-Bm3_4.*oomegao2,t8.*2.0+t9+t10+t20+t21+t22-Bm1_4.*oomegao3,t5.*-2.0-t17+t34+t35+t40+t41+Bm1_4.*oomegao2,Bm5_4.*oomegao3-Bm6_4.*oomegao2+Bm2_4.*ovo3-Bm3_4.*ovo2,t13+t29+t30+t31-Bm4_4.*oomegao3+Bm6_4.*oomegao1.*2.0+Bm6_5.*oomegao2-Bm1_4.*ovo3+Bm3_4.*ovo1,-t12-t26-t27-t28+Bm4_4.*oomegao2-Bm5_4.*oomegao1.*2.0-Bm5_6.*oomegao3+Bm1_4.*ovo2-Bm2_4.*ovo1];
            mt3 = [t9.*-2.0-t21+t36+t37+t42+t43+Bm2_5.*oomegao3,-Bm1_5.*oomegao3+Bm3_5.*oomegao1,t2+t3.*2.0+t4+t14+t15+t16-Bm2_5.*oomegao1,-t13-t29-t30-t31+Bm5_5.*oomegao3-Bm6_4.*oomegao1-Bm6_5.*oomegao2.*2.0+Bm2_5.*ovo3-Bm3_5.*ovo2,-Bm4_5.*oomegao3+Bm6_5.*oomegao1-Bm1_5.*ovo3+Bm3_5.*ovo1,t11+t23+t24+t25+Bm4_5.*oomegao2.*2.0+Bm4_6.*oomegao3-Bm5_5.*oomegao1+Bm1_5.*ovo2-Bm2_5.*ovo1,t5+t6+t7.*2.0+t17+t18+t19-Bm3_6.*oomegao2,t4.*-2.0-t16+t32+t33+t38+t39+Bm3_6.*oomegao1,Bm1_6.*oomegao2-Bm2_6.*oomegao1,t12+t26+t27+t28+Bm5_4.*oomegao1+Bm5_6.*oomegao3.*2.0-Bm6_6.*oomegao2+Bm2_6.*ovo3-Bm3_6.*ovo2];
            mt4 = [-t11-t23-t24-t25-Bm4_5.*oomegao2-Bm4_6.*oomegao3.*2.0+Bm6_6.*oomegao1-Bm1_6.*ovo3+Bm3_6.*ovo1,Bm4_6.*oomegao2-Bm5_6.*oomegao1+Bm1_6.*ovo2-Bm2_6.*ovo1];
            Jacobian_otwisto_dot_to_otwisto_skew_term = reshape([mt1,mt2,mt3,mt4],6,6);
        end

        
        
        
        
        function jacobian = jacob_output_fcn(obj, x, u)
            % Override the Jacobian of the output function

            jacobian = zeros(7*2*obj.n_pose_measures,length(x));

            % jacobian measures from robot 1
            b1Tb = inv(obj.bTb1);
            b1Qb = rotm2quat(b1Tb(1:3,1:3))';
            
            Jp_1 = obj.jacobian_output_to_position(b1Tb);   %output position jacobian robot 1
            JQ_1 = zeros(7,4); 
            JQ_1(4:7,1:4) = obj.Jacobian_quaternion_product_right(b1Qb);
            output_jacobian_1_kth = [Jp_1, JQ_1, zeros(7,3), zeros(7,3)];
            jacobian(1:7*obj.n_pose_measures,:) = repmat(output_jacobian_1_kth,obj.n_pose_measures,1);
            
            % jacobian measures from robot 1
            b2Tb = inv(obj.b1Tb2) * inv(obj.bTb1);
            b2Qb = rotm2quat(b2Tb(1:3,1:3))';

            Jp_2 = obj.jacobian_output_to_position(b2Tb);   
            JQ_2 = zeros(7,4); 
            JQ_2(4:7,1:4) = obj.Jacobian_quaternion_product_right(b2Qb);
            output_jacobian_2_kth = [Jp_2, JQ_2, zeros(7,3), zeros(7,3)];
            jacobian(7*obj.n_pose_measures+1:end,:) = repmat(output_jacobian_2_kth,obj.n_pose_measures,1);

        end



        function jacobian = jacobian_output_to_position(obj,T)
            % depends only from the rotation matrix between the base frame
            % and the k-th base frame 
            jacobian = [T(1:3,1:3);zeros(4,3)];
        end 

        function Jacobian_quaternion_product_right = Jacobian_quaternion_product_right(obj,Qleft)
            %Jacobian_quaternion_product_right
            %    Jacobian_quaternion_product_right = Jacobian_quaternion_product_right(IN1)
            
            %    This function was generated by the Symbolic Math Toolbox version 23.2.
            %    17-Mar-2025 17:00:19
            
            Q11 = Qleft(1,:);
            Q12 = Qleft(2,:);
            Q13 = Qleft(3,:);
            Q14 = Qleft(4,:);
            t2 = -Q12;
            t3 = -Q13;
            t4 = -Q14;
            Jacobian_quaternion_product_right = reshape([Q11,Q12,Q13,Q14,t2,Q11,Q14,t3,t3,t4,Q11,Q12,t4,Q13,t2,Q11],[4,4]);
        end

        function clonedSystem = clone(obj)
            clonedSystem = RobotsObjectSystem(obj.state, obj.sizeState, obj.sizeOutput,obj.SampleTime, obj.Bm,obj.bg,obj.oTg1,obj.oTg2,obj.n_pose_measures ...
                                            ,obj.b1Tb2,obj.bTb1,obj.viscous_friction);
        end

    end
end
