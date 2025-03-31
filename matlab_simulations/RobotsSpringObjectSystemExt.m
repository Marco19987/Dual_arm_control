classdef RobotsSpringObjectSystemExt < SimpleSystem
    % Add b2Tb1 online estimation

    properties   
        base_system;

    end
    methods
        function obj = RobotsSpringObjectSystemExt(initialState,system)
            % Call the constructor of the superclass
            obj@SimpleSystem(initialState, system.sizeState+7, system.sizeOutput);
            obj.base_system = system;            
           
        end

        function update_b2Tb1(obj, b2Tb1)
            obj.base_system.b2Tb1 = b2Tb1;
        end 

        function xdot = eval_xdot(obj, x, u)
            xdot(1:27) = obj.base_system.eval_xdot(x(1:27),u);
            xdot(28:34) = zeros(7,1);
        end

        function newState = state_fcn(obj, x, u)
            newState = obj.eval_xdot(x, u)';
        end


        function updateState(obj,state)
            obj.state = state;
            obj.state(4:7) = obj.state(4:7)/norm(obj.state(4:7));   
            obj.state(4:7) = Helper.quaternion_continuity(newState(4:7),x(4:7));
            obj.state(17:20) = obj.state(17:20)/norm(obj.state(17:20));
            obj.state(17:20) = Helper.quaternion_continuity(newState(17:20),x(17:20));
            obj.state(24:27) = obj.state(24:27)/norm(obj.state(24:27));
            obj.state(24:27) = Helper.quaternion_continuity(newState(24:27),x(24:27));
            obj.state(31:34) = obj.state(31:34)/norm(obj.state(31:34));
            obj.state(31:34) = Helper.quaternion_continuity(newState(31:34),x(31:34));
        end
        

         function jacobian = jacob_state_fcn(obj, x, u)
            jacobian = zeros(length(x));
            jacobian(1:27,1:27) = obj.base_system.jacob_state_fcn(x(1:27),u);
             % add jacobian h to b2Tb1 to the twist term
            J_h_x = obj.get_jacobian_h_to_b2Tb1(x,u);
            J_h_x = obj.base_system.Bm \  (obj.base_system.W * obj.base_system.Rbar * J_h_x);
            jacobian(8:13,28:34) = J_h_x;
            jacobian(28:end,28:end) = zeros(7);
        end

                
        function output = output_fcn(obj, x, u)
           x(31:34) = Helper.normalize_quaternion(x(31:34));

           b2Tb1 = obj.base_system.b2Tb1; %store b2Tb1 state
           obj.update_b2Tb1(Helper.transformation_matrix(x(28:30), x(31:34)));
           
           output = obj.base_system.output_fcn(x(1:27),u);

           obj.update_b2Tb1(b2Tb1); % reset to the original value - dirty solution 
        
        end

        function jacobian = jacob_output_fcn(obj, x, u)
            b2Tb1 = obj.base_system.b2Tb1; %store b2Tb1 state
            obj.update_b2Tb1(Helper.transformation_matrix(x(28:30), x(31:34)));

            x(4:7) = Helper.normalize_quaternion(x(4:7));
            x(31:34) = Helper.normalize_quaternion(x(31:34));

            bQo = x(4:7);
            b2Qb1 = x(31:34);
            bQb1 = rotm2quat((obj.base_system.bTb1(1:3,1:3)))';
            b1Qb = Helper.quaternion_inverse(bQb1)';
            

            bTo = Helper.transformation_matrix(x(1:3), bQo);
            bTb1 = obj.base_system.bTb1;

   
            jacobian = zeros(obj.base_system.sizeOutput,length(x));

            jacobian(:,1:27) = obj.base_system.jacob_output_fcn(x(1:27),u);

            
            
            % jacobian measures from robot 2
            b1To = bTb1 \ bTo;
            b1Qo = Helper.quaternion_product(b1Qb,bQo)';

            J_b2po_b2pb1 = eye(3); 
            J_b2po_b2Qb1 = obj.Jacobian_Rp_to_Q(b2Qb1,b1To(1:3,4));
            J_b2Qo_b2Qb1 = obj.base_system.Jacobian_quaternion_product_left(b1Qo);

            output_jacobian_2_kth = [J_b2po_b2pb1 J_b2po_b2Qb1; zeros(4,3) J_b2Qo_b2Qb1];
            jacobian(7*obj.base_system.n_pose_measures+1:2*7*obj.base_system.n_pose_measures,28:end) = repmat(output_jacobian_2_kth,obj.base_system.n_pose_measures,1);


            % jacobian wrench spring
            J_h_b2Tb1 = obj.get_jacobian_h_to_b2Tb1(x,u);
            jacobian(2*7*obj.base_system.n_pose_measures+1:2*7*obj.base_system.n_pose_measures+12,28:end) = J_h_b2Tb1;

            obj.update_b2Tb1(b2Tb1); % reset to the original value - dirty solution 

        end

        function Jacobian_bpo_dot_to_Q = Jacobian_Rp_to_Q(obj,Q,p)
            % Jacobian of the function R(Q)*p, with R(Q) being the 
            % rotation matrix corresponding to the quaternion Q and p being a 
            % 3D vector,
            % with respect to the quaternion Q
            
            Q1 = Q(1,:);
            Q2 = Q(2,:);
            Q3 = Q(3,:);
            Q4 = Q(4,:);
            p1 = p(1,:);
            p2 = p(2,:);
            p3 = p(3,:);
            t2 = Q1.*p1.*2.0;
            t3 = Q1.*p2.*2.0;
            t4 = Q2.*p1.*2.0;
            t5 = Q1.*p3.*2.0;
            t6 = Q2.*p2.*2.0;
            t7 = Q3.*p1.*2.0;
            t8 = Q2.*p3.*2.0;
            t9 = Q3.*p2.*2.0;
            t10 = Q4.*p1.*2.0;
            t11 = Q3.*p3.*2.0;
            t12 = Q4.*p2.*2.0;
            t13 = Q4.*p3.*2.0;
            Jacobian_bpo_dot_to_Q = reshape([t11-t12,-t8+t10,t6-t7,t9+t13,-t5+t7-Q2.*p2.*4.0,t3+t10-Q2.*p3.*4.0,t5+t6-Q3.*p1.*4.0,t4+t13,-t2+t12-Q3.*p3.*4.0,-t3+t8-Q4.*p1.*4.0,t2+t11-Q4.*p2.*4.0,t4+t9],[3,4]);
        end


        function jacobian = get_jacobian_h_to_b2Tb1(obj,x,u)
            bpo_b = x(1:3);       % object's position in the base frame 
            bQo = x(4:7);       % object's quaternion in the base frame  
            ovo = x(8:10);      % object's linear velocity in the object frame
            oomegao = x(11:13); % object's angular velocity in the object frame
            b1pe1_b1 = x(14:16); % robot 1 end-effector position in the robot 1 base frame
            b1Qe1 = x(17:20);    % robot 1 end-effector quaternion in the robot 1 base frame
            b2pe2_b2 = x(21:23); % robot 2 end-effector position in the robot 2 base frame
            b2Qe2 = x(24:27);    % robot 2 end-effector quaternion in the robot 2 base frame
            b2pb1 = x(28:30);
            b2Qb1 = x(31:34);

            b1pe1_dot = u(1:3);  % robot 1 end-effector linear velocity in the robot 1 base frame
            b1_omega_e1 = u(4:6); % robot 1 end-effector angular velocity in the robot 1 base frame
            b2pe2_dot = u(7:9);  % robot 2 end-effector linear velocity in the robot 2 base frame
            b2_omega_e2 = u(10:12); % robot 2 end-effector angular velocity in the robot 2 base frame
            bpb1 = obj.base_system.bTb1(1:3,4); 
            bQb1 = rotm2quat(obj.base_system.bTb1(1:3,1:3))'; 
            opg1 = obj.base_system.oTg1(1:3,4);
            oQg1 = rotm2quat(obj.base_system.oTg1(1:3,1:3))';
            opg2 = obj.base_system.oTg2(1:3,4);
            oQg2 = rotm2quat(obj.base_system.oTg2(1:3,1:3))';
            K_1_diag = diag(obj.base_system.K_1);
            B_1_diag = diag(obj.base_system.B_1);
            K_2_diag = diag(obj.base_system.K_2);
            B_2_diag = diag(obj.base_system.B_2);

            jacobian = jacobian_h_to_b2Tb1(bpo_b,bQo,ovo,oomegao,b1pe1_b1,b1Qe1,b2pe2_b2,b2Qe2,b2pb1,b2Qb1, ...
                                            b1pe1_dot,b1_omega_e1,b2pe2_dot,...
                                    b2_omega_e2,bpb1, bQb1, opg1,oQg1,  opg2,oQg2,K_1_diag,B_1_diag,K_2_diag,B_2_diag);
        end 


        

        function clonedSystem = clone(obj)
            clonedSystem = RobotsSpringObjectSystemExt(obj.state, obj.base_system);
        end

    end
end
