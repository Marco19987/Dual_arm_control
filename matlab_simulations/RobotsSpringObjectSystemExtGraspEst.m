classdef RobotsSpringObjectSystemExtGraspEst < SimpleSystem
    % Add oTg1 and oTg2 online estimation

    properties   
        base_system; % a RobotsSpringObjectSystemExt object

    end
    methods
        function obj = RobotsSpringObjectSystemExtGraspEst(initialState,system)
            % Call the constructor of the superclass
            obj@SimpleSystem(initialState, system.sizeState+14, system.sizeOutput);
            obj.base_system = system;            
        end

        function update_b2Tb1(obj, b2Tb1)
            obj.base_system.update_b2Tb1(b2Tb1);
        end 

        function xdot = eval_xdot(obj, x, u)
            obj.base_system.base_system.update_oTg1(Helper.transformation_matrix(x(35:37),x(38:41)));
            obj.base_system.base_system.update_oTg2(Helper.transformation_matrix(x(42:44),x(45:48)));
            xdot(1:34) = obj.base_system.eval_xdot(x(1:34),u);
            xdot(35:48) = zeros(14,1);
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
            obj.state(38:41) = obj.state(38:41)/norm(obj.state(38:41));
            obj.state(38:41) = Helper.quaternion_continuity(newState(38:41),x(38:41));
            obj.state(45:48) = obj.state(45:48)/norm(obj.state(45:48));
            obj.state(45:48) = Helper.quaternion_continuity(newState(45:48),x(45:48));
        end
        

         function jacobian = jacob_state_fcn(obj, x, u)
            obj.base_system.base_system.update_oTg1(Helper.transformation_matrix(x(35:37),x(38:41)));
            obj.base_system.base_system.update_oTg2(Helper.transformation_matrix(x(42:44),x(45:48)));
            jacobian = zeros(length(x));
            jacobian(1:34,1:34) = obj.base_system.jacob_state_fcn(x(1:34),u);
             % add jacobian WRh to oTg1 oTg2 to the twist term
            J_h_x = obj.get_jacobian_WRh_to_oTg1_oTg2(x,u);
            jacobian(8:13,35:48) = obj.base_system.base_system.Bm \ J_h_x;
            jacobian(35:48,35:48) = zeros(14);
        end

                
        function output = output_fcn(obj, x, u)
            obj.base_system.base_system.update_oTg1(Helper.transformation_matrix(x(35:37),x(38:41)));
            obj.base_system.base_system.update_oTg2(Helper.transformation_matrix(x(42:44),x(45:48)));
            output = obj.base_system.output_fcn(x(1:34),u);
        end

        function jacobian = jacob_output_fcn(obj, x, u)
            obj.base_system.base_system.update_oTg1(Helper.transformation_matrix(x(35:37),x(38:41)));
            obj.base_system.base_system.update_oTg2(Helper.transformation_matrix(x(42:44),x(45:48)));

         
            jacobian = zeros(obj.base_system.sizeOutput,length(x));

            jacobian(:,1:34) = obj.base_system.jacob_output_fcn(x(1:34),u);
        
       
            % jacobian wrench spring
            J_h_oTg1_oTg2 = obj.get_jacobian_h_to_oTg1_oTg2(x,u);
            jacobian(2*7*obj.base_system.base_system.n_pose_measures+1:2*7*obj.base_system.base_system.n_pose_measures+12,35:48) = J_h_oTg1_oTg2;

        end

       function jacobian = get_jacobian_h_to_oTg1_oTg2(obj,x,u)
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
            oTg1 = Helper.transformation_matrix(x(35:37),x(38:41));
            opg1 = oTg1(1:3,4);
            oQg1 = rotm2quat(oTg1(1:3,1:3))';
            oTg2 = Helper.transformation_matrix(x(42:44),x(45:48));
            opg2 = oTg2(1:3,4);
            oQg2 = rotm2quat(oTg2(1:3,1:3))';


            b1pe1_dot = u(1:3);  % robot 1 end-effector linear velocity in the robot 1 base frame
            b1_omega_e1 = u(4:6); % robot 1 end-effector angular velocity in the robot 1 base frame
            b2pe2_dot = u(7:9);  % robot 2 end-effector linear velocity in the robot 2 base frame
            b2_omega_e2 = u(10:12); % robot 2 end-effector angular velocity in the robot 2 base frame
            bpb1 = obj.base_system.base_system.bTb1(1:3,4); 
            bQb1 = rotm2quat(obj.base_system.base_system.bTb1(1:3,1:3))'; 
           
            K_1_diag = diag(obj.base_system.base_system.K_1);
            B_1_diag = diag(obj.base_system.base_system.B_1);
            K_2_diag = diag(obj.base_system.base_system.K_2);
            B_2_diag = diag(obj.base_system.base_system.B_2);

            jacobian = jacobian_h_to_oTg1_oTg2(bpo_b,bQo,ovo,oomegao,b1pe1_b1,b1Qe1,b2pe2_b2,b2Qe2,b2pb1,b2Qb1, ...
                                            b1pe1_dot,b1_omega_e1,b2pe2_dot,...
                                    b2_omega_e2,bpb1, bQb1, opg1,oQg1,  opg2,oQg2,K_1_diag,B_1_diag,K_2_diag,B_2_diag);
       end 

       function jacobian = get_jacobian_WRh_to_oTg1_oTg2(obj,x,u)
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
            oTg1 = Helper.transformation_matrix(x(35:37),x(38:41));
            opg1 = oTg1(1:3,4);
            oQg1 = rotm2quat(oTg1(1:3,1:3))';
            oTg2 = Helper.transformation_matrix(x(42:44),x(45:48));
            opg2 = oTg2(1:3,4);
            oQg2 = rotm2quat(oTg2(1:3,1:3))';

            b1pe1_dot = u(1:3);  % robot 1 end-effector linear velocity in the robot 1 base frame
            b1_omega_e1 = u(4:6); % robot 1 end-effector angular velocity in the robot 1 base frame
            b2pe2_dot = u(7:9);  % robot 2 end-effector linear velocity in the robot 2 base frame
            b2_omega_e2 = u(10:12); % robot 2 end-effector angular velocity in the robot 2 base frame
            bpb1 = obj.base_system.base_system.bTb1(1:3,4); 
            bQb1 = rotm2quat(obj.base_system.base_system.bTb1(1:3,1:3))'; 
            
            K_1_diag = diag(obj.base_system.base_system.K_1);
            B_1_diag = diag(obj.base_system.base_system.B_1);
            K_2_diag = diag(obj.base_system.base_system.K_2);
            B_2_diag = diag(obj.base_system.base_system.B_2);

            jacobian = jacobian_WRh_to_oTg1_oTg2(bpo_b,bQo,ovo,oomegao,b1pe1_b1,b1Qe1,b2pe2_b2,b2Qe2,b2pb1,b2Qb1, ...
                                            b1pe1_dot,b1_omega_e1,b2pe2_dot,...
                                    b2_omega_e2,bpb1, bQb1, opg1,oQg1,  opg2,oQg2,K_1_diag,B_1_diag,K_2_diag,B_2_diag);
        end


        

        function clonedSystem = clone(obj)
            clonedSystem = RobotsSpringObjectSystemExtGraspEst(obj.state, obj.base_system);
        end

    end
end
