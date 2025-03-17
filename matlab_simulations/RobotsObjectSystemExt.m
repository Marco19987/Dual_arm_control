classdef RobotsObjectSystemExt < SimpleSystem
    % Add b1Tb2 online estimation

    properties   
        base_system;

    end
    methods
        function obj = RobotsObjectSystemExt(initialState,system)
            % Call the constructor of the superclass
            % Call the constructor of the superclass
            obj@SimpleSystem(initialState, system.sizeState+7, system.sizeOutput, system.SampleTime);
            obj.base_system = system;            
           
        end

        function xdot = eval_xdot(obj, x, u)
            xdot(1:13) = obj.base_system.eval_xdot(x(1:13),u);
            xdot(14:20) = zeros(7,1);
        end

        function newState = state_fcn(obj, x, u)
            % discretized version
            % xk+1 = xk + SampleTime * xdotk
            newState = x + obj.SampleTime * obj.eval_xdot(x, u)';
            newState(4:7) = newState(4:7)/norm(newState(4:7));   
            newState(4:7) = Helper.quaternion_continuity(newState(4:7),x(4:7));
            newState(17:20) = newState(17:20)/norm(newState(17:20));
            newState(17:20) = Helper.quaternion_continuity(newState(17:20),x(17:20));
            obj.base_system.b1Tb2 = Helper.transformation_matrix(x(14:16), x(17:20)); % update estimated b1Tb2 in the base system 
        end

         function jacobian = jacob_state_fcn(obj, x, u)
            jacobian = zeros(length(x));
            jacobian(1:13,1:13) = obj.base_system.jacob_state_fcn(x,u);
        end

                
        function output = output_fcn(obj, x, u)
           x(4:7) = Helper.normalize_quaternion(x(4:7));
           x(17:20) = Helper.normalize_quaternion(x(17:20));
           
           output = obj.base_system.output_fcn(x(1:13),u);
        
        end

        function jacobian = jacob_output_fcn(obj, x, u)
            x(4:7) = x(4:7)/norm(x(4:7));
            x(17:20) = x(17:20)/norm(x(17:20));


            bTo = Helper.transformation_matrix(x(1:3), x(4:7));

            b1Qb2 = x(17:20);
            obj.b1Tb2 = Helper.transformation_matrix(x(14:16), b1Qb2);
            bQo = x(4:7);
            
            jacobian = zeros(7*2*obj.n_pose_measures,length(x));

            % jacobian measures from robot 1
            b1Tb = inv(obj.bTb1);
            b1Qb = rotm2quat(b1Tb(1:3,1:3)); 
            Jp_1 = obj.jacobian_output_to_position(b1Tb);   %output position jacobian robot 1
            JQ_1 = obj.jacobian_output_to_quaternion_right(b1Qb);  %output quaternion jacobian robot 1
            output_jacobian_1_kth = [Jp_1, JQ_1, zeros(7,3), zeros(7,3),  zeros(7,3), zeros(7,4)];
            jacobian(1:7*obj.n_pose_measures,:) = repmat(output_jacobian_1_kth,obj.n_pose_measures,1);
            
            % jacobian measures from robot 1
            b2Tb = b2_tildeTb2 * inv(obj.b1Tb2) * inv(obj.bTb1);
            b2Qb = Helper.quaternion_product(Helper.quaternion_inverse(b1Qb2),b1Qb);
            b2_tildeQb = Helper.quaternion_product(x(17:20),b2Qb);

            Q_tmp = Helper.quaternion_product(Helper.quaternion_inverse(b1Qb2),Helper.quaternion_product(b1Qb,bQo));

            Jp_2 = obj.jacobian_output_to_position(b2Tb);   %output position jacobian robot 1
            JQ_2 = obj.jacobian_output_to_quaternion_right(b2_tildeQb);  %output quaternion jacobian robot 1
            Jb1pb2 = obj.jacobian_b2po_wrtb1Tb2(inv(obj.b1Tb2)*inv(obj.bTb1)*bTo,x(17:20));
            Jb1Qb2 = obj.jacobian_output_to_quaternion_left(Q_tmp);
            output_jacobian_2_kth = [Jp_2, JQ_2, zeros(7,3), zeros(7,3), [Jb1pb2*1; 1*Jb1Qb2]];
            jacobian(7*obj.n_pose_measures+1:end,:) = repmat(output_jacobian_2_kth,obj.n_pose_measures,1);

        end

        function jacob = jacobian_b2po_wrtb1Tb2(obj,b1To, b1qb2)
                % Define the components of the quaternion part of the transformation
                b1To1_4 = b1To(1,4);
                b1To2_4 = b1To(2,4);
                b1To3_4 = b1To(3,4);
                qw = b1qb2(1);
                qx = b1qb2(2);
                qy = b1qb2(3);
                qz = b1qb2(4);
            
                % Initialize the Jacobian matrix
                jacob = zeros(3,7);
                jacob(1:3,1:3) = eye(3);
                % jacob(1:3,4:7) = [
                %     4*b1To1_4*qw + 2*b1To3_4*qy - 2*b1To2_4*qz, 4*b1To1_4*qx + 2*b1To2_4*qy + 2*b1To3_4*qz, 2*b1To3_4*qw + 2*b1To2_4*qx, 2*b1To3_4*qx - 2*b1To2_4*qw,
                %     4*b1To2_4*qw - 2*b1To3_4*qx + 2*b1To1_4*qz, 2*b1To1_4*qy - 2*b1To3_4*qw, 2*b1To1_4*qx + 4*b1To2_4*qy + 2*b1To3_4*qz, 2*b1To1_4*qw + 2*b1To3_4*qy,
                %     4*b1To3_4*qw + 2*b1To2_4*qx - 2*b1To1_4*qy, 2*b1To2_4*qw + 2*b1To1_4*qz, 2*b1To2_4*qz - 2*b1To1_4*qw, 2*b1To1_4*qx + 2*b1To2_4*qy + 4*b1To3_4*qz
                % ];

                jacob(1:3,4:7) = [
                  2*b1To2_4*qz - 2*b1To3_4*qy,                2*b1To2_4*qy + 2*b1To3_4*qz, 2*b1To2_4*qx - 2*b1To3_4*qw - 4*b1To1_4*qy, 2*b1To2_4*qw + 2*b1To3_4*qx - 4*b1To1_4*qz,
                  2*b1To3_4*qx - 2*b1To1_4*qz, 2*b1To3_4*qw - 4*b1To2_4*qx + 2*b1To1_4*qy,                2*b1To1_4*qx + 2*b1To3_4*qz, 2*b1To3_4*qy - 2*b1To1_4*qw - 4*b1To2_4*qz,
                - 2*b1To2_4*qx - 2*b1To1_4*qy, 2*b1To1_4*qz - 4*b1To3_4*qx - 2*b1To2_4*qw, 2*b1To2_4*qz - 4*b1To3_4*qy - 2*b1To1_4*qw,                2*b1To1_4*qx + 2*b1To2_4*qy];
 
        end

        function jacobian = jacobian_output_to_quaternion_right(obj,q)   
            % derivative of Q1*Q2 wrt Q2
            qw_1 = q(1); qx_1 = q(2); qy_1 = q(3); qz_1 = q(4);

            jacobian = zeros(7,4); 

            jacobian(4:7,1:4) = [qw_1, -qx_1, -qy_1, -qz_1,
                                qx_1,  qw_1, -qz_1,  qy_1,
                                qy_1,  qz_1,  qw_1, -qx_1,
                                qz_1, -qy_1,  qx_1,  qw_1];

        end 
        function jacobian = jacobian_output_to_quaternion_left(obj,q)  
            % derivative of Q1*Q2 wrt Q1
            qw_2 = q(1); qx_2 = q(2); qy_2 = q(3); qz_2 = q(4);

            jacobian = zeros(4,7); 

            jacobian(1:4,4:7) = [qw_2, -qx_2, -qy_2, -qz_2,
                                 qx_2,  qw_2,  qz_2, -qy_2,
                                 qy_2, -qz_2,  qw_2,  qx_2,
                                 qz_2,  qy_2, -qx_2,  qw_2];

        end 

        function clonedSystem = clone(obj)
            clonedSystem = RobotsObjectSystemExt(obj.state, obj.sizeState, obj.sizeOutput,obj.SampleTime, obj.Bm,obj.bg,obj.oTg1,obj.oTg2,obj.n_pose_measures ...
                                            ,obj.b1Tb2,obj.bTb1,obj.viscous_friction);
        end

    end
end
