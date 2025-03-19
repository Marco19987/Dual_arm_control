classdef RobotsObjectSystemExt < SimpleSystem
    % Add b2Tb1 online estimation

    properties   
        base_system;

    end
    methods
        function obj = RobotsObjectSystemExt(initialState,system)
            % Call the constructor of the superclass
            obj@SimpleSystem(initialState, system.sizeState+7, system.sizeOutput, system.SampleTime);
            obj.base_system = system;            
           
        end

        function update_b2Tb1(obj, b2Tb1)
            obj.base_system.b2Tb1 = b2Tb1;
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

        end

         function jacobian = jacob_state_fcn(obj, x, u)
            jacobian = zeros(length(x));
            jacobian(1:13,1:13) = obj.base_system.jacob_state_fcn(x(1:13),u);
            jacobian(14:end,14:end) = eye(7);
        end

                
        function output = output_fcn(obj, x, u)
           x(4:7) = Helper.normalize_quaternion(x(4:7));
           x(17:20) = Helper.normalize_quaternion(x(17:20));

           b2Tb1 = obj.base_system.b2Tb1; %store b2Tb1 state
           obj.update_b2Tb1(Helper.transformation_matrix(x(14:16), x(17:20)));
           
           output = obj.base_system.output_fcn(x(1:13),u);

           obj.update_b2Tb1(b2Tb1); % reset to the original value - dirty solution 
        
        end

        function jacobian = jacob_output_fcn(obj, x, u)
            b2Tb1 = obj.base_system.b2Tb1; %store b2Tb1 state
            obj.update_b2Tb1(Helper.transformation_matrix(x(14:16), x(17:20)));

            x(4:7) = Helper.normalize_quaternion(x(4:7));
            x(17:20) = Helper.normalize_quaternion(x(17:20));

            bQo = x(4:7);
            b2Qb1 = x(17:20);
            bQb1 = rotm2quat((obj.base_system.bTb1(1:3,1:3)))';
            b1Qb = Helper.quaternion_inverse(bQb1)';
            

            bTo = Helper.transformation_matrix(x(1:3), bQo);
            bTb1 = obj.base_system.bTb1;

   
            jacobian = zeros(7*2*obj.base_system.n_pose_measures,length(x));

            jacobian(:,1:13) = obj.base_system.jacob_output_fcn(x(1:13),u);

            
            
            % jacobian measures from robot 2
            b1To = bTb1 \ bTo;
            b1Qo = Helper.quaternion_product(b1Qb,bQo)';

            J_b2po_b2pb1 = eye(3); 
            J_b2po_b2Qb1 = obj.Jacobian_Rp_to_Q(b2Qb1,b1To(1:3,4));
            J_b2Qo_b2Qb1 = obj.base_system.Jacobian_quaternion_product_left(b1Qo);

            output_jacobian_2_kth = [J_b2po_b2pb1 J_b2po_b2Qb1; zeros(4,3) J_b2Qo_b2Qb1];
            jacobian(7*obj.base_system.n_pose_measures+1:end,14:end) = repmat(output_jacobian_2_kth,obj.base_system.n_pose_measures,1);

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


        

        function clonedSystem = clone(obj)
            clonedSystem = RobotsObjectSystemExt(obj.state, obj.base_system);
        end

    end
end
