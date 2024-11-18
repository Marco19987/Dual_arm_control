classdef RobotsObjectSystemExt < RobotsObjectSystem 
    % Add b1Tb2 online estimation

    properties      

    end
    methods
        function obj = RobotsObjectSystemExt(state, sizeState, sizeOutput,SampleTime, Bm,bg,oTg1,oTg2,n_pose_measures ...
                                            ,b1Tb2,bTb1,viscous_friction)
            % Call the constructor of the superclass
            obj@RobotsObjectSystem(state, sizeState, sizeOutput,SampleTime, Bm,bg,oTg1,oTg2,n_pose_measures ...
                                            ,b1Tb2,bTb1,viscous_friction)            
           
        end

        function xdot = eval_xdot(obj, x, u)
            x(4:7) = x(4:7)/norm(x(4:7));
            x(17:20) = x(17:20)/norm(x(4:7));

            % implementation of xdot = f(x,u)
            bvo = x(8:10);      % object's linear velocity in the base frame
            bomegao = x(11:13); % object's angular velocity in the base frame
            bQo = x(4:7);       % object's quaternion in the base frame
            g1hg1 = u(1:6);     % robot1's grasp wrench in the grasp frame 1
            g2hg2 = u(7:12);    % robot2's grasp wrench in the grasp frame 2
            oh = obj.W * obj.Rbar * [g1hg1;g2hg2];  % resulting wrench in the object frame
            bRo = quat2rotm(bQo');

            xdot(1:3) = bvo;
            xdot(4:7) = Helper.quaternion_propagation(bQo,bomegao);
            xdot(8:13) = blkdiag(bRo,bRo)*(obj.Bm\oh) + [obj.bg;0;0;0] - obj.viscous_friction*[bvo;bomegao];
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
                
        function output = output_fcn(obj, x, u)
           % in the output there are the measures coming from the object pose
           % estimators expressed in the respective robot's base frame: bkTo
           % there is only a transformation involved in the relation 
           x(4:7) = x(4:7)/norm(x(4:7));
           x(17:20) = x(17:20)/norm(x(4:7));
           
           obj.b1Tb2 = Helper.transformation_matrix(x(14:16), x(17:20));
           b1Qb2 = x(17:20);
            
       
           output = zeros(7*obj.n_pose_measures*2,1); 
          
           bTo = Helper.transformation_matrix(x(1:3), x(4:7));
            
           b1To = obj.bTb1 \ bTo; % measured object pose from robot 1 with the k-th pose estimator in the base1 frame
           b2To = inv(obj.b1Tb2) * inv(obj.bTb1) * bTo; % measured object pose from robot 2 with the k-th pose estimator in the base2 frame
            
           b1Qo = Helper.quaternion_product(rotm2quat(obj.bTb1(1:3,1:3)'),x(4:7));

           b2Qb1 = Helper.quaternion_inverse(b1Qb2);
           b1Qb = Helper.quaternion_inverse(rotm2quat(obj.bTb1(1:3,1:3)'));
           b2Qo =  Helper.quaternion_product(Helper.quaternion_product(b2Qb1,b1Qb),x(4:7));
         
           b1qo = [b1To(1:3,4);b1Qo'];
           b2qo = [b2To(1:3,4);b2Qo'];
            
           output(1:7*obj.n_pose_measures) = repmat(b1qo,obj.n_pose_measures,1);
           output(7*obj.n_pose_measures+1:end) = repmat(b2qo,obj.n_pose_measures,1);

            
        end

        function jacobian = jacob_output_fcn(obj, x, u)
            x(4:7) = x(4:7)/norm(x(4:7));
            x(17:20) = x(17:20)/norm(x(4:7));


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
            b2Tb = inv(obj.b1Tb2) * inv(obj.bTb1);
            b2Qb = Helper.quaternion_product(Helper.quaternion_inverse(b1Qb2),b1Qb);
            Jp_2 = obj.jacobian_output_to_position(b2Tb);   %output position jacobian robot 1
            JQ_2 = obj.jacobian_output_to_quaternion_right(b2Qb);  %output quaternion jacobian robot 1
            Jb1pb2 = obj.jacobian_b2po_wrtb1Tb2(obj.bTb1\bTo,b1Qb2);
            Jb1Qb2 = obj.jacobian_output_to_quaternion_left(Helper.quaternion_product(b1Qb,bQo));
            output_jacobian_2_kth = [Jp_2, JQ_2, zeros(7,3), zeros(7,3), [Jb1pb2*1; 1*Jb1Qb2']];
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
                jacob(1:3,4:7) = [
                    4*b1To1_4*qw + 2*b1To3_4*qy - 2*b1To2_4*qz, 4*b1To1_4*qx + 2*b1To2_4*qy + 2*b1To3_4*qz, 2*b1To3_4*qw + 2*b1To2_4*qx, 2*b1To3_4*qx - 2*b1To2_4*qw,
                    4*b1To2_4*qw - 2*b1To3_4*qx + 2*b1To1_4*qz, 2*b1To1_4*qy - 2*b1To3_4*qw, 2*b1To1_4*qx + 4*b1To2_4*qy + 2*b1To3_4*qz, 2*b1To1_4*qw + 2*b1To3_4*qy,
                    4*b1To3_4*qw + 2*b1To2_4*qx - 2*b1To1_4*qy, 2*b1To2_4*qw + 2*b1To1_4*qz, 2*b1To2_4*qz - 2*b1To1_4*qw, 2*b1To1_4*qx + 2*b1To2_4*qy + 4*b1To3_4*qz
                ];
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

            jacobian = zeros(7,4); 

            jacobian(4:7,1:4) = [qw_2, -qx_2, -qy_2, -qz_2,
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
