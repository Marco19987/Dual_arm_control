classdef RobotsObjectSystemExtForces < SimpleSystem
    % Add b1Tb2 online estimation

    properties   
        base_system;

    end
    methods
        function obj = RobotsObjectSystemExtForces(initialState,system)
            % Call the constructor of the superclass
            obj@SimpleSystem(initialState, system.sizeState+12, system.sizeOutput, system.SampleTime);
            obj.base_system = system;            
           
        end

        function xdot = eval_xdot(obj, x, u)
            xdot(1:20) = obj.base_system.eval_xdot(x(1:20),x(21:32));
            xdot(21:32) = zeros(12,1);
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
            Bm = obj.base_system.base_system.Bm;
            W = obj.base_system.base_system.W;
            Rbar = obj.base_system.base_system.Rbar;
            J = Bm \ (W*Rbar);

            jacobian = zeros(length(x));
            jacobian(1:20,1:20) = obj.base_system.jacob_state_fcn(x(1:20),x(21:32));
            jacobian(21:32,21:32) = zeros(12);
            jacobian(8:13,21:32) = obj.SampleTime*J;
            jacobian(21:32,21:32) = eye(12) + obj.SampleTime * jacobian(21:32,21:32);

        end

                
        function output = output_fcn(obj, x, u)
           
           output = obj.base_system.output_fcn(x(1:20),x(21:32));
           output = [output; x(21:32)];

        end

        function jacobian = jacob_output_fcn(obj, x, u)
            jacobian_base_system = obj.base_system.jacob_output_fcn(x(1:20),x(21:32));
            [r,c] = size(jacobian_base_system);
            jacobian = [jacobian_base_system zeros(r,12); zeros(12,c), eye(12)];
        end


        

        function clonedSystem = clone(obj)
            clonedSystem = RobotsObjectSystemExtForces(obj.state, obj.base_system);
        end

    end
end
