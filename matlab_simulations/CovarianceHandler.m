% Define a class to handle the covariance matrix
% The update rule is the following
% V_k+1 = (alpa_occlusion)^alpha_state * V_default; 
classdef CovarianceHandler < handle
    properties
        alpha_state % exponential
        alpha_occlusion % exponential base
        num_exponential_steps % maximum number of exponential steps
        V_default % starting covariance matrix
    end
    
    methods
        function obj = SimpleSystem(alpha_occlusion, num_exponential_steps, V_default)
            obj.alpha_state = 0;
            obj.alpha_occlusion = alpha_occlusion;
            obj.num_exponential_steps = num_exponential_steps;
            obj.V_default = V_default;
        end
        
        function state = getState(obj)
            state = obj.state;
        end
        
        function sizeState = getSizeState(obj)
            sizeState = obj.sizeState;
        end

        function updateState(obj,state)
            obj.state = state;
        end
        
        function sizeOutput = getSizeOutput(obj)
            sizeOutput = obj.sizeOutput;
        end
        
        function newState = state_fcn(obj, x, u)
            % Define the state transition function
            newState = (-x + u);
        end
        
        function output = output_fcn(obj, x, u)
            % Define the output function
            output = x(1) + x(2); % Example: direct state as output
        end
        
        function jacobian = jacob_state_fcn(obj, x, u)
            % Define the Jacobian of the state transition function
            jacobian = eye(length(x)); % Example: identity matrix
        end
        
        function jacobian = jacob_output_fcn(obj, x, u)
            % Define the Jacobian of the output function
            jacobian = [1 1];%eye(length(x)); % Example: identity matrix
        end
        
        function clonedSystem = clone(obj)
            clonedSystem = SimpleSystem(obj.state, obj.sizeState, obj.sizeOutput);
        end
    end
end

