classdef AdvancedSystem < SimpleSystem
    methods
        function obj = AdvancedSystem(state, sizeState, sizeOutput)
            % Call the constructor of the superclass
            obj@SimpleSystem(state, sizeState, sizeOutput);
        end
        
        function newState = state_fcn(obj, x, u)
            % Override the state transition function
            newState = x + 2 * u; % Example: different state transition
        end
        
        function output = output_fcn(obj, x, u)
            % Override the output function
            output = x + 0.5 * u; % Example: different output function
        end
        
        function jacobian = jacob_state_fcn(obj, x, u)
            % Override the Jacobian of the state transition function
            jacobian = 2 * eye(length(x)); % Example: different Jacobian
        end
        
        function jacobian = jacob_output_fcn(obj, x, u)
            % Override the Jacobian of the output function
            jacobian = 0.5 * eye(length(x)); % Example: different Jacobian
        end
    end
end
