classdef set_tcp_mass < matlab.System
    % Set TCP mass
    %
    % This template includes the minimum set of functions required
    % to define a System object.

    % Public, tunable properties
    properties
    end

    properties(Nontunable)
        robot_model
    end

    % Pre-computed constants or internal states
    properties (Access = private)
        % sdu_controllers
    end

    methods (Access = protected)
        function setupImpl(obj)
            % obj.sdu_controllers = py.importlib.import_module('sdu_controllers');
        end

        function [] = stepImpl(obj, mass, com, inertia)
            obj.robot_model.set_tcp_mass(mass, com, inertia);
        end

        function [] = isOutputFixedSizeImpl(~)
            
        end

        function resetImpl(obj)
            % Initialize / reset internal properties
        end

        function num = getNumInputsImpl(~)
            num = 3;
        end

        function flag = isInputSizeMutableImpl(obj, ~)
            flag = false;
        end

        function validateInputsImpl(~, mass, com, inertia)
            if mass < 0
                error('Mass has to be positive');
            end

            if length(com) ~= 3
                error('CoM has to be a vector 3');
            end

            if any(size(inertia) ~= [3, 3])
                error("Inertia has to be a 3x3 matrix");
            end
        end

        function [] = getOutputSizeImpl(obj)
        end

        function [] = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            % out = propagatedInputDataType(obj,1);
        end

        function [] = isOutputComplexImpl(obj)
            % Return true for each output port with complex data
        end

        function icon = getIconImpl(obj)
            icon = {'Set TCP mass', class(obj.robot_model)};
        end
    end
end
