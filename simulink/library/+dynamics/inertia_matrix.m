classdef inertia_matrix < matlab.System
    % Inertia matrix
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
        links

        sdu_controllers
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.sdu_controllers = py.importlib.import_module('sdu_controllers');

            obj.links = double(obj.robot_model.get_dof());
        end

        function [B] = stepImpl(obj, q)
            % Implement algorithm. Calculate y as a function of input u and
            % internal states.
            q = reshape(q, 1, obj.links);
            B = obj.robot_model.get_inertia_matrix(q);
            B = reshape(double(B), obj.links, obj.links);
        end

        function B = isOutputFixedSizeImpl(~)
            B = true;
        end

        function resetImpl(obj)
            % Initialize / reset internal properties
        end

        function B = getOutputSizeImpl(obj)
            qsize = propagatedInputSize(obj, 1);
            B = [qsize(1), qsize(1)];
        end

        function B = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            B = "double";

            % Example: inherit data type from first input port
            % out = propagatedInputDataType(obj,1);
        end

        function B = isOutputComplexImpl(obj)
            % Return true for each output port with complex data
            B = false;
            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end

        function icon = getIconImpl(obj)
            icon = {'Inertia Matrix', class(obj.robot_model)};
        end
    end
end
