classdef coriolis_matrix < matlab.System
    % Coriolis matrix
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
            % Perform one-time calculations, such as computing constants
            obj.sdu_controllers = py.importlib.import_module('sdu_controllers');

            obj.links = double(obj.robot_model.get_dof());
            % disp(obj.links)
        end

        function [C] = stepImpl(obj, q, dq)
            % Implement algorithm. Calculate y as a function of input u and
            % internal states.
            q = reshape(q, 1, obj.links);
            dq = reshape(dq, 1, obj.links);

            C = obj.robot_model.get_coriolis(q, dq);
            C = reshape(double(C), obj.links, obj.links);
        end

        function C = isOutputFixedSizeImpl(~)
            C = true;
        end

        function resetImpl(obj)
            % Initialize / reset internal properties
        end

        function C = getOutputSizeImpl(obj)
            qsize = propagatedInputSize(obj, 1);
            C = [qsize(1), qsize(1)];
        end

        function C = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            C = "double";

            % Example: inherit data type from first input port
            % out = propagatedInputDataType(obj,1);
        end

        function C = isOutputComplexImpl(obj)
            % Return true for each output port with complex data
            C = false;
            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end

        function icon = getIconImpl(obj)
            icon = {'Coriolis Matrix', class(obj.robot_model)};
        end
    end
end
