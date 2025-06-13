classdef gravity_torques < matlab.System
    % Gravity torques
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
            % disp(obj.links)
        end

        function tau = stepImpl(obj, q)
            % Implement algorithm. Calculate y as a function of input u and
            % internal states.
            q = reshape(q, 1, obj.links);

            tau = obj.robot_model.get_gravity(q);
            tau = reshape(double(tau), obj.links, 1);
        end

        function tau = isOutputFixedSizeImpl(~)
            tau = true;
        end

        function resetImpl(obj)
            % Initialize / reset internal properties
        end

        function tau = getOutputSizeImpl(obj)
            tau = propagatedInputSize(obj, 1);
        end

        function tau = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            tau = "double";

            % Example: inherit data type from first input port
            % out = propagatedInputDataType(obj,1);
        end

        function tau = isOutputComplexImpl(obj)
            % Return true for each output port with complex data
            tau = false;
            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end

        function icon = getIconImpl(obj)
            icon = {'Gravity Torques', class(obj.robot_model)};
        end
    end
end
