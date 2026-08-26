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
        dof

        sdu_controllers
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.sdu_controllers = py.importlib.import_module('sdu_controllers');
            
            obj.dof = double(obj.robot_model.get_dof());
        end

        function tau = stepImpl(obj, q)

            q = reshape(q, 1, obj.dof);

            tau = obj.robot_model.get_gravity(q);
            tau = reshape(double(tau), obj.dof, 1);
        end

        function tau = isOutputFixedSizeImpl(~)
            tau = true;
        end

        function resetImpl(obj)

        end

        function tau = getOutputSizeImpl(obj)
            tau = propagatedInputSize(obj, 1);
        end

        function tau = getOutputDataTypeImpl(obj)

            tau = "double";

        end

        function tau = isOutputComplexImpl(obj)

            tau = false;

        end

        function icon = getIconImpl(obj)
            icon = {'Gravity Torques', class(obj.robot_model)};
        end
    end
end
