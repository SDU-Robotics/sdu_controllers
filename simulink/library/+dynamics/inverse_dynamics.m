classdef inverse_dynamics < matlab.System
    % Inverse Dynamics
    %

    properties
    end

    properties(Nontunable)
        robot_model
    end

    properties (Access = private)
        dof

        inv_dyn

        sdu_controllers
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.sdu_controllers = py.importlib.import_module('sdu_controllers');

            obj.dof = double(obj.robot_model.get_dof());

            obj.inv_dyn = obj.sdu_controllers.math.InverseDynamicsJointSpace(obj.robot_model);
        end

        function tau = stepImpl(obj, q, dq, y)

            y = reshape(y, 1, obj.dof);
            q = reshape(q, 1, obj.dof);
            dq = reshape(dq, 1, obj.dof);

            tau = obj.inv_dyn.inverse_dynamics(y, q, dq);
            tau = reshape(double(tau), propagatedInputSize(obj, 1));

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
% ln
        function tau = getOutputDataTypeImpl(obj)

            tau = "double";

        end

        function tau = isOutputComplexImpl(obj)
            tau = false;

        end

        function icon = getIconImpl(obj)
            icon = {'Inverse Dynamics', class(obj.robot_model)};
        end
    end
end
