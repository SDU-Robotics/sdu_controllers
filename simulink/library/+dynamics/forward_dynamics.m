classdef forward_dynamics < matlab.System
    % Forward Dynamics

    properties

    end

    properties(Nontunable)
        robot_model
    end


    properties (Access = private)
        dof

        fwd_dyn

        sdu_controllers
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.sdu_controllers = py.importlib.import_module('sdu_controllers');

            obj.dof = double(obj.robot_model.get_dof());

            obj.fwd_dyn = obj.sdu_controllers.math.ForwardDynamics(obj.robot_model);
        end

        function ddq = stepImpl(obj, q, dq, tau)
            q = reshape(q, 1, obj.dof);
            dq = reshape(dq, 1, obj.dof);
            tau = reshape(tau, 1, obj.dof);

            ddq = obj.fwd_dyn.forward_dynamics(q, dq, tau);

            ddq = reshape(double(ddq), obj.dof, 1);

        end

        function ddq = isOutputFixedSizeImpl(~)
            ddq = true;
        end

        function resetImpl(obj)

        end

        function ddq = getOutputSizeImpl(obj)
            ddq = propagatedInputSize(obj, 1);
        end

        function ddq = getOutputDataTypeImpl(obj)

            ddq = "double";

        end

        function ddq = isOutputComplexImpl(obj)

            ddq = false;

        end

        function icon = getIconImpl(obj)
            icon = {'Forward Dynamics', class(obj.robot_model)};
        end
    end
end
