classdef forward_dynamics < matlab.System
    % Forward Dynamics
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

        fwd_dyn

        sdu_controllers
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.sdu_controllers = py.importlib.import_module('sdu_controllers');

            obj.links = double(obj.robot_model.get_dof());
            % disp(obj.links)

            obj.fwd_dyn = obj.sdu_controllers.math.ForwardDynamics(obj.robot_model);
        end

        function ddq = stepImpl(obj, q, dq, tau)
            % Implement algorithm. Calculate y as a function of input u and
            % internal states.
            q = reshape(q, 1, obj.links);
            dq = reshape(dq, 1, obj.links);
            tau = reshape(tau, 1, obj.links);

            ddq = obj.fwd_dyn.forward_dynamics(q, dq, tau);
            % disp(ddq)
            % ddq = double(ddq).';
            ddq = reshape(double(ddq), obj.links, 1);
            % disp(ddq)
        end

        function ddq = isOutputFixedSizeImpl(~)
            ddq = true;
        end

        function resetImpl(obj)
            % Initialize / reset internal properties
        end

        function ddq = getOutputSizeImpl(obj)
            ddq = propagatedInputSize(obj, 1);
        end

        function ddq = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            ddq = "double";

            % Example: inherit data type from first input port
            % out = propagatedInputDataType(obj,1);
        end

        function ddq = isOutputComplexImpl(obj)
            % Return true for each output port with complex data
            ddq = false;
            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end

        function icon = getIconImpl(obj)
            icon = {'Forward Dynamics', class(obj.robot_model)};
        end
    end
end
