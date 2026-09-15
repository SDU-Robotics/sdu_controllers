classdef eurofusion_hal < matlab.System
    % Converts generalized joint torques to motor torques via
    % sdu_controllers.hal.EurofusionRobot.generalized_tau_2_motor_tau

    properties
    end

    properties(Nontunable)
    end

    properties (Access = private)
        dof = 7        % number of joints (7)
        num_motors = 8  % number of motor outputs (8 -- joint 7 drives two motors)
        sdu_controllers
        eurofusion_robot
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.sdu_controllers = py.importlib.import_module('sdu_controllers');
            obj.eurofusion_robot = obj.sdu_controllers.hal.EurofusionRobot();
        end

        function [motor_tau] = stepImpl(obj, tau_g, q, qd)
            tau_g_py = py.numpy.array(reshape(tau_g, 1, obj.dof));
            q_py     = py.numpy.array(reshape(q,     1, obj.dof));
            qd_py    = py.numpy.array(reshape(qd,    1, obj.dof));

            result = obj.eurofusion_robot.generalized_tau_2_motor_tau(tau_g_py, q_py, qd_py);

            motor_tau = reshape(double(result), obj.num_motors, 1);
        end

        function motor_tau = isOutputFixedSizeImpl(~)
            motor_tau = true;
        end

        function resetImpl(obj)
        end

        function motor_tau = getOutputSizeImpl(obj)
            % Fixed at [8, 1] -- NOT derived from input size, since
            % joint 7 maps to two motor outputs.
            motor_tau = [obj.num_motors, 1];
        end

        function motor_tau = getOutputDataTypeImpl(obj)
            motor_tau = "double";
        end

        function motor_tau = isOutputComplexImpl(obj)
            motor_tau = false;
        end

        function icon = getIconImpl(obj)
            icon = {'Eurofusion HAL', 'generalized_tau_2_motor_tau'};
        end
    end
end