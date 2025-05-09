classdef pid_controller < matlab.System
    % PDController
    %
    % This template includes the minimum set of functions required
    % to define a System object.

    % Public, tunable properties
    properties
       Kp = 100.0
       Ki = 0.0
       Kd = 2*sqrt(100.0)
       N = 1
       num_states = 6
       sample_time = 1/500.
    end

    % Pre-computed constants or internal states
    properties (Access = private)        
        Kp_mat
        Ki_mat
        Kd_mat
        N_mat
        pid_contr
    end

    methods (Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            % obj.Kp_mat = py.numpy.asarray(obj.Kp * eye(obj.num_states));
            % obj.Ki_mat = py.numpy.asarray(obj.Ki * eye(obj.num_states));
            % obj.Kd_mat = py.numpy.asarray(obj.Kd * eye(obj.num_states));
            % obj.N_mat = py.numpy.asarray(obj.N * eye(obj.num_states));
            obj.Kp_mat = obj.Kp * eye(obj.num_states);
            obj.Ki_mat = obj.Ki * eye(obj.num_states);
            obj.Kd_mat = obj.Kd * eye(obj.num_states);
            obj.N_mat = obj.N * eye(obj.num_states);
            
            % Reshaping is necessary when the number of states is only 1.
            obj.Kp_mat = py.numpy.reshape(py.numpy.asarray(obj.Kp_mat), [int16(obj.num_states), int16(obj.num_states)]);
            obj.Ki_mat = py.numpy.reshape(py.numpy.asarray(obj.Ki_mat), [int16(obj.num_states), int16(obj.num_states)]);
            obj.Kd_mat = py.numpy.reshape(py.numpy.asarray(obj.Kd_mat), [int16(obj.num_states), int16(obj.num_states)]);
            obj.N_mat = py.numpy.reshape(py.numpy.asarray(obj.N_mat), [int16(obj.num_states), int16(obj.num_states)]);

            %
            obj.pid_contr = py.sdu_controllers.PIDController(obj.Kp_mat, ...
                obj.Ki_mat, obj.Kd_mat, obj.N_mat, obj.sample_time);
        end

        function [y] = stepImpl(obj, q_d, dq_d, u_ff, q, dq)
            % Reshaping is necessary when the number of states is only 1.
            q_d_np = py.numpy.reshape(q_d, [int8(obj.num_states),]);
            dq_d_np = py.numpy.reshape(dq_d, [int8(obj.num_states),]);
            u_ff_np = py.numpy.reshape(u_ff, [int8(obj.num_states),]);
            q_np = py.numpy.reshape(q, [int8(obj.num_states),]);
            dq_np = py.numpy.reshape(dq, [int8(obj.num_states),]);

            %
            obj.pid_contr.step(q_d_np, dq_d_np, u_ff_np, q_np, dq_np);

            output = obj.pid_contr.get_output();
            % y = double(py.array.array('d', output.flatten('F').tolist()));
            % y = double(obj.pid_contr.get_output().tolist());
            % y = double(output);
            y = double(output.tolist());
        end

        function [y] = isOutputFixedSizeImpl(~)
            y = true;
        end

        % function resetImpl(obj)
        %     % Initialize / reset internal properties
        % end

        function [y] = getOutputSizeImpl(obj)
            % Return size for each output port
            y = [obj.num_states 1];

            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end

        function [y] = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            y = "double";

            % Example: inherit data type from first input port
            % out = propagatedInputDataType(obj,1);
        end

        function [y] = isOutputComplexImpl(obj)
            % Return true for each output port with complex data
            y = false;
            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end
    end
end
