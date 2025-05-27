classdef pid_controller < matlab.System
    % PID Controller
    %
    % This template includes the minimum set of functions required
    % to define a System object.

    % Public, tunable properties
    properties
        Kp = 100.0
        Ki = 0.0
        Kd = 2*sqrt(100.0)
        N = 1
        
        % num_states Number of States
        num_states = 6
        
        % sample_time Sample Time
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
            obj.Kp_mat = obj.Kp * eye(obj.num_states);
            obj.Ki_mat = obj.Ki * eye(obj.num_states);
            obj.Kd_mat = obj.Kd * eye(obj.num_states);
            obj.N_mat = obj.N * eye(obj.num_states);
           
            %
            obj.pid_contr = py.sdu_controllers.PIDController(obj.Kp_mat, ...
                obj.Ki_mat, obj.Kd_mat, obj.N_mat, obj.sample_time);
        end

        % function sts = getSampleTimeImpl(obj)
        %     disp("test")
        %     sts = createSampleTime(obj, 'Type', 'Discrete', ...
        %         'SampleTime', obj.sample_time);
        % end

        function [y] = stepImpl(obj, q_d, dq_d, u_ff, q, dq)
            obj.pid_contr.step(q_d, dq_d, u_ff, q, dq);

            y = double(obj.pid_contr.get_output());
            % y = double(obj.pid_contr.output);
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
