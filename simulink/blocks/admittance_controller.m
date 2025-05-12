classdef admittance_controller < matlab.System
    % Admittance Controller
    %
    % This template includes the minimum set of functions required
    % to define a System object.

    % Public, tunable properties
    properties
        Mp = eye(3)
        Kp = eye(3)
        Dp = eye(3)
        Mo = eye(3)
        Ko = eye(3)
        Do = eye(3)
        sample_time = 1/500.
    end

    % Pre-computed constants or internal states
    properties (Access = private)        
        adm_contr
    end

    methods (Access = protected)
        function setupImpl(obj)          
            frequency = 1/obj.sample_time;
            obj.adm_contr = ...
                py.sdu_controllers.AdmittanceControllerPosition(frequency);

            obj.adm_contr.set_mass_matrix_position(obj.Mp);
            obj.adm_contr.set_stiffness_matrix_position(obj.Kp);
            obj.adm_contr.set_damping_matrix_position(obj.Dp);

            obj.adm_contr.set_mass_matrix_orientation(obj.Mo);
            obj.adm_contr.set_stiffness_matrix_orientation(obj.Ko);
            obj.adm_contr.set_damping_matrix_orientation(obj.Do);
        end

        function [x_c, quat_c] = stepImpl(obj, input_force, input_torque, ...
                x_desired, quat_desired)
            % const Eigen::Vector3d &input_force, 
            % const Eigen::Vector3d &input_torque, 
            % const Eigen::Vector3d &x_desired, 
            % const Eigen::Vector4d &quat_desired

            obj.adm_contr.step(input_force, input_torque, ...
                x_desired, quat_desired);

            y = double(obj.adm_contr.get_output());
            
            x_c = y(1:3);
            quat_c = y(4:7);            
        end

        function [x_c, quat_c] = isOutputFixedSizeImpl(~)
            x_c = true;
            quat_c = true;
        end

        % function resetImpl(obj)
        %     % Initialize / reset internal properties
        % end

        function [x_c, quat_c] = getOutputSizeImpl(obj)
            % Return size for each output port
            x_c = [1 3];
            quat_c = [1 4]; 

            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end

        function [x_c, quat_c] = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            x_c = "double";
            quat_c = "double";

            % Example: inherit data type from first input port
            % out = propagatedInputDataType(obj,1);
        end

        function [x_c, quat_c] = isOutputComplexImpl(obj)
            % Return true for each output port with complex data
            x_c = false;
            quat_c = false;
            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end
    end
end
