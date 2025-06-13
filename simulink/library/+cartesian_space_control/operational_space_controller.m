classdef operational_space_controller < matlab.System
    % Operational Space Controller
    % x_d is position in [x, y, z] and rotation in ZYZ Eueler
    % angles.
    %
    % This template includes the minimum set of functions required
    % to define a System object.

    % Public, tunable properties
    properties
        Kp = eye(6)
        Kd = eye(6)
    end

    properties(Nontunable)
        RobotType {mustBeMember(RobotType, ["BB Handler", "UR3e", "UR5e"])} = "UR5e"
    end

    % Pre-computed constants or internal states
    properties (Access = private)        
        robot_model
        links

        all_robot_types = ["BB Handler", "UR3e", "UR5e"];
        
        sdu_controllers
        os_contr
    end

    methods (Access = protected)
        function setupImpl(obj)          
            obj.sdu_controllers = py.importlib.import_module('sdu_controllers');

            % Perform one-time calculations, such as computing constants
            switch obj.RobotType
                case obj.all_robot_types(1)
                    obj.robot_model = obj.sdu_controllers.models.BreedingBlanketHandlingRobotModel();

                case obj.all_robot_types(2)
                    obj.robot_model = obj.sdu_controllers.models.URRobotModel(...
                        obj.sdu_controllers.models.RobotType(0) ...
                    );

                case obj.all_robot_types(3)
                    obj.robot_model = obj.sdu_controllers.models.URRobotModel(...
                        obj.sdu_controllers.models.RobotType(1) ...
                    );
            end

            obj.links = double(obj.robot_model.get_dof());

            obj.os_contr = obj.sdu_controllers.controllers.OperationalSpaceController(obj.Kp, obj.Kd, obj.robot_model);
        end

        function [y] = stepImpl(obj, x_d, dx_d, ddx_d, q, dq)
            obj.os_contr.step(x_d, dx_d, ddx_d, q, dq);

            y = double(obj.os_contr.get_output());
            y = reshape(y, obj.links, 1);
        end

        function [y] = isOutputFixedSizeImpl(~)
            y = true;
        end

        % function resetImpl(obj)
        %     % Initialize / reset internal properties
        % end

        function [y] = getOutputSizeImpl(obj)
            % Example: inherit size from fourth input port
            y = propagatedInputSize(obj, 4);
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

        function icon = getIconImpl(obj)
            icon = {'Operational Space', 'Controller', obj.RobotType};
        end
    end
end
