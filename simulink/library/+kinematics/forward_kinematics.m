classdef forward_kinematics < matlab.System
    % Inertia matrix
    %
    % This template includes the minimum set of functions required
    % to define a System object.

    % Public, tunable properties
    properties

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
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.sdu_controllers = py.importlib.import_module('sdu_controllers');
            
            % Perform one-time calculations, such as computing constants
            switch obj.RobotType
                case obj.all_robot_types(1)
                    obj.robot_model = obj.sdu_controllers.models.BreedingBlanketHandlingRobotModel();
                    obj.links = 7;

                case obj.all_robot_types(2)
                    obj.robot_model = obj.sdu_controllers.models.URRobotModel(...
                        obj.sdu_controllers.models.RobotType(0) ...
                    );
                    obj.links = 6;

                case obj.all_robot_types(3)
                    obj.robot_model = obj.sdu_controllers.models.URRobotModel(...
                        obj.sdu_controllers.models.RobotType(1) ...
                    );
                    obj.links = 6;
            end

            % obj.links = double(obj.robot_model.get_dof());
            % disp(obj.links)
        end

        function [T] = stepImpl(obj, q)
            % Implement algorithm. Calculate y as a function of input u and
            % internal states.
            T = obj.sdu_controllers.kinematics.forward_kinematics(q, obj.robot_model);
            T = double(T);
        end

        function T = isOutputFixedSizeImpl(~)
            T = true;
        end

        function resetImpl(obj)
            % Initialize / reset internal properties
        end

        function T = getOutputSizeImpl(obj)
            T = [4, 4];
        end

        function T = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            T = "double";

            % Example: inherit data type from first input port
            % out = propagatedInputDataType(obj,1);
        end

        function T = isOutputComplexImpl(obj)
            % Return true for each output port with complex data
            T = false;
            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end

        function icon = getIconImpl(obj)
            icon = {'Forward Kinematics', obj.RobotType};
        end
    end
end
