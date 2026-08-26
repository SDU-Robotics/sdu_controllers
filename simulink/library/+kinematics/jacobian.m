classdef jacobian < matlab.System
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
        dof
        fk_solver

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

                case obj.all_robot_types(2)
                    obj.robot_model = obj.sdu_controllers.models.URRobotModel(...
                        obj.sdu_controllers.models.RobotType(0) ...
                    );

                case obj.all_robot_types(3)
                    obj.robot_model = obj.sdu_controllers.models.URRobotModel(...
                        obj.sdu_controllers.models.RobotType(1) ...
                    );
            end

            obj.dof = double(obj.robot_model.get_dof());
        end

        function [J] = stepImpl(obj, q)
            % Implement algorithm. Calculate y as a function of input u and
            % internal states.
            J = obj.robot_model.get_jacobian(q');
            
            J = double(J);
        end

        function J = isOutputFixedSizeImpl(~)
            J = true;
        end

        function resetImpl(obj)
            % Initialize / reset internal properties
        end

        function J = getOutputSizeImpl(obj)
            switch obj.RobotType
                case obj.all_robot_types(1)
                    obj.links = 6;

                case obj.all_robot_types(2)
                    obj.links = 6;

                case obj.all_robot_types(3)
                    obj.links = 6;
            end
            J = [6,obj.links];
        end

        function J = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            J = "double";

            % Example: inherit data type from first input port
            % out = propagatedInputDataType(obj,1);
        end

        function J = isOutputComplexImpl(obj)
            % Return true for each output port with complex data
            J = false;
            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end

        function icon = getIconImpl(obj)
            icon = {'Jacobian', obj.RobotType};
        end
    end
end
