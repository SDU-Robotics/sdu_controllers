classdef forward_kinematics < matlab.System

    properties

    end

    properties(Nontunable)
        RobotType {mustBeMember(RobotType, ["BB Handler", "UR3e", "UR5e"])} = "UR5e"
    end


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
                    obj.robot_model = obj.sdu_controllers.models.ParameterRobotModel("/home/madla/Documents/sdu_controllers/config/models/breeding_blanket_handling_robot.yaml");

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
            obj.fk_solver = obj.robot_model.get_fk_solver();

        end

        function [T] = stepImpl(obj, q)

            T = obj.fk_solver.forward_kinematics(q);
            T = double(T);
        end

        function T = isOutputFixedSizeImpl(~)
            T = true;
        end

        function resetImpl(obj)

        end

        function T = getOutputSizeImpl(obj)
            T = [4, 4];
        end

        function T = getOutputDataTypeImpl(obj)

            T = "double";

        end

        function T = isOutputComplexImpl(obj)

            T = false;

        end

        function icon = getIconImpl(obj)
            icon = {'Forward Kinematics', obj.RobotType};
        end
    end
end
