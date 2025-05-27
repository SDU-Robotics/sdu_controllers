classdef inertia_matrix < matlab.System
    % Forward Dynamics
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

        inv_dyn
    end

    methods (Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            switch obj.RobotType
                case obj.all_robot_types(1)
                    obj.robot_model = py.sdu_controllers.BreedingBlanketHandlingRobotModel();
                    obj.links = 7;

                case obj.all_robot_types(2)
                    obj.robot_model = py.sdu_controllers.URRobotModel(...
                        py.sdu_controllers.RobotType(0) ...
                    );
                    obj.links = 6;

                case obj.all_robot_types(3)
                    obj.robot_model = py.sdu_controllers.URRobotModel(...
                        py.sdu_controllers.RobotType(1) ...
                    );
                    obj.links = 6;
            end

            % obj.links = double(obj.robot_model.get_dof());
            % disp(obj.links)
        end

        function [B] = stepImpl(obj, q)
            % Implement algorithm. Calculate y as a function of input u and
            % internal states.
            B = obj.robot_model.get_inertia_matrix(q);
            B = reshape(double(B), obj.links, obj.links);
        end

        function B = isOutputFixedSizeImpl(~)
            B = true;
        end

        function resetImpl(obj)
            % Initialize / reset internal properties
        end

        function B = getOutputSizeImpl(obj)
            qsize = propagatedInputSize(obj, 1);
            B = [qsize(1), qsize(1)];
        end

        function B = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            B = "double";

            % Example: inherit data type from first input port
            % out = propagatedInputDataType(obj,1);
        end

        function B = isOutputComplexImpl(obj)
            % Return true for each output port with complex data
            B = false;
            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end

        function icon = getIconImpl(obj)
            icon = {'Inertia Matrix', obj.RobotType};
        end
    end
end
