classdef forward_dynamics < matlab.System
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

        fwd_dyn
    end

    methods (Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            switch obj.RobotType
                case obj.all_robot_types(1)
                    obj.robot_model = py.sdu_controllers.BreedingBlanketHandlingRobotModel();

                case obj.all_robot_types(2)
                    obj.robot_model = py.sdu_controllers.URRobotModel(...
                        py.sdu_controllers.RobotType(0) ...
                    );

                case obj.all_robot_types(3)
                    obj.robot_model = py.sdu_controllers.URRobotModel(...
                        py.sdu_controllers.RobotType(1) ...
                    );
            end

            obj.links = double(obj.robot_model.get_dof());
            disp(obj.links)

            obj.fwd_dyn = py.sdu_controllers.ForwardDynamics(obj.robot_model);
        end

        function ddq = stepImpl(obj, q, dq, tau)
            % Implement algorithm. Calculate y as a function of input u and
            % internal states.
            ddq = obj.fwd_dyn.forward_dynamics(q, dq, tau);
            disp(ddq)
            ddq = double(ddq).';
            disp(ddq)
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
    end
end
