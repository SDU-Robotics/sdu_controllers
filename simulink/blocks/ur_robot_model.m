classdef ur_robot_model < matlab.System
    % untitled Add summary here
    %
    % This template includes the minimum set of functions required
    % to define a System object.

    % Public, tunable properties
    properties
        % robot
        % robot_type
    end

    % Pre-computed constants or internal states
    properties (Access = private)
        robot
    end

    methods (Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.robot = py.sdu_controllers.URRobotModel();
        end

        % function y = stepImpl(obj,u)
        %     % Implement algorithm. Calculate y as a function of input u and
        %     % internal states.
        %     y = u;
        % end
        function robot = stepImpl(obj)
            robot = obj.robot;
        end

        function [robot] = isOutputFixedSizeImpl(~)
            robot = true;
        end

        function resetImpl(obj)
            % Initialize / reset internal properties
        end
    end
end
