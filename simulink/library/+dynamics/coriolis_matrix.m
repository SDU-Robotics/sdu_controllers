classdef coriolis_matrix < matlab.System
    % Coriolis matrix
 
    properties

    end

    properties(Nontunable)
        robot_model
    end

    properties (Access = private)
        dof

        sdu_controllers
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.sdu_controllers = py.importlib.import_module('sdu_controllers');

            obj.dof = double(obj.robot_model.get_dof());

        end

        function [C] = stepImpl(obj, q, dq)

            q = reshape(q, 1, obj.dof);
            dq = reshape(dq, 1, obj.dof);

            C = obj.robot_model.get_coriolis(q, dq);
            C = reshape(double(C), obj.dof, obj.dof);
        end

        function C = isOutputFixedSizeImpl(~)
            C = true;
        end

        function resetImpl(obj)

        end

        function C = getOutputSizeImpl(obj)
            qsize = propagatedInputSize(obj, 1);
            C = [qsize(1), qsize(1)];
        end

        function C = getOutputDataTypeImpl(obj)

            C = "double";

        end

        function C = isOutputComplexImpl(obj)
            C = false;

        end

        function icon = getIconImpl(obj)
            icon = {'Coriolis Matrix', class(obj.robot_model)};
        end
    end
end
