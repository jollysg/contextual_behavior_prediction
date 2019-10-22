classdef NonLinearMotionModel < MotionModel
    methods(Abstract)
        F = jacobian(obj, x, u)
    end
end