classdef MeasurementModel < handle
    properties(Abstract)
        %output states
        output_states
        
    end
    
    methods(Abstract)
        y_hat = estimatedMeasurement(obj, x_minus)
    end
end