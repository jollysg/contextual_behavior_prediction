classdef MeasurementModel < handle
    properties
        %output states
        output_states
        
    end
    
    methods
        function y_hat = estimatedMeasurement(obj, x_minus, u)
            % x_minus should typically be the predicted estimate, a column
            % vector
            y_hat = obj.Cd_matrix * x_minus + obj.Dd_matrix * u;
        end

    end
end