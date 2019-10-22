classdef VehicleParametersBicycleModel
    %default vehicle parameters
    properties
        %mass (Kgs)
        m
        %dist of front axle from CG (m)
        l_f
        %dis of rear axle from CG (m)
        l_r
        %Moment of Inertia (Kg-m2)
        Jz
        %Cornering stiffness front axle (Kg/rad) (~ 2 times the tires)
        C_f
        %cornering stiffness rear axle (Kg/rad) (~ 2 times the tires)
        C_r
    end
    
    methods
    end
end