classdef SmallSedanBicycleModelParameters < VehicleParametersBicycleModel
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
        function obj = SmallSedanBicycleModelParameters()
            obj.m = 1150;
            obj.l_f = 1.084;
            obj.l_r = 1.596;
            obj.Jz = 1850;
            c_axle_front = 1270.6; % N/deg per axle
            c_axle_rear = 949.2;    %N / deg per axle
            obj.C_f = c_axle_front * 180/pi;    %72766 N/rad 
            obj.C_r = c_axle_rear * 180/pi;     %54395 N/rad
       end
    end

end