classdef HighwayScenarioIMMForVehD < InteractiveMultiModelFilter
    methods
        function self = HighwayScenarioIMMForVehD(Ts)
            self@InteractiveMultiModelFilter(Ts);
            mm = ConstantLongVelocityStatLateralMotionModel(Ts);

            % process noise covariance
            no_of_states = length(mm.states);
            self.no_of_states = no_of_states;
            Q = eye(no_of_states) * 0.025;
%             Q = diag([0.025 1 0.025 1]);
            % measurement noise covariance
%             R = [0.0025 0; 0 0.0025];
            R = [0.0025 0; 0 0.0025];
                        
            % added const velocity motion model
            flt1 = XKalmanFilter(Ts,mm);
            flt1.updateNoiseStatistics(Q, R);
            %         amm.addElementalFilter(flt);
            
            maneuver_length = 60;
            lane_center_to_center_distance = 3.5;   %meters
            mm = LeftLaneChangeSinusoidalMotionModel(Ts,maneuver_length, lane_center_to_center_distance);
            flt2 = XKalmanFilter(Ts,mm);
            flt2.updateNoiseStatistics(Q, R);
            %         amm.addElementalFilter(flt);
            
            mm = RightLaneChangeSinusoidalMotionModel(Ts,maneuver_length, lane_center_to_center_distance);
            flt3 = XKalmanFilter(Ts,mm);
            flt3.updateNoiseStatistics(Q, R);
            %         amm.addElementalFilter(flt);
            
            self.elementalFilters = {flt1, flt2, flt3};
            self.resetFilterWeights();
            
            X0 = [0 10 0 0]';
            P0 = eye(length(X0))*0.001;
            self.setInitialConditions(X0, P0);
            
%             self.markov_transition_matrix = [0.99 0.005 0.005; ...
%                                             0.01 0.99  0; ...
%                                             0.01 0.0 0.99];
            self.markov_transition_matrix = [1 0 0; ...
                                            0 1 0; ...
                                            0 0 1];
% 
%             self.markov_transition_matrix = [0.97 0.015 0.015; ...
%                                             0.03 0.97  0; ...
%                                             0.03 0 0.97];

%             self.markov_transition_matrix = [0.34 0.33 0.33; ...
%                                             0.399 0.6  0.001; ...
%                                             0.399 0.001 0.6];
%             wts = self.getFilterWeights();
%             self.normalizers = self.markov_transition_matrix' * wts';
              self.normalizers = zeros(length(self.elementalFilters), 1);
        end
    end
end