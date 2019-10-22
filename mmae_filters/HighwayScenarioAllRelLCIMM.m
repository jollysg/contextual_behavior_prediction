classdef HighwayScenarioAllRelLCIMM < InteractiveMultiModelFilter
    methods
        function self = HighwayScenarioAllRelLCIMM(Ts)
            self@InteractiveMultiModelFilter(Ts);
            mm = ConstantVelocityMotionModel(Ts);

            % process noise covariance
            no_of_states = length(mm.states);
            self.no_of_states = no_of_states;
            Q = eye(no_of_states) * 0.01;
            % measurement noise covariance
            R = [0.0025 0; 0 0.0025];
                        
            % added const velocity motion model
            flt1 = XKalmanFilter(Ts,mm);
            flt1.updateNoiseStatistics(Q, R);
            %         amm.addElementalFilter(flt);
            
            maneuver_length = 50;
            lane_center_to_center_distance = 3.5;   %meters
            mm = LeftLaneChangeRelativeMotionModel(Ts,maneuver_length, lane_center_to_center_distance);
            flt2 = XKalmanFilter(Ts,mm);
            Ql = diag([diag(Q)' 1]);
            flt2.updateNoiseStatistics(Ql, R);
            %         amm.addElementalFilter(flt);
            
            mm = RightLaneChangeRelativeMotionModel(Ts,maneuver_length, lane_center_to_center_distance);
            flt3 = XKalmanFilter(Ts,mm);
            flt3.updateNoiseStatistics(Ql, R);
            %         amm.addElementalFilter(flt);
            
            self.elementalFilters = {flt1, flt2, flt3};
            self.resetFilterWeights();
            
            X0 = [0 10 0 0]';
            P0 = eye(length(X0))*0.001;
            self.setInitialConditions(X0, P0);
            
%             self.markov_transition_matrix = [0.9 0.05 0.05; ...
%                                             0.09 0.9  0.01; ...
%                                             0.09 0.01 0.9];

            self.markov_transition_matrix = [0.97 0.015 0.015; ...
                                            0.029 0.97  0.001; ...
                                            0.029 0.001 0.97];

%             self.markov_transition_matrix = [0.34 0.33 0.33; ...
%                                             0.399 0.6  0.001; ...
%                                             0.399 0.001 0.6];
%             wts = self.getFilterWeights();
%             self.normalizers = self.markov_transition_matrix' * wts';
              self.normalizers = zeros(length(self.elementalFilters), 1);
        end
        
        function setInitialConditions(self, X, P)
            for i = 1:length(self.elementalFilters)
                flt = self.elementalFilters{i};
                if i == 2 || i == 3
                    X_aug = [X;0];
                    P_aug = diag([diag(P)' 1]);
                    flt.setInitialConditions(X_aug,P_aug);
                else
                    flt.setInitialConditions(X, P);
                end
            end
        end        
        
    end
end