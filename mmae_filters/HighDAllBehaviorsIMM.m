classdef HighDAllBehaviorsIMM < InteractiveMultiModelFilter
    methods
        function self = HighDAllBehaviorsIMM(Ts)
            self@InteractiveMultiModelFilter(Ts);
            mm = ConstantLongVelocityStatLateralMotionModel(Ts);

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

            % Lane coordinates
            % upperlanes = [8.5100   12.5900   16.4300];
            % lowerlanes = [21.0000   24.9600   28.8000];
            %lowerlaneMidpoints = 

            %X0 = [0 10 0 0]';
            % for veh 167 HighD
            X0 = [6.005 36.38 22.64 0]';
            P0 = eye(length(X0))*0.001;

            maneuver_length1 = 200;
            maneuver_length2 = 300;
            %lane_center_to_center_distance = 3.5;   %meters
            lane_center_to_center_distance = 26.88 - X0(3);
            mm = LeftLaneChangeRelativeMotionModel(Ts,maneuver_length1, lane_center_to_center_distance);
            flt2 = XKalmanFilter(Ts,mm);
            Ql = diag([diag(Q)' 1]);
            flt2.updateNoiseStatistics(Ql, R);
            
            mm = LeftLaneChangeRelativeMotionModel(Ts,maneuver_length2, lane_center_to_center_distance);
            flt3 = XKalmanFilter(Ts,mm);
            flt3.updateNoiseStatistics(Ql, R);
            
%             lane_center_to_center_distance = 3.5;   %meters
            mm = RightLaneChangeRelativeMotionModel(Ts,maneuver_length1, lane_center_to_center_distance);
            flt4 = XKalmanFilter(Ts,mm);
            flt4.updateNoiseStatistics(Ql, R);
            
            mm = RightLaneChangeRelativeMotionModel(Ts,maneuver_length2, lane_center_to_center_distance);
            flt5 = XKalmanFilter(Ts,mm);
            flt5.updateNoiseStatistics(Ql, R);

            self.elementalFilters = {flt1, flt2, flt3, flt4, flt5};
            self.resetFilterWeights();
            
            self.setInitialConditions(X0, P0);
            
%             self.markov_transition_matrix = [0.9 0.05 0.05; ...
%                                             0.09 0.9  0.01; ...
%                                             0.09 0.01 0.9];

            % filter order st, ll, ll, rl rl
            self.markov_transition_matrix = [0.97 0.0075 0.0075 0.0075 0.0075; ...
                                            0.027  0.97   0.001   0.001   0.001; ...
                                            0.027 0.001    0.97   0.001   0.001;
                                            0.027 0.001   0.001    0.97   0.001;
                                            0.027 0.001   0.001   0.001     0.97];

%             self.markov_transition_matrix = [0.97 0.0075 0.0075 0.0075 0.0075; ...
%                                             0.03  0.97   0.00   0.00   0.00; ...
%                                             0.03 0.00    0.97   0.00   0.00;
%                                             0.03 0.00   0.00    0.97   0.00;
%                                             0.03 0.00   0.00   0.00     0.97];

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
                if i ~= 1
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