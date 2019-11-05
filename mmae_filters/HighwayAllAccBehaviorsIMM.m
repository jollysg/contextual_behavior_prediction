classdef HighwayAllAccBehaviorsIMM < InteractiveMultiModelFilter
    methods
        function self = HighwayAllAccBehaviorsIMM(Ts)
            self@InteractiveMultiModelFilter(Ts);
            mm = ZeroAccelerationAndLateralVelMotionModel(Ts);

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
            mm = LeftLaneChangeRelativeMotionModelWithAcc(Ts,maneuver_length, lane_center_to_center_distance);
            flt2 = XKalmanFilter(Ts,mm);
            Ql = diag([diag(Q)' 1]);
            flt2.updateNoiseStatistics(Ql, R);
            
            maneuver_length = 100;
            mm = LeftLaneChangeRelativeMotionModelWithAcc(Ts,maneuver_length, lane_center_to_center_distance);
            flt3 = XKalmanFilter(Ts,mm);
            flt3.updateNoiseStatistics(Ql, R);
            
%             maneuver_length = 50;
%             lane_center_to_center_distance = 3.5;   %meters
%             mm = RightLaneChangeRelativeMotionModel(Ts,maneuver_length, lane_center_to_center_distance);
%             flt4 = XKalmanFilter(Ts,mm);
%             flt4.updateNoiseStatistics(Ql, R);
%             
%             maneuver_length = 100;
%             mm = RightLaneChangeRelativeMotionModel(Ts,maneuver_length, lane_center_to_center_distance);
%             flt5 = XKalmanFilter(Ts,mm);
%             flt5.updateNoiseStatistics(Ql, R);

            self.elementalFilters = {flt1, flt2, flt3};
            self.resetFilterWeights();
            
            X0 = [0 10 0 0 0 0]';
            P0 = eye(length(X0))*0.001;
            self.setInitialConditions(X0, P0);
            
%             self.markov_transition_matrix = [0.9 0.05 0.05; ...
%                                             0.09 0.9  0.01; ...
%                                             0.09 0.01 0.9];

            % filter order st, ll, ll
            self.markov_transition_matrix = [0.97 0.015 0.015; ...
                                            0.029  0.97 0.001; ...
                                            0.029 0.001 0.97];

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

        function [comb_est, comb_P] = calculateCombinedEstimate(self)
            comb_est = [0 0 0 0 0 0]';
            for i = 1: length(self.elementalFilters)
                flt = self.elementalFilters{i};
                estimate = flt.estimatedState(1:self.no_of_states);
                comb_est = comb_est + flt.weight * estimate;
            end
            self.combined_estimate = comb_est;
            
            % calculate combined covariance
            
            comb_P = zeros(6);
            for i = 1: length(self.elementalFilters)
                flt = self.elementalFilters{i};
                estimate = flt.estimatedState(1:self.no_of_states);
                est_P = flt.estimate_P(1:self.no_of_states, 1:self.no_of_states);
                err = estimate - comb_est;
                comb_P = comb_P + flt.weight * (err * err' + est_P);
            end
            self.P_combined_estimate = comb_P;
        end
        
%         function est_cov = getFilterEstimateCovariances(self)
%             num_filters = length(self.elementalFilters);
%             est_cov = zeros(self.no_of_states, self.no_of_states, num_filters);
%             for i = 1:num_filters
%                 flt = self.elementalFilters{i};
%                 est_cov(:,:, i) = flt.predicted_P(1:self.no_of_states, 1:self.no_of_states);
%             end
%         end
        
        function est = getFilterEstimates(self)
            num_filters = length(self.elementalFilters);
            est = zeros(6, 1, num_filters);
            for i = 1:num_filters
                flt = self.elementalFilters{i};
                est(:,:, i) = flt.predicted_state(1:self.no_of_states);
            end
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