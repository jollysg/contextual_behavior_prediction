classdef StraightBehaviorsIMM < InteractiveMultiModelFilter
    methods
        function self = StraightBehaviorsIMM(Ts)
            self@InteractiveMultiModelFilter(Ts);
            mm = ZeroAccelerationMotionModel(Ts);

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
            
            mm = ConstantAccelerationMotionModel(Ts);
            flt2 = XKalmanFilter(Ts,mm);
            flt2.updateNoiseStatistics(Q, R);
                        
            mm = ConstantAccelerationMotionModel(Ts);
            flt3 = XKalmanFilter(Ts,mm);
            flt3.updateNoiseStatistics(Q, R);

            self.elementalFilters = {flt1, flt2, flt3};
            self.resetFilterWeights();
 
            X0 = [0 10 0 0 0 0]';
            P0 = eye(length(X0))*0.001;
            self.setInitialConditions(X0, P0);
            
            self.markov_transition_matrix = [0.97   0.015 0.015; ...
                                            0.015    0.97   0.015; ...
                                            0.015    0.015     0.97];

              self.normalizers = zeros(length(self.elementalFilters), 1);
        end

        function predict(self, u, X, P)
            if nargin == 2
                % TODO: Figure out what to do here, following is not
                % needed, but is only to preallocate the X and P matrices.
                % There are better ways to do this.
                X = self.mixed_init_state(:,:,1);
                P = self.mixed_init_state_cov(:,:,1);
            end
            % mixing happens here
            for i = 1: length(self.elementalFilters)
                flt = self.elementalFilters{i};
                X = flt.predicted_state;
                P = flt.predicted_P;
                X(1:self.no_of_states) = self.mixed_init_state(:,:,i);
                P(1:self.no_of_states, 1:self.no_of_states) = self.mixed_init_state_cov(:,:,i);
                if i == 2
                    inp = 1;
                elseif i == 3
                    inp = 2
                else
                    inp = u;
                end
                flt.predict(inp, X, P);
            end            
        end
        
%         function setInitialConditions(self, X, P)
%             for i = 1:length(self.elementalFilters)
%                 flt = self.elementalFilters{i};
%                 if i ~= 1
%                     X_aug = [X;0];
%                     P_aug = diag([diag(P)' 1]);
%                     flt.setInitialConditions(X_aug,P_aug);
%                 else
%                     flt.setInitialConditions(X, P);
%                 end
%             end
%         end        
%   
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
        
        function est = getFilterEstimates(self)
            num_filters = length(self.elementalFilters);
            est = zeros(6, 1, num_filters);
            for i = 1:num_filters
                flt = self.elementalFilters{i};
                est(:,:, i) = flt.predicted_state(1:self.no_of_states);
            end
        end

    end
end