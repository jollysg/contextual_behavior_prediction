% MIT License
%
% Copyright (c) 2020 Jasprit Singh Gill (jaspritsgill@gmail.com)
%
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:

% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.

% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.

classdef SLContextualBehaviorPredIMM < InteractiveMultiModelFilter
    properties
        % context vector = 6 distances around and 2 velocities adj lanes of
        % following vehicles
        context
        % aggressive and passive driver probabilities [P(d1); P(d2)];
        driverTypes
        % driver thresholds - current lane threshold, next lane dist
        % threshold and next lane velocity threshold representing diff
        % between self vel & follow veh velocity. Vf - Vr < -1 (for a
        % passive driver), Vf - Vr < 3 for aggressive driver. This means
        % that aggressive driver is willing to accept a lane change gap
        % even if the vehicle following in that lane has a higher speed to
        % cut in.
        driverThresholds
        % gap acceptance probabilities [P(G = 0); P(G = 1)]
        gapAcceptance
        driver1GapAcceptance
        driver2GapAcceptance
        max_no_of_states
        no_of_models
        prediction_interval

    end
    methods
        function self = SLContextualBehaviorPredIMM(Ts)
            self@InteractiveMultiModelFilter(Ts);
            mm = ZeroAccelerationAndLateralVelMotionModel(Ts);

            % process noise covariance
            no_of_states = length(mm.states);
            self.no_of_states = no_of_states;
            Q = eye(no_of_states) * 0.01;

            % measurement noise covariance
            R = [0.0025 0; 0 0.0025];
%               R = [0.0064 0; 0 0.0064];
%             R = [0.01 0; 0 0.01];

            % added const velocity motion model
            flt1 = XKalmanPredictor(Ts,mm);
            flt1.updateNoiseStatistics(Q, R);

            mm = ConstantAccelerationZeroLateralVelMotionModel(Ts);
            flt2 = XKalmanPredictor(Ts,mm);
            flt2.updateNoiseStatistics(Q, R);
    
            maneuver_length = 100;
            lane_center_to_center_distance = 3.5;   %meters
            mm = LeftLaneChangeRelativeMotionModelWithAcc(Ts,maneuver_length, lane_center_to_center_distance);
            flt3 = XKalmanPredictor(Ts,mm);
            Ql = diag([diag(Q)' 1]);
            flt3.updateNoiseStatistics(Ql, R);            

            maneuver_length = 70;
            mm = LeftLaneChangeRelativeMotionModelWithAcc(Ts,maneuver_length, lane_center_to_center_distance);
            flt4 = XKalmanPredictor(Ts,mm);
            flt4.updateNoiseStatistics(Ql, R);

            self.elementalFilters = {flt1, flt2, flt3, flt4};
            self.resetFilterWeights();
            
            X0 = [0 10 0 0 0 0]';
            P0 = eye(length(X0))*0.001;
            self.setInitialConditions(X0, P0);
            
            % filter order st, acc, ll
            self.markov_transition_matrix = [0.97 0.01 0.01 0.01; ...
                                            0.001 0.97 0.001 0.028; ...
                                            0.028 0.001 0.97 0.001; ...
                                            0.028 0.001 0.001 0.97];

            self.normalizers = zeros(length(self.elementalFilters), 1);
            self.context = zeros(6,1);
            
            % start with 2 drivers - aggressive and passive by default
            self.driverTypes = [0.5, 0.5];
            self.driverThresholds = zeros(2, 3);
            self.gapAcceptance = [1; 0];
            self.driver1GapAcceptance = [1;0];
            self.driver2GapAcceptance = [1;0];
            
            self.no_of_models = length(self.elementalFilters);
              max_states = 0;
              for i = 1:length(self.elementalFilters)
                  flt = self.elementalFilters{i};
                  if max_states < flt.no_of_states
                      max_states = flt.no_of_states;
                  end
              end
              self.max_no_of_states = max_states;
              self.prediction_interval = 5;              

        end   
        
        function pred = getPredictions(self)
           no_of_predictions = self.prediction_interval/self.Ts; % 5 / Ts
           % no_of_states, time_steps, no_of_filters
           pred = zeros(self.max_no_of_states,no_of_predictions,self.no_of_models);
           for i = 1: self.no_of_models
               flt = self.elementalFilters{i};
               test = squeeze(flt.predictions_state);
               pred(1:flt.no_of_states, 1:no_of_predictions, i) ...
                   = test(1:flt.no_of_states, 1:no_of_predictions);
           end
       end

        
        function setInitialConditions(self, X, P)
            for i = 1:length(self.elementalFilters)
                flt = self.elementalFilters{i};
                if i == 3 || i == 4
                    X_aug = [X;0];
                    P_aug = diag([diag(P)' 1]);
                    flt.setInitialConditions(X_aug,P_aug);
                else
                    flt.setInitialConditions(X, P);
                end
            end
        end
        
        function extractContext(self, participantStates)
            % This is the helper function for extracting the context.
            % PArticipant states should be the states of all the traffic
            % participants. The context vector needs to be extracted from
            % it. For now this can simply be context
            % context vector = 6 distances around
            self.context = participantStates;
        end
                
        function gap = gapAcceptanceDriverPolicy(self, threshold, c)
            %gap = [ P(G=0; P(G=1)]
            self_vel = self.combined_estimate(2);
            % perhaps c(4) is not needed here, threshold(1) is current lane
            % threshold
            laneSatisfaction = (c(3)) > threshold(1);
            % threshold(2) is adjoining lane rear distance threshold.
            % threshold(3) is diff in velocity with the foll vehicle
            % threshold
            leftLaneDissatisfaction = (abs(c(2)) <= threshold(2)) ...
                                | ((c(2) > threshold(2)) & (c(7)-self_vel) >= threshold(3));
            
            %For now since we do not have a right lane, lets keep this as
            %1, i.e condition dissatisfied. So gap will be rejected if
            %left lane gap requirement is also dissatisfied. If we do not
            %assume this for the right lane, then if the lanesatisfaction
            %is 0, then the rightlane dissatisfaction will never be false
            %as there are no vehicles over there, and hence the gap will
            %always be accepted, even if there is a vehicle in the left
            %lane.
            % TODO: uncomment the following line in multilane scenarios.
%             rightLaneDissatisfaction = (c(5) + c(6)) <= threshold;
             rightLaneDissatisfaction = true;
            gap_rejected = double(laneSatisfaction | (leftLaneDissatisfaction & rightLaneDissatisfaction));
            gap = [gap_rejected; 1 - gap_rejected];
        end
      
        function gapAcceptancePolicy(self, c)
            if nargin == 1
                c = self.context;
            end
            % aggressive
            d1 = self.driverTypes(1);
            % passive
            d2 = self.driverTypes(2);
            gapAcceptAgg = self.gapAcceptanceDriverPolicy(self.driverThresholds(1,:), c);
            gapAcceptPass = self.gapAcceptanceDriverPolicy(self.driverThresholds(2,:), c);
            self.gapAcceptance = d1 * gapAcceptAgg + d2 * gapAcceptPass;
            normalizer = sum(self.gapAcceptance);
            self.gapAcceptance = self.gapAcceptance/normalizer;    
            self.driver1GapAcceptance = gapAcceptAgg;
            self.driver2GapAcceptance = gapAcceptPass;
        end
        
        % Following is needed only if you are choosing bet left and right
        function prob = leftLaneChangeManeuverFunction(self)
            c = self.context;
            num = c(1) + c(2);
            den = num + c(5) + c(6);
            % return c1 + c2 / (c1 + c2 + c5 + c6);
            prob = num/den;
        end
        
        function prob = rightLaneChangeManeuverFunction(self)
            c = self.context;
            num = c(5) + c(6);
            den = num + c(1) + c(2);
            % return c5 + c6 / (c1 + c2 + c5 + c6);
            prob = num/den;
        end
        
        function calculateBehaviorProbabilityTransitionMatrix(self)
            d1 = self.driverTypes(1);
            d2 = self.driverTypes(2);
            g_na = self.gapAcceptance(1);
            g_a = self.gapAcceptance(2);
            
            if (g_na < 0.1)
                % If g_na is very small, then add 0.01 to it, as there is 1
                % in 100 chance (assuming about 10 secs of maneuver length)
                % that the maneuver will be complete and the participant
                % will switch to default maneuver. If g_na is bigger than
                % 0.1, then the influence of this chance can be neglected.
                % Of course, if g_na is adjusted by 0.01, g_a will have to
                % adjusted too
                g_na = g_na + 0.01;
                g_a = 1 - g_na;                
            end
            
            den = g_na + d2*g_a + d1*g_a;
            beh_prob_trans_matrix = [0.97         0.03*g_na/den  0.03*d2*g_a/den       0.03*d1*g_a/den;
                                    (1-0.97)*g_na  0.97      0   (1-0.97)*g_a;
                                       0.03     0       0.97         0;
                                       0    0.03        0          0.97];
                                              
            % Normalize the matrix
            normalizers = sum(beh_prob_trans_matrix');
            n = length(self.elementalFilters);
            normalized_ptm = zeros(size(beh_prob_trans_matrix));
            for i = 1: n
                normalized_ptm(i,:) = beh_prob_trans_matrix(i,:) ./ normalizers(i);
            end
            
            self.markov_transition_matrix = normalized_ptm;
        end
        
        function driverUpdate(self)
            % this should be called after updating the probabilistic
            % weights of all the behaviors (i.e. elemental filters)
            w = self.getFilterWeights();
            d1 = w(2) + w(4);
            d2 = w(1) + w(3);
            updated_driver_weights = [d1 d2];
            normalizer = d1 + d2;
            self.driverTypes = updated_driver_weights/normalizer;
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