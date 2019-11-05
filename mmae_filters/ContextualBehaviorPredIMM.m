classdef ContextualBehaviorPredIMM < HighwayAllBehaviorsIMM
    properties
        % context vector = 6 distances around
        context
        % aggressive and passive driver probabilities [P(d1); P(d2)];
        driverTypes
        % driver thresholds for gaps (distances for now)
        driverThresholds
        % gap acceptance probabilities [P(G = 0); P(G = 1)]
        gapAcceptance
    end
    methods
        function self = ContextualBehaviorPredIMM(Ts)
            self@HighwayAllBehaviorsIMM(Ts);
            self.context = zeros(6,1);
            % start with 2 drivers - aggressive and passive by default
            self.driverTypes = [0.5, 0.5];
            self.thresholds = zeros(2, 1);
            self.gapAcceptance = [1; 0];
        end    
        
        function extractContext(self, participantStates)
            % This is the helper function for extracting the context.
            % PArticipant states should be the states of all the traffic
            % participants. The context vector needs to be extracted from
            % it. For now this can simply be context
            % context vector = 6 distances around
            self.context = participantStates;
        end
                
        function gap = gapAcceptanceDriverPolicy(self, threshold)
            c = self.context;
            %gap = [ P(G=0; P(G=1)]
            laneSatisfaction = (c(3) + c(4)) > threshold; % perhaps c(4) is not needed here
            leftLaneDissatisfaction = (c(1) + c(2)) <= threshold;
            rightLaneDissatisfaction = (c(5) + c(6)) <= threshold;
            gap_rejected = double(laneSatisfaction | (leftLaneDissatisfaction & rightLaneDissatisfaction));
            gap = [gap_rejected; 1 - gap_rejected];
        end
      
        function gapAcceptancePolicy(self)
            % aggressive
            d1 = self.driverTypes(1);
            % passive
            d2 = self.driverTypes(2);
            gapAcceptAgg = self.gapAcceptanceDriverPolicy(self.driverThresholds(1));
            gapAcceptPass = self.gapAcceptanceDriverPolicy(self.driverThresholds(2));
            self.gapAcceptance = d1 * gapAcceptAgg + d2 * gapAcceptPass;
            normalizer = sum(self.gapAcceptance);
            self.gapAcceptance = self.gapAcceptance/normalizer;    
        end
        
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
            
            beh_prob_trans_matrix = [g_a    0   0   0   g_na;
                                      0     g_a 0   0   g_na;
                                      0     0   g_a 0   g_na;
                                      0     0   0   g_a g_na;
                                      d1*ga*fll d2*ga*fll d1*ga*frl d2*ga*frl g_na];
                                  
            % Normalize the matrix
            normalizers = sum(beh_prob_trans_matrix');
            normalized_ptm = beh_prob_trans_matrix ./ normalizers';
            
            % Now can it be mapped directly to transition matrix?
            self.markov_transition_matrix = normalized_ptm;
        end
        
        function driverUpdate(self)
            % this should be called after updating the probabilistic
            % weights of all the behaviors (i.e. elemental filters)
            w = self.weights;
            d1 = w(1) + w(3) + self.driverTypes(1)*w(5);
            d2 = w(2) + w(4) + self.driverTypes(2)*w(5);
            updated_driver_weights = [d1;d2];
            normalizer = d1 + d2;
            self.driverTypes = updated_driver_weights/normalizer;
        end
    end
end