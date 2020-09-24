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

classdef InteractiveMultiModelFilter < AutonomousMultiModelFilter
    properties
        mixed_init_state
        mixed_init_state_cov
        markov_transition_matrix
        normalizers
    end
    
    methods
        function self = InteractiveMultiModelFilter(Ts)
            if nargin == 0
                Ts = 0.1;
            end
            self.Ts = Ts;
            self.markov_transition_matrix = eye(3);
%             self.elementalFilters = {};
        end
        
        function updateMarkovTransitionMatrix(self, Pij)
            self.markov_transition_matrix = Pij;
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
                flt.predict(u, X, P);
            end            
        end

        function updateWeights(self)
            normalizer = 0;
            for i = 1: length(self.elementalFilters)
                flt = self.elementalFilters{i};
                flt.nonNormalizedWeight = self.normalizers(i)* flt.likelihood;
                normalizer = normalizer + flt.nonNormalizedWeight;
            end
                
            for i = 1: length(self.elementalFilters)
                flt = self.elementalFilters{i};
                flt.weight = flt.nonNormalizedWeight/normalizer;
            end
        end        
        
        function mixEstimates(self)
            wts = self.getFilterWeights();
            no_wts = length(wts);
            mix_wts_matrix = zeros(no_wts);
            for i = 1:no_wts
                mix_wts_matrix(:,i) = wts' .* self.markov_transition_matrix(:,i);
            end
            % normalizers are basically the predicted probability of the
            % the system being in the mode i (i.e. filter i being the one
            % with higher likelihood). This is nothing but the weights of
            % all the filters in the last cycle multiplied by the markov
            % mode transition probabilities for transition to mode i. Since
            % this is the beginning of the current cycle, and usually
            % before the measurements have arrived, these are called as
            % predicted probabilities.
            normalizers = sum(mix_wts_matrix);
            
            for i = 1:no_wts
                for j = 1:length(normalizers)
                    if normalizers(j) > 1e-20
                        mix_wts_matrix(i,j) = mix_wts_matrix(i,j) ./ normalizers(j);
                    else
                        normalizers(j) = 0;
                        mix_wts_matrix(i,j) = 0;
                    end
                end
            end
            
            self.normalizers = normalizers';
            % mix wts matrix shows the conditional probabilities wij that
            % the filter will be in the state j, if it were in state i in
            % the last cycle.
            %mix_Wts_matrix = [ w11 w12 w13; 
            %                   w21 w22 w23;
            %                   w31 w32 w33];
            
            % eg xo_(j) = w_1j * x1 + w_2j * x2 + w_3j * x3;
            
            % number of init_states will be for number of filters, which is
            % the same as the number of weights we found above
            self.mixed_init_state = zeros(self.no_of_states, 1, no_wts);
            filterEstimates = self.getFilterEstimates();
            
            % mixed initial states
            for j = 1:no_wts
                mixed_state = zeros(self.no_of_states, 1);
                for i = 1:no_wts
                    % mixed_state = mixed_State + w_ij * xi
                    mixed_state = mixed_state + mix_wts_matrix(i,j) * filterEstimates(:,:,i);
                end
                self.mixed_init_state(:,:,j) = mixed_state;
            end
            
            % mixed initial state covariances
            self.mixed_init_state_cov = zeros(self.no_of_states, self.no_of_states, no_wts);
            covariances = self.getFilterEstimateCovariances();
            
            % mix initial state covariances
            for j = 1: no_wts
                % for every filter, calculate the covaraince influence due
                % to every other filter
                
                % init zeros for covariance first
                mixed_cov = zeros(self.no_of_states);
                for i = 1: no_wts
                    % iterate through the wts of every filter
                    
                    %relative error from the mixed_init_mean
                    err = filterEstimates(:,:,i) - self.mixed_init_state(:,:,j);
                    mixed_cov = mixed_cov + mix_wts_matrix(i,j) ...
                        * (covariances(:,:,i) + err * err');
                end
                self.mixed_init_state_cov(:,:,j) = mixed_cov;
            end
        end
        
    end
end