classdef XKalmanPredictor < XKalmanFilter
    properties
        predictions_state
        predictions_P
    end
    
    methods
        function self = XKalmanPredictor(Ts, motionmodel)
            self@XKalmanFilter(Ts, motionmodel);
            no_of_states = length(self.estimatedState);
            fsteps = 5/self.Ts;
            self.predictions_state = zeros([size(self.estimatedState), fsteps]);
            self.predictions_P = zeros([size(self.estimate_P), fsteps]);
 
        end
                       
        function [x_prop, p_prop] = predict(self, u, x, P)
            if nargin == 2
%                 Use the last propagated states for propagating further
                x = self.estimatedState;
                P = self.estimate_P;
            end
            
            no_of_states = length(self.estimatedState);
            
%             also predict for the next few time steps (4-5 secs)
            fsteps = 5/self.Ts;
            pred_X = zeros([size(x), fsteps]);
            pred_P = zeros([size(P), fsteps]);
            
            for i = 1:fsteps
                x = self.mm.propagate(x, u);

%                 The first term in the covariance equation can be moved
%                 into motion model for abstraction, but it will make the
%                 motion model closely coupled with KF
                F = self.mm.linearizedDiscreteStateTransitionMatrix(x, u);
                P = (F * P * F') + self.Ts*self.Q;
                pred_X(1:no_of_states,1, i) = x;
                pred_P(1:no_of_states,1:no_of_states, i) = P;
            end

            self.predicted_state = pred_X(1:no_of_states,1, 1);
            self.predicted_P = pred_P(1:no_of_states, 1:no_of_states, 1);
            self.predictions_state = pred_X;
            self.predictions_P = pred_P;
            x_prop = self.predicted_state;
            p_prop = self.predicted_P;
        end
 
%         function [x_prop, p_prop] = predict(self, u, x, P)
%             if nargin == 2
%                 % Use the last propagated states for propagating further
%                 x = self.predicted_state;
%                 P = self.predicted_P;
%             end
%             x_prop =  self.mm.propagate(x, u);
%             
%             % The first term in the covariance equation can be moved into
%             % motion model for abstraction, but it will make the motion
%             % model closely coupled with KF
%             F = self.mm.linearizedDiscreteStateTransitionMatrix(x, u);
%             p_prop = (F * P * F') + self.Ts*self.Q;
%             
%             %also predict for the next few time steps
% 
%             self.predicted_state = x_prop;
%             self.predicted_P = p_prop;
%         end
        
        function [x_plus, p_plus] = correct(self, y_tilde, x_prop, P_prop)
            if nargin == 2
                x_prop = self.predicted_state;
                P_prop = self.predicted_P;
            end
                       
            %correct
            yhat_minus = self.mm.estimatedMeasurement(x_prop, 0);
            self.err_innov = y_tilde - yhat_minus;    %residual
            
            % Following and the first term of error cov can be moved into
            % motion model, but it make the motion model coupled closely
            % with kalman filter and we wan't to keep that indepent.
            Ck = self.mm.getOutputMatrix();
            self.err_cov = Ck * P_prop *Ck' + self.R;    %residual covariance
            K = P_prop * Ck' * inv(self.err_cov);
            x_plus = x_prop + K * self.err_innov;
            p_plus = (eye(length(x_prop)) - K * Ck) * P_prop;
            self.estimatedState = x_plus;
            self.mm.setStates(x_plus);
            self.estimate_P = p_plus;
           
            % after correction, predicted and estimated states should be
            % the same.
            self.predicted_state = x_plus;
            self.predicted_P = p_plus;
            
%             self.likelihood = 1/sqrt(det(self.err_cov)*(2*pi)^length(y_tilde)) ...
%                 * exp(-1/2 * self.err_innov'* inv(self.err_cov) * self.err_innov);
            
            self.likelihood = 1/sqrt(det(2*pi*self.err_cov)) ...
                * exp(-1/2 * self.err_innov'* inv(self.err_cov) * self.err_innov);
%             self.nonNormalizedWeight = self.weight * self.likelihood;

        end
    end
end