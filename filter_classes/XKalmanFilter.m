classdef XKalmanFilter < handle
    properties
        Ts
        predicted_state
        predicted_P
        
        estimatedState
        estimate_P
        % Measurement noise
        R
        % Process noise
        Q
        % Motion model
        mm
        % likelihood
        likelihood
        % error innovation
        err_innov
        % error covariance
        err_cov
    end
    
    methods
        function self = XKalmanFilter(Ts, MotionModel)
            if nargin == 0
                Ts = 0.1;
                MotionModel = LinearizedBicycleModel();
            elseif nargin == 1
                MotionModel = LinearizedBicycleModel(Ts);
            end
                
            self.Ts = Ts;
            self.mm = MotionModel;            
        end
        
        function self = updateNoiseStatistics(self, Q, R)
            self.Q = Q;
            self.R = R;
        end
        
        function [x_prop, p_prop] = predict(self, x, P, u)
            x_prop =  self.mm.propagate(x, u);
            
            % The first term in the covariance equation can be moved into
            % motion model for abstraction, but it will make the motion
            % model closely coupled with KF
            F = self.mm.linearizedDiscreteStateTransitionMatrix(x, u);
            p_prop = (F * P * F') + self.Ts*self.Q;

            self.predicted_state = x_prop;
            self.predicted_P = p_prop;
        end
        
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
            self.estimate_P = p_plus;
            
%             self.likelihood = 1/sqrt(det(2*pi*ek_m_covariance)) ...
%                 * exp(-1/2 * ek_minus'* inv(ek_m_covariance) * ek_minus);
            self.likelihood = 1/sqrt(det(2*pi*self.err_cov)) ...
                * exp(-1/2 * self.err_innov'* inv(self.err_cov) * self.err_innov);

        end
    end
end