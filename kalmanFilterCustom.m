function [X_hat_plus, P_plus] = kalmanFilterCustom(X, P, u, y_tilde)

    %sampling time
    Ts = 0.01;

    % measurement noise std deviation (for covariance)
    sigma_r = [0.2 0; 0 0.2];

    % process noise std deviation (for covariance)
%     sigma_qx = 0.5;
%     sigma_qy = 0.5;
%     sigma_qvx = 5.0;
%     sigma_qvy = 5.0;
    
    % process noise covariance
    Q = diag([0.5 5.0 0.5 5.0]);

    % states - X = [x vx y vy]
    Ak = [1 Ts 0 0; 0 1 0 0; 0 0 1 Ts; 0 0 0 1];
    Bk = 0;
    Ck = [1 0 0 0; 0 0 1 0];
    Dk = 0;
    
%     wk = sqrt(stime)* [sigma_q1 0; 0 sigma_q2] *randn(2,1); % this is not a Q matrix, 
%                                                     % this is sqr root of Q matrix
%     x(:, i+1) = Ak * x(:,i) + Bk * 0 + wk;
%     y(1, i+1) = sysC * x(:, i+1);

    %measurement will be available from outside
    % generate measurement
%     v = randn(1, 1);
%     ytilde(1, i+1) = y(1, i+1) + sigma_r*v;

    % Now estimate states using kalman filter
    
    %predict
    xhat_minus = Ak * X + Bk * u;
    P_minus = (Ak * P *Ak') + Ts*Q;
    yhat_minus = Ck * xhat_minus;

    %correct
    covariance_term = Ck * P_minus *Ck';
    inverse_term = inv( covariance_term + sigma_r^2);
    K = P_minus * Ck' * inverse_term;
    X_hat_plus = xhat_minus + K * (y_tilde - yhat_minus);
    P_plus = (eye(4) - K * Ck) * P_minus;
    
    %This will be likely outside
    %e = xhat_plus - x(:, i+1);
    
end