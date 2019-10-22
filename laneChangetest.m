
%%  
    % inputs
    X = [x vx y vy rel_x];
    P = eye(4) * P_init;
    u = 0;
    y_tilde = [1;0];
    wk_prev = 1;
    
    leftLCfilter = XKalmanFilter.empty();
    if isempty(leftLCfilter)
        maneuver_length = 50;  %meters
        lane_center_to_center_distance = 3.5;   %meters
        
        %TODO: See how can this be moved out of the function later
        %sampling time
        Ts = 0.1;
        
        mm = RightLaneChangeSinusoidalMotionModel(Ts,maneuver_length, lane_center_to_center_distance);
        
        leftLCfilter = XKalmanFilter(Ts,mm);
        
        % measurement noise covariance
        %     R = [0.2 0; 0 0.2];
        R = [0.0025 0; 0 0.0025];
        
        % process noise covariance
        %     Q = diag([0.5 5.0 0.5 5.0]);
        Q = diag([0.001 0.001 0.001 0.001]);
        
        leftLCfilter.updateNoiseStatistics(Q, R);

    end
%%
    % Now estimate states using kalman filter
    %predict
    [Xhat_minus, P_minus] = leftLCfilter.predict(u, X, P);
    
    %correct
    [X_hat_plus, P_plus] = leftLCfilter.correct(y_tilde);
    ek_minus = leftLCfilter.err_innov;    %residual
    ek_m_covariance = leftLCfilter.err_cov;    %residual covariance
    
    %calculate MMAE weights
    p_yk_xkminus = leftLCfilter.likelihood;
    wk = wk_prev * p_yk_xkminus;
   
