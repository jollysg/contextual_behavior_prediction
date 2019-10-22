Ts = 0.1;
u = 0;
X = [0 10 0 0]';
P = eye(4);
y_tilde = [1 0]';

amm = AutonomousMultiModelFilter(Ts);

mm = ConstantVelocityMotionModel(Ts);

% measurement noise covariance
R = [0.0025 0; 0 0.0025];

% process noise covariance
no_of_states = length(mm.states);
Q = eye(no_of_states) * 0.001;

% added const velocity motion model
flt = XKalmanFilter(Ts,mm);
flt.updateNoiseStatistics(Q, R);
amm.addElementalFilter(flt);

maneuver_length = 50;
lane_center_to_center_distance = 3.5;   %meters
mm = LeftLaneChangeSinusoidalMotionModel(Ts,maneuver_length, lane_center_to_center_distance);        
flt = XKalmanFilter(Ts,mm);
flt.updateNoiseStatistics(Q, R);
amm.addElementalFilter(flt);

mm = RightLaneChangeSinusoidalMotionModel(Ts,maneuver_length, lane_center_to_center_distance);
flt = XKalmanFilter(Ts,mm);
flt.updateNoiseStatistics(Q, R);
amm.addElementalFilter(flt);        

X0 = [0 10 0 0]';
P0 = eye(length(X0))*0.001;
amm.setInitialConditions(X0, P0);

amm.predict(u, X, P);

amm.correct(y_tilde, X, P);

[comb_x, comb_p] = amm.calculateCombinedEstimate();

prenormalizedWts = amm.getFilterNonNormalizedWeights();
weights = amm.getFilterWeights();
likelihood = amm.getFilterLikelihoods();
estimates = amm.getFilterEstimates();