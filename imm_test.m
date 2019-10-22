Ts = 0.1;
u = 0;
X = [0 10 0 0]';
P = eye(4);
y_tilde = [1 0]';

imm = InteractiveMultiModelFilter(Ts);


mm = ConstantVelocityMotionModel(Ts);
imm.markov_transition_matrix = [0.6 0.2 0.2; ...
                                0.1 0.9  0; ...
                                0.1  0  0.9];
% measurement noise covariance
R = [0.0025 0; 0 0.0025];

% process noise covariance
no_of_states = length(mm.states);
Q = eye(no_of_states) * 0.001;

% added const velocity motion model
flt = XKalmanFilter(Ts,mm);
flt.updateNoiseStatistics(Q, R);
imm.addElementalFilter(flt);

maneuver_length = 50;
lane_center_to_center_distance = 3.5;   %meters
mm = LeftLaneChangeSinusoidalMotionModel(Ts,maneuver_length, lane_center_to_center_distance);        
flt = XKalmanFilter(Ts,mm);
flt.updateNoiseStatistics(Q, R);
imm.addElementalFilter(flt);

mm = RightLaneChangeSinusoidalMotionModel(Ts,maneuver_length, lane_center_to_center_distance);
flt = XKalmanFilter(Ts,mm);
flt.updateNoiseStatistics(Q, R);
imm.addElementalFilter(flt);        

X0 = [0 10 0 0]';
P0 = eye(length(X0))*0.001;
imm.setInitialConditions(X0, P0);

wts = imm.getFilterWeights();
imm.normalizers = imm.markov_transition_matrix * wts';

imm.mixEstimates();

imm.predict(u);

imm.correct(y_tilde);

[comb_x, comb_p] = imm.calculateCombinedEstimate();

prenormalizedWts = imm.getFilterNonNormalizedWeights();
weights = imm.getFilterWeights();
likelihood = imm.getFilterLikelihoods();
estimates = imm.getFilterEstimates();

