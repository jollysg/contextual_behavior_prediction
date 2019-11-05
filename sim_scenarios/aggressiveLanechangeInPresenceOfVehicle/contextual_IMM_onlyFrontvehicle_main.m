% contextual IMM program

ctx_imm = SLContextualBehaviorPredIMM(Ts_bp);
ctx_imm.driverThresholds = [60; 60];
ctx_imm.driverTypes = [0.5 0.5];

    initial_front_car_distance = 100;

for i = 1:length(simtime)
    t = simtime(i);
    if t < 20
        front_car_distance = initial_front_car_distance - 3.33 * t;
    end
%     if simtime > 5
%         % update the prob transition matrix over here
%         amm.markov_transition_matrix = [0.99   0.005 0.005; ...
%             0.015    0.97   0.015; ...
%             0.015    0.015     0.97];
%     end
% 
%     if simtime > 5.1
%         % update the prob transition matrix over here
%         amm.markov_transition_matrix = [0.97   0.015 0.015; ...
%             0.015    0.97   0.015; ...
%             0.015    0.015     0.97];
%     end
    % mix initial states for the current cycle first
    
    no_of_filters = length(ctx_imm.elementalFilters);
%     tp_matrix = eye(no_of_filters);
    
    context = [100 100 100 100 100 100]';
    context(3) = front_car_distance;
    ctx_imm.extractContext(context);
    ctx_imm.gapAcceptancePolicy();
    
    ctx_imm.calculateBehaviorProbabilityTransitionMatrix();
%     tp_matrix = mat(1:4, 1:4);
    
    ctx_imm.mixEstimates();
    
    filter_traj(i).mark_trans_matrix = ctx_imm.markov_transition_matrix;
    filter_traj(i).mix_init_states = ctx_imm.mixed_init_state;
    % predict
    ctx_imm.predict(0);
    
    % correct and update probabilistic weights
    y_tilde = groundTruth(i).y_gt;
    ctx_imm.correct(y_tilde);
    
    % combined estimates
    [comb_x, comb_p] = ctx_imm.calculateCombinedEstimate();
    ctx_imm.driverUpdate();
    
    filter_traj(i).prenormalizedWts = ctx_imm.getFilterNonNormalizedWeights()';
    filter_traj(i).weights = ctx_imm.getFilterWeights()';
    filter_traj(i).likelihoods = ctx_imm.getFilterLikelihoods()';
    filter_traj(i).estimates = ctx_imm.getFilterEstimates();
    filter_traj(i).err = ctx_imm.getFilterErrors();
%     innovation = zeros(2, 1, 4);
%     innovation = err(1:2, 1, 1:4);
    filter_traj(i).driver_weights = ctx_imm.driverTypes';
    filter_traj(i).combined_estimates = comb_x;
    filter_traj(i).combined_p = comb_p;
    filter_traj(i).gapAcceptance = ctx_imm.gapAcceptance;
    filter_traj(i).context = context;
end

post_processing_plots;
