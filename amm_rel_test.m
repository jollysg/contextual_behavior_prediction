    amm = HighwayScenarioRelLCAMM.empty();
    
    if isempty(amm)
        Ts = 0.1;
        
        amm = HighwayScenarioRelLCAMM(Ts);
    end
    
    amm.predict(u);
    
    amm.correct(y_tilde);
    
    [comb_x, comb_p] = amm.calculateCombinedEstimate();
    
    prenormalizedWts = amm.getFilterNonNormalizedWeights();
    weights = amm.getFilterWeights();
    likelihood = amm.getFilterLikelihoods();
    estimates = amm.getFilterEstimates();
    