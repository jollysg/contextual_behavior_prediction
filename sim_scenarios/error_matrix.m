% test1 = out.comb_estimate.Data;
% test2 = out.groundTruth.Data;
% test = test1 - test2;
% err_amm = rms(test);
%  0.0287    0.0417    0.0219    0.0622

% AMM IMM process noise test with ST data
test1_imm = [imm_comb_estimate.Data(:,1) imm_comb_estimate.Data(:,3)];
test2_imm = ground_truth.Data;

% Custom signal AMM with reset logic -  0.0778    0.0735
% test1_imm = [gt_custom.Signal_1.Data(:) gt_custom.Signal_2.Data(:)];
% test2_imm = [comb_estimate.Data(:, 1) comb_estimate.Data(:,3)];

% Custom signal IMM -  0.0773    0.0257
% test1_imm = [out.groundTruth_imm.Signal_1.Data(:) out.groundTruth_imm.Signal_2.Data(:)];
% test2_imm = [out.imm_comb_est.Data(:,1) out.imm_comb_est.Data(:,3)];
test_imm = test1_imm - test2_imm;
err_imm = rms(test_imm)


