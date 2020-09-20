Research models for Jasprit

# behavior_prediction
research models developed

The entry point is in Sim Scenarios folder:

1. Run the 'lane_change_noisy_measurements_generator.m' or the 'lane_change_measurements_generator.m' file. It generates the ground truth and calls the contextual_IMM_main.m file.
2. contextual_IMM_main.m runs the predictions for the synthetic measurements and calls post_processing_plots.m.
3. pred_post_processing_matlab_ws_x_coord_mod.m generates the animations for the scenario plot.

