# Probabilistic Framework for Behavior Characterization of Traffic Participants Enabling Long Term Prediction
This research aims at developing new methods that predict the behaviors of the human driven traffic participants, enabling safe operation of autonomous vehicles in complex traffic environments. Autonomous vehicles are expected to operate amongst human driven conventional vehicles in the traffic at least for the next few decades. For safe navigation they will need to infer the intents as well as the behaviors of the human traffic participants using extrinsically observable information. Doing so aides in predicting their trajectories for a time horizon long enough to analyze imminent risks and gracefully avert any risky situation. This work approaches above challenge by recognizing that any maneuver performed by a human driver can be divided into four stages that depend on the surrounding context: intent determination, maneuver preparation, gap acceptance and maneuver execution. It builds on the hypothesis that for a given driver, the behavior not only spans across these four maneuver stages, but across multiple maneuvers. As a result, identifying the driver behavior in any of these stages can help characterize the nature of all the subsequent maneuvers that the driver is likely to perform, thus resulting in a more accurate prediction for a longer time horizon. To enable this, a novel probabilistic framework is realized that couples the different maneuver stages of the observed traffic participant together and associates them to a driving style. The framework is implemented by modeling a traffic participant as a hybrid system. The contextual information of the observed traffic participant is utilized in extending t he Interacting Multiple Model (IMM) and classification of the driving style of the traffic participant (behavior characterization) is demonstrated for two use case scenarios in this repository. The developed Contextual IMM (CIMM) framework shows improvements in the performance of the behavior classification of the traffic participants compared to the IMM for the identified use case scenarios. Further, it also estimates the driving style of a traffic participant. This outcome warrants strong potential of this approach for autonomous vehicles on public roads.

For further details, please refer to the [PhD dissertation](https://tigerprints.clemson.edu/all_dissertations/2509) detailing this work.

## Citation: 
Gill, Jasprit Singh, "Probabilistic Framework for Behavior Characterization of Traffic Participants Enabling Long Term Prediction" (2019). All Dissertations. 2509.
https://tigerprints.clemson.edu/all_dissertations/2509

## Source Code Instructions
At present, the developed model is in MATLAB source files. The source for C++/Python is in works and will be available soon. 


1. To ensure all the relevant files are included in the MATLAB path, run the initialization.m file from the root folder.
2. The entry point for the source is in Sim Scenarios folder. 
- Run the 'lane_change_noisy_measurements_generator.m' file in the sim_scenarios folder. It generates the ground truth for the simulated scenarios and calls the contextual_IMM_main.m file. To aggressive or passive driver scenarios can be switched using the 'aggressive_driver_use_case' flag in this file.
- contextual_IMM_main.m runs the predictions for the scenarios modeled and calls post_processing_plots.m.
- post_processing_plots.m generates the plots for the prediction model.
3. Finally, run pred_post_processing_matlab_ws_x_coord_mod.m for animations of the scenario.

