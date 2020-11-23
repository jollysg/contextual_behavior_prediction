# Interaction-Aware Probabilistic Framework for Autonomous Vehicles to Predict Behaviors and Driving Policy of Traffic Participants 

This work provides a proof-of-concept for an interaction-aware behavior prediction framework for autonomous vehicles that not only predicts the behaviors of the traffic participants but also classifies the driving policy that can be associated with them. The work was developed as a part of following PhD Dissertation.

**Title**: Probabilistic Framework for Behavior Characterization of Traffic Participants Enabling Long Term Prediction

**Abstract**: This research aims at developing new methods that predict the behaviors of the human driven traffic participants, enabling safe operation of autonomous vehicles in complex traffic environments. Autonomous vehicles are expected to operate amongst human driven conventional vehicles in the traffic at least for the next few decades. For safe navigation they will need to infer the intents as well as the behaviors of the human traffic participants using extrinsically observable information. Doing so aides in predicting their trajectories for a time horizon long enough to analyze imminent risks and gracefully avert any risky situation. This work approaches above challenge by recognizing that any maneuver performed by a human driver can be divided into four stages that depend on the surrounding context: intent determination, maneuver preparation, gap acceptance and maneuver execution. It builds on the hypothesis that for a given driver, the behavior not only spans across these four maneuver stages, but across multiple maneuvers. As a result, identifying the driver behavior in any of these stages can help characterize the nature of all the subsequent maneuvers that the driver is likely to perform, thus resulting in a more accurate prediction for a longer time horizon. To enable this, a novel probabilistic framework is realized that couples the different maneuver stages of the observed traffic participant together and associates them to a driving style. The framework is implemented by modeling a traffic participant as a hybrid system. The contextual information of the observed traffic participant is utilized in extending t he Interacting Multiple Model (IMM) and classification of the driving style of the traffic participant (behavior characterization) is demonstrated for two use case scenarios in this repository. The developed Contextual IMM (CIMM) framework shows improvements in the performance of the behavior classification of the traffic participants compared to the IMM for the identified use case scenarios. Further, it also estimates the driving style of a traffic participant. This outcome warrants strong potential of this approach for autonomous vehicles on public roads.

For further details, please refer to the [PhD dissertation](https://tigerprints.clemson.edu/all_dissertations/2509) detailing this work.

## Citation: 
Gill, Jasprit Singh, "Probabilistic Framework for Behavior Characterization of Traffic Participants Enabling Long Term Prediction" (2019). All Dissertations. 2509.
https://tigerprints.clemson.edu/all_dissertations/2509

## Framework demonstration
The framework allows modeling different driver policies ( aggressive, passive, cooperative, non-cooperative, etc.) and classifies which policy the observed traffic participant is driving with. It also utilizes this information in predicting their future behaviors. To model driver policies, the framework provides hooks for:

1) the gap acceptance criteria for different driver policies
2) the maneuver selection criteria for different driver policies

The provided source code demonstrates how to set up the framework with two different driver policies (aggressive and passive driver policy) and use it to run inference on a traffic participant. Click on the images below to see the animations generated from the matlab source.

#### Aggressive Driver scenario:
[![Aggressive Drive Scenario](https://github.com/jollysg/contextual_behavior_prediction/blob/master/readme_images/aggressive_driver_scenario.png)](https://www.youtube.com/embed/viispI7NazM)

#### Passive Driver scenario: 
[![Passive Drive Scenario](https://github.com/jollysg/contextual_behavior_prediction/blob/master/readme_images/passive_driver_scenario.png)](https://www.youtube.com/embed/KN_oYHsRDd4)

The simulated video shows 3 subplots:
1. Participant Trajectories: This subplot shows the trajectories followed by the 3 participants chosen for the 2 lane driving scenario. Y coordinate of 0 represents the center of the right lane and 3.5m represents the center of the left lane. The observed vehicle is in the right lane (bottom). Red line represents the trajectory tracked (i.e. current and past positions) by the observed vehicle and yellow represents the predicted trajectory for it at any time instant. The predictions take into account the probabilities of the behavior type as well as the driver types identified for it at any instant. Purple line represents the preceeding vehicle position in front of the observed vehicle in the right lane. Green line represents the position of the another traffic participant in the left lane. 

2. Behavior Probabilities: This subplot shows the probabilistic measures of the behavior identified for the observed vehicle by the behavior prediction model after taking into account the vehicle measurements and the context around it. Four behaviors were considered for this scenario - a passive straight, an aggressive straight (with heavy acceleration), short aggressive lane change and long passive lane change.

3. Driver type weights vs time: This subplot shows the probabilitic measures of the driver type detected for the observed vehicle by the behavior prediction model after taking into account the vehicle measurements and the context around it. 

## Source Code Instructions
At present, the developed model is in MATLAB source files. The source for C++/Python is in works and will be available soon. 


1. To ensure all the relevant files are included in the MATLAB path, run the initialization.m file from the root folder.
2. The entry point for the source is in sim_scenarios folder. 
- Run the 'lane_change_noisy_measurements_generator.m' file in the sim_scenarios folder. It generates the ground truth for the simulated scenarios and calls the contextual_IMM_main.m file. To aggressive or passive driver scenarios can be switched using the 'aggressive_driver_use_case' flag in this file.
- contextual_IMM_main.m runs the predictions for the scenarios modeled and calls post_processing_plots.m.
- post_processing_plots.m generates the plots for the prediction model.
3. Finally, run pred_post_processing_matlab_ws_x_coord_mod.m for animations of the scenario.
4. [SLContextualBehaviorPredIMM class](https://github.com/jollysg/contextual_behavior_prediction/blob/master/mmae_filters/SLContextualBehaviorPredIMM.m) demonstrates how to set up the scenario and motionmodels for behavior prediction.
5. To experiment with different motion models or customize to your own scenario, a good way to start is to make a copy of SLContextualBehaviorPredIMM class and modify it.
6. Some sample motion models are provided in the [motionModelClasses](https://github.com/jollysg/contextual_behavior_prediction/tree/master/motionModelClasses) folder. To experiment with your own motion models, simply inherit from either the [MotionModel](https://github.com/jollysg/contextual_behavior_prediction/blob/master/motionModelClasses/MotionModel.m) class or [NonLinearMotionModel](https://github.com/jollysg/contextual_behavior_prediction/blob/master/motionModelClasses/NonLinearMotionModel.m) class.
7. The code also provides sources for Linear as well as extended kalman filter ([XKalmanFilter](https://github.com/jollysg/contextual_behavior_prediction/tree/master/filter_classes)), Autonomous Multiple Model (AMM) filter and Interactive-Multiple Model filter (IMM) in the [mmae_filters](https://github.com/jollysg/contextual_behavior_prediction/tree/master/mmae_filters) folder.

Queries: jaspritsgill@gmail.com

To learn more about my work, please visit my website: https://www.jaspritsgill.com/
