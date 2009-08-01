
# Used to descibe a fully calibrated robot after running the calibration

float64[] arm_initial_params
geometry_msgs/Transform arm_initial
LinkParamsDH[] arm_chain

geometry_msgs/Transform head_initial
LinkParamsDH[] head_chain
geometry_msgs/Transform stereo_final
float64[] stereo_final_params

geometry_msgs/Transform high_res_final
float64[] high_res_final_params