
# Used to descibe a fully calibrated robot after running the calibration

float64[] arm_initial_params
robot_msgs/Transform arm_initial
LinkParamsDH[] arm_chain

robot_msgs/Transform head_initial
LinkParamsDH[] head_chain
robot_msgs/Transform stereo_final
float64[] stereo_final_params

robot_msgs/Transform high_res_final
float64[] high_res_final_params