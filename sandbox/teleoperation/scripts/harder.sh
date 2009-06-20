# Start thump connecting ONLY right fdbk.
#sudo ~/thump/thumpnode/bin/thumpnode /thumpnode/feedback_pose_right:=/r_arm_cartesian_pose_controller/state/pose

# Start thump connecting right fdbk and right cmd.
#sudo ~/thump/thumpnode/bin/thumpnode /thumpnode/feedback_pose_right:=/r_arm_cartesian_pose_controller/state/pose /thumpnode/command_pose_right:=/r_arm_cartesian_pose_controller/command


# Start CLUTCHING VERSION thump connecting right fdbk and right cmd.
#sudo ~/thump/thumpnode/bin/thumpnodec /thumpnode/feedback_pose_right:=/r_arm_cartesian_pose_controller/state/pose /thumpnode/command_pose_right:=/r_arm_cartesian_pose_controller/command


# Start SCALING VERSION thump connecting right fdbk and right cmd.
sudo ~/thump/thumpnode/bin/thumpnodes \
    /thumpnode/feedback_pose_left:=/l_arm_cartesian_pose_controller/state/pose \
    /thumpnode/feedback_pose_right:=/r_arm_cartesian_pose_controller/state/pose \
    /thumpnode/command_pose_left:=/l_arm_cartesian_pose_controller/command \
    /thumpnode/command_pose_right:=/r_arm_cartesian_pose_controller/command
