# Start thump connecting ONLY right fdbk.
#sudo ~/thump/thumpnode/bin/thumpnode \
#    /thumpnode/feedback_pose_right:=/r_arm_cartesian_pose_controller/state/pose

# Start thump connecting right fdbk/cmd.
#sudo ~/thump/thumpnode/bin/thumpnode \
#    /thumpnode/feedback_pose_right:=/r_arm_cartesian_pose_controller/state/pose \
#    /thumpnode/command_pose_right:=/r_arm_cartesian_pose_controller/command

# Start thump connecting left/right fdbk/cmd.
sudo ~/thump/thumpnode/bin/thumpnode \
    /thumpnode/feedback_pose_left:=/l_arm_cartesian_pose_controller/state/pose \
    /thumpnode/feedback_pose_right:=/r_arm_cartesian_pose_controller/state/pose \
    /thumpnode/command_pose_left:=/l_arm_cartesian_pose_controller/command \
    /thumpnode/command_pose_right:=/r_arm_cartesian_pose_controller/command

#sudo ~/thump/thumpnode/bin/thumpnode \
#    /thumpnode/feedback_pose_left:=/l_arm_cartesian_pose_controller/command \
#    /thumpnode/feedback_pose_right:=/recv \
#    /thumpnode/command_pose_left:=/l_arm_cartesian_pose_controller/command \
#    /thumpnode/command_pose_right:=/send
