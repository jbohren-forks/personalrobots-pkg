#
#
#  this make file is used to generate urdf xml and gazebo .model files for viewing purposes only, will be deprecated soon
#
#
XACRO = `rospack find xacro`/xacro.py
URDF2GAZEBO = `rospack find gazebo_robot_description`/urdf2gazebo
MODEL_FILE = `rospack find gazebo_robot_description`/world

ROBOT = pr2_gripper

$(ROBOT).model: $(ROBOT).xacro.xml
	$(XACRO) $^ > $(ROBOT).xml
	$(URDF2GAZEBO) $(ROBOT).xml $(MODEL_FILE)/$@

clean:
	$(RM) $(ROBOT).xml $(MODEL_FILE)/$(ROBOT).model
	$(RM) core.* Ogre.log

