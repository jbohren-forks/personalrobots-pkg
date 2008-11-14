XACRO = `rospack find xacro`/xacro.py
URDF2GAZEBO = `rospack find gazebo_robot_description`/urdf2gazebo
MODEL_FILE = `rospack find gazebo_robot_description`/world

ROBOT = pr2

$(ROBOT).model: $(ROBOT).xml
	$(XACRO) $^ > $^.expanded
	$(URDF2GAZEBO) $^.expanded $(MODEL_FILE)/$@

# build a version without limits on joints
$(ROBOT)_nolimit.model: $(ROBOT).xml
	$(XACRO) $^ > $^.expanded
	$(URDF2GAZEBO) $^.expanded $(MODEL_FILE)/$@ 1

clean:
	$(RM) $(ROBOT).xml.expanded $(MODEL_FILE)/$(ROBOT).model $(MODEL_FILE)/$(ROBOT)_nolimit.model
	$(RM) core.* Ogre.log

