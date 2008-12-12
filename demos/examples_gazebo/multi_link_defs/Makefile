XACRO = `rospack find xacro`/xacro.py
URDF2GAZEBO = `rospack find gazebo_robot_description`/urdf2gazebo
MODEL_FILE = `rospack find gazebo_robot_description`/world

ROBOT = multi_link

$(ROBOT).model: $(ROBOT).xml
	$(XACRO) $^ > $^.expanded
	$(URDF2GAZEBO) $^.expanded $(MODEL_FILE)/$@

clean:
	$(RM) $(ROBOT).xml.expanded $(MODEL_FILE)/$(ROBOT).model
	$(RM) core.* Ogre.log

