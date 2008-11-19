XACRO = `rospack find xacro`/xacro.py
URDF2GAZEBO = `rospack find gazebo_robot_description`/urdf2gazebo
MODEL_FILE = `rospack find gazebo_robot_description`/world

ROBOT = pr2_prototype1

$(ROBOT).model: $(ROBOT).xacro.xml
	$(XACRO) $^ > $^.xml
	$(URDF2GAZEBO) $^.xml $(MODEL_FILE)/$@

clean:
	$(RM) $(ROBOT).xml.expanded $(MODEL_FILE)/$(ROBOT).model
	$(RM) core.* Ogre.log
