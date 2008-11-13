XACRO = `rospack find xacro`/xacro.py
URDF2GAZEBO = `rospack find gazebo_robot_description`/urdf2gazebo

ROBOT = pr2_new

$(ROBOT).model: $(ROBOT).xml
	$(XACRO) $^ > $^.expanded
	$(URDF2GAZEBO) $^.expanded $@

clean:
	$(RM) $(ROBOT).xml.expanded $(ROBOT).model
	$(RM) core.* Ogre.log

