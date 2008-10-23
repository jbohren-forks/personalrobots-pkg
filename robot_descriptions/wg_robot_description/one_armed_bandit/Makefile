XACRO = `rospack find xacro`/xacro.py
URDF2GAZEBO = `rospack find gazebo_robot_description`/urdf2gazebo

ROBOT = one_armed_bandit

$(ROBOT).model: $(ROBOT).xml
	$(XACRO) $^ > $^.expanded
	$(URDF2GAZEBO) $^.expanded $@
	$(RM) $^.expanded
	sed -e 's#gazebo_joints\.xml#../pr2_robot_defs/gazebo.xml#' < $@ > $@.temp
	mv $@.temp $@

clean:
	$(RM) $(ROBOT).model
	$(RM) core.* Ogre.log

