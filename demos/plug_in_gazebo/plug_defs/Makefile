XACRO = `rospack find xacro`/xacro.py
URDF2GAZEBO = `rospack find gazebo_plugin`/urdf2file
ROBOT_PATH = `rospack find plug_in_gazebo`/robots

ROBOT = plug

$(ROBOT).model: $(ROBOT_PATH)/$(ROBOT).xacro.xml
	$(XACRO) $^ > $^.expanded.xml
	$(URDF2GAZEBO) $^.expanded.xml $(ROBOT_PATH)/$@

clean:
	$(RM) $(ROBOT).xml.expanded $(ROBOT_PATH)/$(ROBOT).model
	$(RM) core.* Ogre.log

