XACRO = `rospack find xacro`/xacro.py
URDF2GAZEBO = `rospack find gazebo_plugin`/bin/urdf2file
MODEL_DIR = `pwd`

ROBOT = table

$(ROBOT).model: $(ROBOT).xml
	$(XACRO) $^ > $^.expanded
	$(URDF2GAZEBO) $^.expanded $(MODEL_DIR)/$@

clean:
	$(RM) $(ROBOT).xml.expanded $(MODEL_DIR)/$(ROBOT).model
	$(RM) core.* Ogre.log

