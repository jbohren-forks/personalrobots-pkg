all: gen_msgs
	ln -s `rospack find wg_robot_description`/pr2/cameras.xml .
clean: clean_msgs
	run cameras.xml
include $(shell rospack find mk)/msg.mk
