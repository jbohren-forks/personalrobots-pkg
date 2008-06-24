INSTALLATION

1.Follow the instructions for installation from the wiki at:
	 http://pr.willowgarage.com/wiki/ROS/Installation
2.Build ros using make
3.Type "rosmake roscpp"
4.Install jam by using: "sudo apt-get install jam"
5.Type: "cd $ROS_PACKAGE_PATH"
6.Type: "source exec_trex/config"
7.Type: "rosmake exec_trex"
8.Type: "rosmake teleop_base_keyboard"

RUNNING

1. Open four console windows.
2. In the first type: "botherder"
3. In the second type: "cd $ROS_PACKAGE_PATH/exec_trex && roslaunch Launch.xml"
4. In the third type: "cd $ROS_PACKAGE_PATH/nav/teleop_base_keyboard && ./teleop_base_keyboard"
5. Select the nav view window and hold shift.
6. Click the middle mouse button in a large white area. (While holding shift)
7. Click the middle mouse button in a different spot to set the direction. (While holding shift)
8. Press the I key and then the space bar to move the robot so it can localize. 
9. In the fourth consle window, type: "cd $ROS_PACKAGE_PATH/exec_trex && ./start_trex_demo"
10. The robot should move to a few goal points and then stop.

