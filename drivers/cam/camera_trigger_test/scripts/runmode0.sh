#! /bin/bash
MODES="752x480x15   
       752x480x12.5 
       640x480x30   
       640x480x25   
       640x480x15   
       640x480x12.5 
       320x240x60   
       320x240x50   
       320x240x30   
       320x240x25"   

for mode in $MODES
do
cat > data/testmode0-$mode.launch << EOF
<launch>
  <!--node machine="xenomai_root" pkg="pr2_etherCAT" type="pr2_etherCAT" args="-i eth0 -x /robotdesc/pr2"/-->
  <node pkg="pr2_etherCAT" type="pr2_etherCAT" args="-i eth0 -x \$(find camera_trigger_test)/test_configuration.xml" output="screen"/>
  
  <node pkg="mechanism_control" type="spawner.py" args="\$(find camera_trigger_test)/test_controllers.xml" />

  <group ns="forearm">
    <param name="ext_trigger" type="bool" value="True"/>
    <param name="if_name" type="str" value="eth1"/>
		<param name="ip_address" type="str" value="169.254.0.1"/>
    <!--param name="serial_number" type="int" value="2"/-->
    <param name="video_mode" type="str" value="$mode"/>
    <param name="do_colorize" type="bool" value="True"/>
    <param name="do_rectify" type="bool" value="False"/>
  </group>
  <node name="forearm" pkg="forearm_cam" type="forearm_node" respawn="false" output="screen"/>
  <!--node name="forearm_proc" pkg="stereo_image_proc" type="imageproc" respawn="false" output="screen">
    <remap from="camera" to="forearm"/>
  </node-->
  
  <group ns="forearm_view">
    <param name="window_name" type="str" value="Forearm camera"/>
  </group>
  <!--node name="forearm_view" pkg="image_view" type="image_view" respawn="false">
    <remap from="image" to="/forearm/image_color"/>
  </node-->
	
  <group ns="trigger_test">
    <param name="file_name" type="str" value="\$(find camera_trigger_test)/data/mode0out-$mode.txt"/>
    <param name="mode" type="int" value="0"/>
    <param name="ignore_count" type="int" value="1"/>
    <param name="repetitions" type="int" value="3"/>
  </group>
  <node name="trigger_test" pkg="camera_trigger_test" type="trigger_test" respawn="false" output="screen">
    <remap from="image" to="/forearm/image_raw"/>
  </node>
</launch>
EOF

roslaunch data/testmode0-$mode.launch &
sleep 50
echo ========================================================
echo ========================================================
echo ========================================================
echo ========================================================
echo Killing the job!
echo ========================================================
echo ========================================================
echo ========================================================
kill %%
sleep 15
done
             
             
             
             
             
             
             
             
             
