Hi All,

This is a node which will put texture light on and off and combine image without texture light with good stereo point clouds. 
It will publish the following topic:
/t_on_off/caminfo
/t_on_off/cloud
/t_on_off/disp
/t_on_off/image

There is a simple launch file (t_on_off.launch) to run it,
and please remember to put led_ctrl and mailbox under ~/ros/ros .
Hopefully, it will be useful for you.

Min Sun
