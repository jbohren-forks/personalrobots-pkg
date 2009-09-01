cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(bagserver)

gensrv()
genmsg()

rospack_download_test_data(http://pr.willowgarage.com/data/bagserver/test_images2.bag test_data/test_images2.bag)
rospack_download_test_data(http://pr.willowgarage.com/data/bagserver/test_images2.index test_data/test_images2.index)

#Test
rospack_add_rostest(test/test_bagserver.launch)
