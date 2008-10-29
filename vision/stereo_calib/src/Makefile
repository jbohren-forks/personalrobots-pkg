#
# simple makefile for stereo cal GUI
#

CC = gcc
CXX = g++
LD = g++

# change the following to point to your version of visual odometry/pose estimate lib
POSEESTIMATE = $(ROS_PACKAGE_PATH)/vision/visual_odometry

# change the following to point to your version of opencv
OPENCV=$(ROS_PACKAGE_PATH)/3rdparty/opencv_latest/opencv

# change the following to point to your version of boost
BOOST=$(ROS_PACKAGE_PATH)/3rdparty/boost/boost_1_36_0

CPPFLAGS = -Wall -fmessage-length=0 -g3 -g -O0 -I$(OPENCV)/include\
	-I$(POSEESTIMATE)/include\
	-I$(BOOST)

# sudo apt-get install libfltk-dev

LDFLAGS = -L$(POSEESTIMATE)/lib -L$(OPENCV)/lib\
	-lvisual_odometry -lcv -lhighgui -lfltk

OBJECTS = stereocal.o stereogui.o imwin.o calwin.o stereolib.o
STLRFOBJECTS = stlrf.o stereolrfgui.o imwin.o calwin.o cloud_transform.o

all:	stereocal stlrf

stereocal:	$(OBJECTS) 
	$(LD) $(LDFLAGS) -o stereocal $(OBJECTS) 

stereocal.o:	stereolib.h 

stlrf:	$(STLRFOBJECTS) 
	$(LD) -o stlrf $(STLRFOBJECTS) $(LDFLAGS) 


clean:
	- rm *.o
	- rm stereocal

dist:
	tar -cvzf cal.tgz *.c *.cpp *.h Makefile data/*.bmp

