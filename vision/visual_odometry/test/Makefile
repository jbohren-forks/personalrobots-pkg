CC = g++
CFLAGS = -c -g -Wall -I/usr/include -I../include
CFLAGS += -O3 -DNDEBUG
# OpenCV
CV_LATEST = $(shell rospack find opencv_latest)/opencv
CFLAGS += -I$(CV_LATEST)/include/
LDFLAGS = -L$(CV_LATEST)/lib -lcv -lhighgui
# Boost
BOOST_ROOT = $(shell rospack find boost)/boost_1_36_0/
CFLAGS += -I$(BOOST_ROOT)
# visual odometry lib
VO_DIR = $(shell rospack find visual_odometry)
CFLAGS += -I/usr/include -I$(STAR_DIR)/include
LDFLAGS += -L$(VO_DIR)/lib -lvisual_odometry -lpython2.5

#SOURCES = CvTest3DPoseEstimate.cpp
SOURCES = 
OBJECTS = $(SOURCES:.cpp=.o)
PROGRAMS = CvTest3DPoseEstimate
PROGRAMS += 

all: $(SOURCES) $(PROGRAMS)

.SECONDEXPANSION:

$(PROGRAMS): $$@.o $(OBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS) $< -o $@

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm -f $(PROGRAMS) $(SIFT_PROGRAMS) $(SURF_PROGRAMS) *.o
