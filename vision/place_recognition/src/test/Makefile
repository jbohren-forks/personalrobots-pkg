CC = g++
CFLAGS = -c -g -Wall
# Boost
BOOST_ROOT = $(shell rospack find boost)/boost
CFLAGS += -I$(BOOST_ROOT)/include
LDFLAGS += -L$(BOOST_ROOT)/lib -lboost_filesystem
# OpenCV
CV_LATEST = $(shell rospack find opencv_latest)/opencv
CFLAGS += -I$(CV_LATEST)/include/opencv -I$(CV_LATEST)/include
LDFLAGS += -L$(CV_LATEST)/lib -lcv -lhighgui
# Star detector
STAR_DIR = $(shell rospack find star_detector)
CFLAGS += -I$(STAR_DIR)/include
LDFLAGS += -L$(STAR_DIR)/lib -lstarfeature
# C-descriptor
CALONDER = $(shell rospack find calonder_descriptor)
CFLAGS += -I$(CALONDER)/include
LDFLAGS += -L$(CALONDER)/lib -lcalonder
# Visual odometry
VISODOM = $(shell rospack find visual_odometry)
CFLAGS += -I$(VISODOM)/include
LDFLAGS += -L$(VISODOM)/lib -lvisual_odometry
STEREO_UTILS = $(shell rospack find stereo_utils)
CFLAGS += -I$(STEREO_UTILS)/include
LDFLAGS += -L$(STEREO_UTILS)/lib -lstereo_utils
# Eigen
EIGEN = $(shell rospack find eigen)/build/eigen2
CFLAGS += -I$(EIGEN)
# And of course, place recognition
PLACE_RECOGNITION = $(shell rospack find place_recognition)
CFLAGS += -I$(PLACE_RECOGNITION)/include
LDFLAGS += -L$(PLACE_RECOGNITION)/lib -lplace_recognition
# Optimization
#CFLAGS += -O0
CFLAGS += -O3
CFLAGS += -msse2
CFLAGS += -DNDEBUG

SOURCES = 
OBJECTS = $(SOURCES:.cpp=.o)
PROGRAMS = vocab_test geom_test loop_test load_test kmeans_test

all: $(PROGRAMS)

.SECONDEXPANSION:

$(PROGRAMS): $$@.o $(OBJECTS)
	$(CC) $(OBJECTS) $< $(LDFLAGS) -o $@

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm -f $(PROGRAMS) *.o
