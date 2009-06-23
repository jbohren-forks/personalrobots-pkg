CC = g++
CFLAGS = -c -g -Wall -I/usr/include -I../../include
CFLAGS += -O3 -DNDEBUG
#CFLAGS += -O0
# OpenCV
CV_LATEST = $(shell rospack find opencv_latest)/opencv
CFLAGS += -I$(CV_LATEST)/include/opencv
LDFLAGS = -L$(CV_LATEST)/lib -lcv -lhighgui
# Boost
CFLAGS += -I$(BOOST_ROOT)/include
# Star detector
STAR_DIR = $(shell rospack find star_detector)
CFLAGS += -I/usr/include -I$(STAR_DIR)/include
LDFLAGS += -L$(STAR_DIR)/lib -lstarfeature

SOURCES = keypoint_utils.cpp
OBJECTS = $(SOURCES:.cpp=.o)
PROGRAMS = affine_test show_keypts transform_image
PROGRAMS += willow_detect repeatability willow_timing

# SURF_ROOT must be set appropriately
CFLAGS += -I$(SURF_ROOT)
SURF_LDFLAGS = $(LDFLAGS) -L$(SURF_ROOT) -lSurf
SURF_SOURCES = surf_points.cpp $(SURF_ROOT)/imload.cpp keypoint_utils.cpp
SURF_OBJECTS = $(SURF_SOURCES:.cpp=.o)
SURF_PROGRAMS = surf_detect

# SIFT_ROOT must be set appropriately
CFLAGS += -I$(SIFT_ROOT)
SIFT_LDFLAGS = $(LDFLAGS) -L$(SIFT_ROOT)/lib -lsift
SIFT_SOURCES = keypoint_utils.cpp
SIFT_OBJECTS = $(SIFT_SOURCES:.cpp=.o)
SIFT_PROGRAMS = sift_detect

all: $(SOURCES) $(PROGRAMS) $(SURF_PROGRAMS) $(SIFT_PROGRAMS)

.SECONDEXPANSION:

$(PROGRAMS): $$@.o $(OBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS) $< -o $@

$(SURF_PROGRAMS): $$@.o $(SURF_OBJECTS)
	$(CC) $(SURF_LDFLAGS) $(SURF_OBJECTS) $< -o $@

$(SIFT_PROGRAMS): $$@.o $(OBJECTS)
	$(CC) $(SIFT_LDFLAGS) $(SIFT_OBJECTS) $< -o $@

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm -f $(PROGRAMS) $(SIFT_PROGRAMS) $(SURF_PROGRAMS) *.o
