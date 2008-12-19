# TODO: integrate with ROS build system
CC = g++
CFLAGS = -c -g -Wall
# OpenCV
CV_LATEST = $(shell rospack find opencv_latest)/opencv
CFLAGS += -I$(CV_LATEST)/include/opencv
LDFLAGS = -L$(CV_LATEST)/lib -lcv -lhighgui
# Boost
CFLAGS += -I$(BOOST_ROOT)/include
LDFLAGS += -L$(BOOST_ROOT)/lib -lboost_program_options-mt -lboost_filesystem-mt
#LDFLAGS += -lboost_thread-mt
# Descriptor
CALONDER = $(shell rospack find calonder_descriptor)
CFLAGS += -I/usr/include -I$(CALONDER)/include
LDFLAGS += -L$(CALONDER)/lib -lcalonder
# Star detector
STAR_DIR = $(shell rospack find star_detector)
CFLAGS += -I/usr/include -I$(STAR_DIR)/include -I$(STAR_DIR)/src/test
LDFLAGS += -L$(STAR_DIR)/lib -lstarfeature
# Google CPU profiler
CFLAGS += -I/usr/local/include
LDFLAGS += -L/usr/local/lib -lprofiler
# GSL
#CFLAGS += -DHAVE_GSL
#LDFLAGS += -L/usr/lib -lgsl -lgslcblas
# SIFT (get rid of this?)
CFLAGS += -I$(SIFT_ROOT)
LDFLAGS += -L$(SIFT_ROOT)/lib -lsift
# FAST
FAST_DIR = $(shell rospack find fast_detector)
CFLAGS += -I$(FAST_DIR)/src
LDFLAGS += -L$(FAST_DIR)/lib -lfast_detector
# Optimization settings
CFLAGS += -DNDEBUG
#CFLAGS += -O0 -g
#CFLAGS += -O1
#CFLAGS += -O2
CFLAGS += -O3 -msse2
#CFLAGS += -march=native
#CFLAGS += -mfpmath=sse
#CFLAGS += -ffast-math
#CFLAGS += -fomit-frame-pointer
#CFLAGS += -fno-rtti

SOURCES = detectors.cpp
OBJECTS = $(SOURCES:.cpp=.o)
PROGRAMS = baseset_test recognition_test show_base_set patch_test
PROGRAMS += match_benchmark directed_test
PROGRAMS += getsig_benchmark sig_profile
#PROGRAMS += matcher_test rtree_test classifier_test write_posteriors

all: $(PROGRAMS)

.SECONDEXPANSION:

$(PROGRAMS): $$@.o $(OBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS) $< -o $@

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm -f $(PROGRAMS) *.o
