CC = g++
FEATURES = ../../..
CFLAGS = -c -g -Wall
# OpenCV
CFLAGS += `pkg-config opencv --cflags`
LDFLAGS = `pkg-config opencv --libs`
# Boost
CFLAGS += -I$(BOOST_ROOT)/include
LDFLAGS += -L$(BOOST_ROOT)/lib -lboost_program_options-gcc42-mt -lboost_filesystem-gcc42-mt
#LDFLAGS += -lboost_thread-gcc42-mt
# Descriptor
CFLAGS += -I/usr/include -I$(FEATURES)/calonder_descriptor/include
# Star detector
CFLAGS += -I$(FEATURES)/star_detector/include -I$(FEATURES)/star_detector/src/test
LDFLAGS += -L$(FEATURES)/star_detector/lib -lwgdetect
# GSL
#CFLAGS += -DHAVE_GSL
#LDFLAGS += -L/usr/lib -lgsl -lgslcblas
# SIFT
CFLAGS += -I$(SIFT_ROOT)
LDFLAGS += -L$(SIFT_ROOT)/lib -lsift
# Optimization settings
CFLAGS += -O3 -DNDEBUG
#CFLAGS += -O0

SOURCES = ../patch_generator.cpp ../randomized_tree.cpp ../rtree_classifier.cpp
SOURCES += detectors.cpp
OBJECTS = $(SOURCES:.cpp=.o)
PROGRAMS = baseset_test matcher_test recognition_test show_base_set patch_test
#PROGRAMS += patch_test rtree_test classifier_test

all: $(PROGRAMS)

.SECONDEXPANSION:

$(PROGRAMS): $$@.o $(OBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS) $< -o $@

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm $(PROGRAMS) *.o ../*.o
