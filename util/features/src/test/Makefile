CC = g++
CFLAGS = -c -g -Wall -I/usr/include -I../../include
CFLAGS += -O3
#CFLAGS += -DNDEBUG
CFLAGS += `pkg-config opencv --cflags`
LDFLAGS = `pkg-config opencv --libs`
SOURCES = ../integral.cpp ../detector.cpp ../keypoint.cpp keypoint_utils.cpp
OBJECTS = $(SOURCES:.cpp=.o)
PROGRAMS = affine_test show_keypts transform_image
PROGRAMS += willow_detect repeatability willow_timing

# SURF_ROOT must be set appropriately
CFLAGS += -I$(SURF_ROOT)
SURF_LDFLAGS = $(LDFLAGS) -L$(SURF_ROOT) -lSurf
SURF_SOURCES = surf_points.cpp ../keypoint.cpp $(SURF_ROOT)/imload.cpp keypoint_utils.cpp
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
	rm $(PROGRAMS) $(addsuffix .o,$(PROGRAMS)) $(OBJECTS)
	rm $(SIFT_PROGRAMS) $(addsuffix .o,$(SIFT_PROGRAMS))
	rm $(SURF_PROGRAMS) $(addsuffix .o,$(SURF_PROGRAMS)) $(SURF_OBJECTS)
