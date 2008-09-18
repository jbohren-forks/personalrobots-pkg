CC = g++
CFLAGS = -c -g -Wall
# Boost
CFLAGS += -I$(BOOST_ROOT)/include
# Star detector (for timer.h)
FEATURES = /u/mihelich/projects/features
CFLAGS += -I$(FEATURES)/star_detector/src/test
CFLAGS += -I$(FEATURES)/calonder_descriptor/include
# Descriptor
CALONDER = $(shell rospack find calonder_descriptor)
CFLAGS += -I/usr/include -I$(CALONDER)/include
# Optimization options
CFLAGS += -O3 -DNDEBUG
#CFLAGS += -march=native -mfpmath=sse
#CFLAGS += -fomit-frame-pointer
#CFLAGS += -funroll-loops
#CFLAGS += -ffast-math
# Profile feedback based optimization seems not to help
#PROFILE_OPT = -fprofile-generate
#PROFILE_OPT = -fprofile-use
#CFLAGS += $(PROFILE_OPT)
#LDFLAGS += $(PROFILE_OPT)

SOURCES = 
OBJECTS = $(SOURCES:.cpp=.o)
PROGRAMS = sparse_benchmark dense_benchmark

all: $(PROGRAMS)

.SECONDEXPANSION:

$(PROGRAMS): $$@.o $(OBJECTS)
	$(CC) -o $@ $< $(OBJECTS) $(LDFLAGS)

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm $(PROGRAMS) *.o
