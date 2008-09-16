CC = g++
FEATURES = ../../../..
CFLAGS = -c -g -Wall
CFLAGS += -I/usr/include
CFLAGS += -I$(FEATURES)/star_detector/src/test -I$(FEATURES)/calonder_descriptor/include
LDFLAGS = -L/usr/lib -lgsl -lgslcblas

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
