CC = g++
CFLAGS = -c -Wall `pkg-config opencv --cflags` -I/usr/include -I../../include
CFLAGS += -O3
#CFLAGS += -DNDEBUG
LDFLAGS = `pkg-config opencv --libs`
SOURCES = ../integral.cpp ../censure.cpp
OBJECTS = $(SOURCES:.cpp=.o)
PROGRAMS = oct_filter bilevel_test keypoint_test affine_test

all: $(SOURCES) $(PROGRAMS)

.SECONDEXPANSION:

$(PROGRAMS): $$@.o $(OBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS) $< -o $@

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm $(PROGRAMS) $(addsuffix .o,$(PROGRAMS)) $(OBJECTS)
