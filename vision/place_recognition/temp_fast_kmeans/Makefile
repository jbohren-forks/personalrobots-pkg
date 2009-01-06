CC=g++

PROF_FLAGS = #-pg
OPT_FLAGS  = -march=pentium4 -O3 #-mfpmath=sse -funroll-loops

CFLAGS = -c -Wall $(OPT_FLAGS) $(PROF_FLAGS) -I../../src -I/u/calonder/ros/ros-pkg/vision/calonder_descriptor/include -I/u/calonder/ros/ros-pkg/3rdparty/opencv_latest/opencv/include/opencv -I/u/calonder/ros/ros-pkg/vision/star_detector/include -I/u/calonder/ros/ros-pkg/3rdparty/boost/boost_1_36_0 -I/u/calonder/dev/mcutils/include -I/u/calonder/lib/gsl-1.11 -I/u/calonder/dev/mpi_kmeans/orig_code

LDFLAGS = $(PROF_FLAGS) -L/u/calonder/ros/ros-pkg/vision/calonder_descriptor/lib -L/u/calonder/ros/ros-pkg/3rdparty/opencv_latest/opencv/lib -L/u/calonder/ros/ros-pkg/vision/star_detector/lib -L/u/calonder/dev/mcutils/lib -L/u/calonder/lib/gsl-1.11/.libs -lcalonder -lcv -lcxcore -lhighgui -lstarfeature -lmcutils -lgsl

SOURCES=main.cpp mpi_kmeans.cpp
OBJECTS=$(SOURCES:.cpp=.o)
EXECUTABLE=kmeans

all: $(SOURCES) $(EXECUTABLE)
	
$(EXECUTABLE): $(OBJECTS) 
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm -f $(OBJECTS) $(EXECUTABLE)

