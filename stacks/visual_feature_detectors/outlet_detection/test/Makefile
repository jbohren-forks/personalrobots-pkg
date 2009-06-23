CC=g++

CFLAGS=-c -Wall -I../include -I../src -I/u/bradski/ros/ros-pkg/vision/calonder_descriptor/include -I/u/bradski/ros/ros-pkg/3rdparty/opencv_latest/opencv/include/opencv -I/u/bradski/ros/ros-pkg/vision/star_detector/include -I/u/bradski/ros/ros-pkg/3rdparty/boost/boost_1_36_0 -I/u/calonder/dev/mcutils/include

LDFLAGS=-L/u/bradski/ros/ros-pkg/vision/calonder_descriptor/lib -L/u/bradski/ros/ros-pkg/3rdparty/opencv_latest/opencv/lib -L/u/bradski/ros/ros-pkg/vision/star_detector/lib -L/u/calonder/dev/mcutils/lib -L/u/calonder/ros/ros-pkg/3rdparty/boost/boost_1_36_0/lib -lcalonder -lcv -lcxcore -lhighgui -lstarfeature -lmcutils -lboost_program_options

SOURCES=main.cpp ../src/TrainBase.cpp

OBJECTS=$(SOURCES:.cpp=.o)

EXECUTABLE=../bin/train_base

all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm -f $(OBJECTS)

