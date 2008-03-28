CPP = g++
AR = ar
CPPFLAGS = -g -Wall -Werror `pkg-config --cflags playercore playerxdr playerutils playerdrivers` `rospack cflags roscpp` `rospack cflags common_flows`
LDFLAGS = `pkg-config --libs playercore playerxdr playerutils playerdrivers` `rospack lflags roscpp` `rospack lflags common_flows`

all: nodes/hokuyourg_player

nodes/hokuyourg_player: hokuyourg_player.cc
	$(CPP) $(CPPFLAGS) -o $@ $< $(LDFLAGS)

clean:
	rm -f *.o nodes/hokuyourg_player
