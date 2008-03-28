CPP = g++
AR = ar
CPPFLAGS = -g -Wall -Werror `pkg-config --cflags playercore playerxdr playerutils playerdrivers` `rospack cflags roscpp` `rospack cflags common_flows`
LDFLAGS = `pkg-config --libs playercore playerxdr playerutils playerdrivers` `rospack lflags roscpp` `rospack lflags common_flows`

all: nodes/erratic_player

nodes/erratic_player: erratic_player.cc
	$(CPP) $(CPPFLAGS) -o $@ $< $(LDFLAGS)

clean:
	rm -f *.o nodes/erratic_player
