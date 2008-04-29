PKG = hokuyourg_player
CXX = g++
all: $(PKG)

#CFLAGS = -g -Wall -Werror `pkg-config --cflags-only-I playercore` `rospack export/cpp/cflags $(PKG)`
#LFLAGS = `pkg-config --libs-only-L playercore` -lurglaser_standalone `rospack export/cpp/lflags $(PKG)`
CFLAGS = -g -Wall -Werror `rospack export/cpp/cflags $(PKG)`
LFLAGS = `rospack export/cpp/lflags $(PKG)` -lurglaser_standalone 

hokuyourg_player: hokuyourg_player.cc
	$(CXX) $(CFLAGS) -o $@ $< $(LFLAGS)

clean:
	rm -f *.o $(PKG)
