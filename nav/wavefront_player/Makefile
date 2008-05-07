PKG = wavefront_player
CXX = g++
all: $(PKG)

CFLAGS = -g -Wall `rospack export/cpp/cflags $(PKG)`
LFLAGS = `rospack export/cpp/lflags $(PKG)`

wavefront_player: wavefront_player.cc
	$(CXX) $(CFLAGS) -o $@ $< $(LFLAGS)

clean:
	rm -f *.o $(PKG)
