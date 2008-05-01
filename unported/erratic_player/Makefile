PKG = erratic_player
CXX = g++
all: $(PKG)

CFLAGS = -g -Wall -Werror `rospack export/cpp/cflags $(PKG)`
LFLAGS = `rospack export/cpp/lflags $(PKG)`

erratic_player: erratic_player.cc
	$(CXX) $(CFLAGS) -o $@ $< $(LFLAGS)

clean:
	rm -f *.o $(PKG)
