PKG = wavefront_player
CXX = g++
all: $(PKG)

CFLAGS = -O3 -Wall `rospack export/cpp/cflags $(PKG)` `pkg-config --cflags gdk-pixbuf-2.0`
LFLAGS = `rospack export/cpp/lflags $(PKG)` -lwavefront_standalone `pkg-config --libs gdk-pixbuf-2.0`

wavefront_player: wavefront_player.cc
	$(CXX) $(CFLAGS) -o $@ $< $(LFLAGS)

send_goal: send_goal.cc
	$(CXX) $(CFLAGS) -o $@ $< $(LFLAGS)

testtf: testtf.cc
	$(CXX) $(CFLAGS) -o $@ $< $(LFLAGS)

clean:
	rm -f *.o $(PKG)
