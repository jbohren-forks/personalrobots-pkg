PKG = amcl_player
CXX = g++
all: $(PKG)

CFLAGS = -O3 -Wall `rospack export/cpp/cflags $(PKG)` `pkg-config --cflags gdk-pixbuf-2.0`
#LFLAGS = `rospack export/cpp/lflags $(PKG)` -lplayerdrivers -L /u/gerkey/code/personalrobots/3rdparty/player/player-svn/build/rtk2 -lrtk `pkg-config --libs gdk-pixbuf-2.0`
LFLAGS = `rospack export/cpp/lflags $(PKG)` -lplayerdrivers `pkg-config --libs gdk-pixbuf-2.0`

amcl_player: amcl_player.cc
	$(CXX) $(CFLAGS) -o $@ $< $(LFLAGS)

clean:
	rm -f *.o $(PKG)
