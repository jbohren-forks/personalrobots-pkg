PKG = slam_gmapping
CXX = g++
all: $(PKG)

CFLAGS = -O3 -Wall `rospack export/cpp/cflags $(PKG)`
LFLAGS = `rospack export/cpp/lflags $(PKG)`

slam_gmapping: slam_gmapping.cc
	$(CXX) $(CFLAGS) -o $@ $< $(LFLAGS)

clean:
	rm -f *.o $(PKG)
