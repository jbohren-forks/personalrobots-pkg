PKG = slam_gmapping
CXX = g++
all: $(PKG)

CFLAGS = -O3 -Wall `rospack --lang=cpp --attrib=cflags export $(PKG)`
LFLAGS = `rospack --lang=cpp --attrib=lflags export $(PKG)`

slam_gmapping: slam_gmapping.cc
	$(CXX) $(CFLAGS) -o $@ $< $(LFLAGS)

clean:
	rm -f *.o $(PKG)
