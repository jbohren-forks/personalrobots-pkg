PKG = teleop_base_keyboard
CXX = g++
all: $(PKG)

CFLAGS = -g -Wall `rospack export/cpp/cflags $(PKG)`
LFLAGS = `rospack export/cpp/lflags $(PKG)`

teleop_base_keyboard: teleop_base_keyboard.cc
	$(CXX) $(CFLAGS) -o $@ $< $(LFLAGS)

clean:
	rm -rf *.o $(PKG) $(PKG).dSYM
