PKG = teleop_arm_keyboard
CXX = g++
all: $(PKG)

CFLAGS = -g -Wall `rospack export/cpp/cflags $(PKG)` $(shell pkg-config --cflags libpr2API)
LFLAGS = `rospack export/cpp/lflags $(PKG)`

teleop_arm_keyboard: teleop_arm_keyboard.cc
	$(CXX) $(CFLAGS) -o $@ $< $(LFLAGS)

clean:
	rm -rf *.o $(PKG) $(PKG).dSYM
