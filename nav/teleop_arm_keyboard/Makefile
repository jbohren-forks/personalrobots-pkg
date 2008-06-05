SRC = teleop_arm_keyboard.cc
OUT = teleop_arm_keyboard
PKG = teleop_arm_keyboard
CXX = g++
all: $(PKG)


CFLAGS = -g -Wall $(shell rospack export/cpp/cflags $(PKG))

LFLAGS = $(shell rospack export/cpp/lflags $(PKG))

teleop_arm_keyboard: teleop_arm_keyboard.cc
	$(CXX) $(CFLAGS) -o $@ $< $(LFLAGS)

clean:
	rm -rf *.o $(PKG) $(PKG).dSYM
