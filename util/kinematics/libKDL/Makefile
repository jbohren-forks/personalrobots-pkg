PKG = libKDL
CXX = g++

CFLAGS = -g -Wall -Iinclude $(shell rospack export/cpp/cflags $(PKG))
LFLAGS  = $(shell rospack export/cpp/lflags $(PKG))

LIBKDL = lib/libPr2KDL.a

all: bin/pr2_kin_test $(LIBKDL)

bin/pr2_kin_test: $(LIBKDL) include/kdl_kinematics.h test/pr2_kin_test.cpp
	$(CXX) $(CFLAGS) -o $@ $^ $(LFLAGS)

$(LIBKDL): obj/kdl_kinematics.o
	ar -rs $@ $^

obj/kdl_kinematics.o: src/kdl_kinematics.cpp include/kdl_kinematics.h
	g++  $(CFLAGS) -o $@ -c $<

clean:
	-rm -f obj/*
	-rm -f lib/*
	-rm -f bin/*

