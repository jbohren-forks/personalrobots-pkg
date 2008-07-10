PKG = libKDL
CXX = g++

CFLAGS = -g -Wall -Iinclude $(shell rospack export/cpp/cflags $(PKG))
LFLAGS  = $(shell rospack export/cpp/lflags $(PKG))

LIBKDL = lib/libPr2KDL.a

all: bin/pr2_kin_test bin/dynamics $(LIBKDL)

bin/dynamics: $(LIBKDL) include/libKDL/kdl_kinematics.h obj/dynamics.o
	$(CXX) $(CFLAGS) -o $@ $^ $(LFLAGS)

bin/pr2_kin_test: $(LIBKDL) include/libKDL/kdl_kinematics.h obj/pr2_kin_test.o
	$(CXX) $(CFLAGS) -o $@ $^ $(LFLAGS)

$(LIBKDL): obj/kdl_kinematics.o
	ar -rs $@ $^

obj/pr2_kin_test.o: test/pr2_kin_test.cpp include/libKDL/kdl_kinematics.h
	g++  $(CFLAGS) -o $@ -c $<

obj/dynamics.o: test/dynamics.cpp include/libKDL/kdl_kinematics.h include/libKDL/dynamics.h
	g++  $(CFLAGS) -o $@ -c $<

obj/kdl_kinematics.o: src/kdl_kinematics.cpp include/libKDL/kdl_kinematics.h
	g++  $(CFLAGS) -o $@ -c $<

clean:
	-rm -f obj/*
	-rm -f lib/*
	-rm -f bin/*

