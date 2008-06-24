PKG  = libpr2API
CXX = g++

CFLAGS = -g -Wall $(shell rospack export/cpp/cflags $(PKG)) \
				 $(shell rospack export/cpp/cflags libKDL)

LDFLAGS = $(shell rospack export/cpp/lflags gazebo) \
          $(shell rospack export/cpp/lflags pr2Core) \
          $(shell rospack export/cpp/lflags libKinematics) \
          $(shell rospack export/cpp/lflags libpr2HW) \
          $(shell rospack export/cpp/lflags newmat10) \
          $(shell rospack export/cpp/lflags libKDL)

LFLAGS = $(shell rospack export/cpp/lflags $(PKG))

LIBP = lib/libpr2API.a
LIBS = lib/libpr2API.so

all: $(LIBP) bin/test_libpr2API bin/pr2_kin_kdl bin/pick_object bin/test_kin_controllers bin/torque_test

obj/pr2API.o: src/pr2API.cpp include/libpr2API/pr2API.h
	$(CXX) $(CFLAGS) -c -o $@ $<

obj/kinematic_controllers.o: src/kinematic_controllers.cpp include/libpr2API/pr2API.h include/kinematic_controllers.h
	$(CXX) $(CFLAGS) -c -o $@ $<

obj/test_kin_controllers.o: src/test/test_kin_controllers.cpp
	$(CXX) $(CFLAGS) -c -o $@ $<

$(LIBP): obj/pr2API.o
	ar -rv $@ $^

$(LIBS): pr2API.o
	$(CXX) -shared -o $@ $^ $(LDFLAGS)

bin/test_libpr2API: src/test/test_pr2API.cpp $(LIBP)
	$(CXX) $(CFLAGS) -o $@ $< $(LFLAGS)

bin/torque_test: src/test/torque_test.cpp $(LIBP)
	$(CXX) $(CFLAGS) -o $@ $< $(LFLAGS)

bin/pick_object: src/test/pick_object.cpp $(LIBP)
	$(CXX) $(CFLAGS) -o $@ $< $(LFLAGS)

bin/test_kin_controllers: obj/test_kin_controllers.o obj/kinematic_controllers.o $(LIBP)
	$(CXX) -o $@ $^ $(LFLAGS)

bin/pr2_kin_kdl: src/test/pr2_kin_kdl.cpp $(LIBP)
	$(CXX) $(CFLAGS) -o $@ $< $(LFLAGS)


clean:
	-rm -f $(LIBP) $(LIBS) bin/* obj/*
	-rm -rf *.o $(PKG) $(PKG).dSYM

