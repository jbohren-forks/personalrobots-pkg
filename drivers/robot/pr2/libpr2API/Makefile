PKG  = libpr2API
CXX = g++

CFLAGS = -g -Wall $(shell rospack export/cpp/cflags $(PKG)) \
				 $(shell rospack export/cpp/cflags libKDL)

EXT_DEPS=

LFLAGS = $(shell rospack export/cpp/lflags $(PKG))

LIBP = lib/libpr2API.a
LIBS = lib/libpr2API.so

all: $(LIBS) bin/test_libpr2API bin/pr2_kin_kdl bin/pick_object bin/test_kin_controllers bin/torque_test bin/test_FK

#$(LIBP): obj/pr2API.o
#	ar -rv $@ $^

$(LIBS): obj/pr2API.o
	$(CXX) -shared -o $@ $^ $(LFLAGS)

#--------- obj ---------------
obj/pr2API.o: src/pr2API.cpp include/libpr2API/pr2API.h
	$(CXX) $(CFLAGS) -c -o $@ $<

obj/kinematic_controllers.o: src/kinematic_controllers.cpp include/libpr2API/pr2API.h include/kinematic_controllers.h
	$(CXX) $(CFLAGS) -c -o $@ $<

obj/test_kin_controllers.o: src/test/test_kin_controllers.cpp
	$(CXX) $(CFLAGS) -c -o $@ $<

#--------- bin -------------
bin/test_libpr2API: src/test/test_pr2API.cpp
	$(CXX) $(CFLAGS) -o $@ $< $(LFLAGS)

bin/torque_test: src/test/torque_test.cpp
	$(CXX) $(CFLAGS) -o $@ $< $(LFLAGS)

bin/pick_object: src/test/pick_object.cpp
	$(CXX) $(CFLAGS) -o $@ $< $(LFLAGS)

bin/test_kin_controllers: obj/test_kin_controllers.o obj/kinematic_controllers.o
	$(CXX) -o $@ $^ $(LFLAGS)

bin/pr2_kin_kdl: src/test/pr2_kin_kdl.cpp
	$(CXX) $(CFLAGS) -o $@ $< $(LFLAGS)

bin/test_FK: src/test/test_FK.cpp
	$(CXX) $(CFLAGS) -o $@ $< $(LFLAGS)

clean:
	-rm -f $(LIBS) bin/* obj/*
	-rm -rf *.o $(PKG) $(PKG).dSYM



