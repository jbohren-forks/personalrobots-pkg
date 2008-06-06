PKG = gazebo_plugin

CFLAGS = -g -Wall $(shell rospack export/cpp/cflags $(PKG)) -I/usr/include/libxml2

LDFLAGS = $(shell rospack export/cpp/lflags $(PKG))

LIB_ACT = lib/libPr2_Actarray.a
LIB_GRP = lib/libPr2_Gripper.a
LIB_ACT_S = lib/libPr2_Actarray.so
LIB_GRP_S = lib/libPr2_Gripper.so

all: $(LIB_ACT) $(LIB_GRP) $(LIB_ACT_S) $(LIB_GRP_S) #player_pr2


libPr2_Actarray.o: src/Pr2_Actarray.cc
	g++ $(CFLAGS) -o $@ -c $< 

libPr2_Gripper.o: src/Pr2_Gripper.cc
	g++ $(CFLAGS) -o $@ -c $< 

$(LIB_ACT): libPr2_Actarray.o
	ar -rs $@ $^

$(LIB_GRP): libPr2_Gripper.o
	ar -rs $@ $^

$(LIB_ACT_S): libPr2_Actarray.o
	g++ -shared -o $@ $^

$(LIB_GRP_S): libPr2_Gripper.o
	g++ -shared -o $@ $^

player_pr2: src/player/Pr2_Player.cc
	g++ $(CFLAGS) -o $@ $^ $(LDFLAGS)

clean:
	rm -rf player_pr2
	rm -f *.o *.a lib/*.a

