PKG = gazebo_plugin

CFLAGS = -g -Wall $(shell rospack export/cpp/cflags $(PKG)) -I/usr/include/libxml2

LFLAGS = $(shell rospack export/cpp/lflags $(PKG))

LDFLAGS = $(shell rospack export/cpp/lflags gazebo) \
          $(shell rospack export/cpp/lflags pr2Core) \
          $(shell rospack export/cpp/lflags controllers) \
          $(shell rospack export/cpp/lflags newmat10)

LIB_ACT = lib/libPr2_Actarray.a
LIB_GRP = lib/libPr2_Gripper.a
LIB_ACT_S = lib/libPr2_Actarray.so
LIB_GRP_S = lib/libPr2_Gripper.so
LIB_P3D = lib/libP3D.a
LIB_P3D_S = lib/libP3D.so

all: $(LIB_ACT) $(LIB_GRP) $(LIB_ACT_S) $(LIB_GRP_S)  $(LIB_P3D_S)  $(LIB_P3D) #player_pr2


libPr2_Actarray.o: src/Pr2_Actarray.cc
	g++ $(CFLAGS) -o $@ -c $< 

libPr2_Gripper.o: src/Pr2_Gripper.cc
	g++ $(CFLAGS) -o $@ -c $< 

libP3D.o: src/P3D.cc
	g++ $(CFLAGS) -o $@ -c $< 

$(LIB_ACT): libPr2_Actarray.o
	ar -rs $@ $^

$(LIB_GRP): libPr2_Gripper.o
	ar -rs $@ $^

$(LIB_P3D): libP3D.o
	ar -rs $@ $^

$(LIB_ACT_S): libPr2_Actarray.o
	g++ -shared -o $@ $^ $(LDFLAGS)

$(LIB_GRP_S): libPr2_Gripper.o
	g++ -shared -o $@ $^ $(LDFLAGS)

$(LIB_P3D_S): libP3D.o
	g++ -shared -o $@ $^ $(LDFLAGS)

player_pr2: src/player/Pr2_Player.cc
	g++ $(CFLAGS) -o $@ $^ $(LFLAGS)

clean:
	rm -rf player_pr2
	rm -f *.o *.a lib/*.a lib/*.so

