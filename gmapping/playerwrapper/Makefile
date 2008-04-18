OBJS= ../gui/gsp_thread.o ../gui/qparticleviewer.o ../gui/qgraphpainter.o ../gui/qmappainter.o  playergfswrapper.o
APPS= 

LIBS+= `pkg-config --libs playercore playerxdr` -lsensor_range -lgridfastslam -llog -lutils -lconfigfile -lsensor_base
CPPFLAGS+=-Wall -I../sensor `pkg-config --cflags playercore`

LDFLAGS+=$(QT_LIB)
CPPFLAGS+=$(QT_INCLUDE)

-include ../global.mk

ifeq ($(PLAYERSUPPORT), 0)
OBJS=
	-include ../build_tools/Makefile.app
else
	-include ../build_tools/Makefile.generic-shared-object
endif
