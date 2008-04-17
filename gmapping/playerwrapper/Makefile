OBJS= playergfswrapper.o
APPS= 

#LIBS+= -L$(CARMEN_HOME)/lib -lnavigator_interface -lsimulator_interface -lrobot_interface -llaser_interface -lparam_interface -lglobal -lipc -lm -lutils -lsensor_range -llog -lscanmatcher -lpthread -lz
LIBS+= `pkg-config --libs playercore` -lsensor_range
#-L$(CARMEN_HOME)/lib -lnavigator_interface -lsimulator_interface -lrobot_interface -llaser_interface -lparam_interface -lglobal -lipc -lm -lutils -lsensor_range -llog -lscanmatcher -lpthread -lz
CPPFLAGS+=-I../sensor `pkg-config --cflags playercore`

-include ../global.mk

ifeq ($(PLAYERSUPPORT), 0)
OBJS=
	-include ../build_tools/Makefile.app
else
	-include ../build_tools/Makefile.generic-shared-object
endif
