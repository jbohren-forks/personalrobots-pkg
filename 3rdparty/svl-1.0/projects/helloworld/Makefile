# STAIR VISION LIBRARY HELLOWORLD MAKEFILE
# Stephen Gould <sgould@stanford.edu>

LASIK_PATH := $(shell pwd)/../..

USE_OPENCV = 1
USE_WX = 0
USE_GSL = 0
USE_LAPACK = 0

-include $(LASIK_PATH)/make.mk

#######################################################################
# add application and shared source files here
#######################################################################
APP_SRC = helloWorldTrain.cpp helloWorldTest.cpp

OTHER_SRC = helloWorldModel.cpp

#######################################################################

APP_PROG_NAMES = $(APP_SRC:.cpp=)
APP_OBJ = $(APP_SRC:.cpp=.o)
OTHER_OBJ = $(OTHER_SRC:.cpp=.o)

.PHONY: clean
.PRECIOUS: $(APP_OBJ)

all: depend ${addprefix ${BIN_PATH}/,$(APP_PROG_NAMES)}

$(BIN_PATH)/%: %.o $(OTHER_OBJ) $(LIBSVL)
	${CCC} $*.o -o $(@:.o=) $(OTHER_OBJ) $(LIBSVL) $(LFLAGS)

$(LIBSVL):
	@echo "** YOU NEED TO MAKE THE SVL LIBRARIES FIRST **"
	false

.cpp.o:
	${CCC} ${CFLAGS} -c $< -o $@

depend:
	g++ ${CFLAGS} -MM ${APP_SRC} ${OTHER_SRC} >depend

clean:
	-rm $(APP_OBJ)
	-rm $(OTHER_OBJ)
	-rm ${addprefix ${BIN_PATH}/,$(APP_PROG_NAMES)}
	-rm depend

-include depend

