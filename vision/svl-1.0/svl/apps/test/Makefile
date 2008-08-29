# STAIR VISION LIBRARY TEST APPLICATIONS MAKEFILE
# Stephen Gould <sgould@stanford.edu>

LASIK_PATH := $(shell pwd)/../../..

USE_OPENCV = 1
USE_WX = 0
USE_GSL = 0
USE_LAPACK = 0

-include $(LASIK_PATH)/make.mk

#######################################################################
# add application source files here
#######################################################################
APP_SRC = testGraphicalModels.cpp testUtils.cpp testVisionUtils.cpp

#######################################################################

APP_PROG_NAMES = $(APP_SRC:.cpp=)
APP_OBJ = $(APP_SRC:.cpp=.o)

.PHONY: clean
.PRECIOUS: $(APP_OBJ)

all: depend ${addprefix ${BIN_PATH}/,$(APP_PROG_NAMES)}

$(BIN_PATH)/%: %.o $(LIBSVL)
	${CCC} $*.o -o $(@:.o=) $(LIBSVL) $(LFLAGS)

.cpp.o:
	${CCC} ${CFLAGS} -c $< -o $@

$(LIBSVL):
	@echo "** YOU NEED TO MAKE THE SVL LIBRARIES FIRST **"
	false

depend:
	g++ ${CFLAGS} -MM ${APP_SRC} >depend

clean:
	-rm $(APP_OBJ)
	-rm ${addprefix ${BIN_PATH}/,$(APP_PROG_NAMES)}
	-rm depend

-include depend
