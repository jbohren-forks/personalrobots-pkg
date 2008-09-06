# STAIR VISION LIBRARY MATLAB (MEX) APPLICATIONS MAKEFILE
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
APP_SRC = mexCRFLearn.cpp mexCRFInfer.cpp mexTrainClassifier.cpp \
	mexEvalClassifier.cpp

OTHER_SRC = #matlabUtils.cpp

#######################################################################

APP_PROG_NAMES = $(APP_SRC:.cpp=)
APP_OBJ = $(APP_SRC:.cpp=.o)
OTHER_OBJ = $(OTHER_SRC:.cpp=.o)

MEXFLAGS = $(PLAT_CFLAGS) $(EXTRA_CFLAGS) \
	-lm $(PLAT_LFLAGS) $(EXTRA_LFLAGS) $(STATICSVL) $(EXTLIB)

ifeq ($(FORCE32BIT), 1)
  MEXFLAGS += -glnx86 #CFLAGS='-m32' LDFLAGS='-m32'
endif

.PHONY: clean
.PRECIOUS: $(APP_OBJ)

ifeq ($(BUILD_MEX_APPS), 1)
all: depend ${addprefix ${BIN_PATH}/,$(APP_PROG_NAMES)}
else
all:
endif

mex: ${addprefix ${BIN_PATH}/,$(APP_PROG_NAMES)}

$(BIN_PATH)/%: %.cpp $(OTHER_OBJ) $(STATICSVL)
	mex -g $*.cpp -o $(@:.cpp=) $(OTHER_OBJ) $(MEXFLAGS) $(STATICSVL) $(EXTLIB)

.cpp.o:
	${CCC} ${CFLAGS} -c $< -o $@

$(STATICSVL):
	@echo "** YOU NEED TO MAKE THE SVL LIBRARIES FIRST **"
	false

depend:
#	g++ ${CFLAGS} -MM ${OTHER_SRC} >depend

clean:
#	-rm $(APP_OBJ)
	-rm $(OTHER_OBJ)
	-rm ${addprefix ${addprefix ${BIN_PATH}/,$(APP_PROG_NAMES)}, ".mexglx"}
	-rm ${addprefix ${addprefix ${BIN_PATH}/,$(APP_PROG_NAMES)}, ".mexa64"}
	-rm depend

-include depend
