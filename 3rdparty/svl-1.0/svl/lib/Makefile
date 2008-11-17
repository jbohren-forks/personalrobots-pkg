# STAIR VISION LIBRARY MAKEFILE
# Stephen Gould <sgould@stanford.edu>

LASIK_PATH := $(shell pwd)/../..
USE_OPENCV = 1
USE_OPENGL = 0
USE_WX = 0
USE_BOOST = 1

-include $(LASIK_PATH)/make.mk

# library source files ---------------------------------------------------
BASE_LIB_SRC = svlCodeProfiler.cpp svlCommandLine.cpp svlFileUtils.cpp \
	svlLogger.cpp svlOptimizer.cpp svlOptions.cpp svlStatsUtils.cpp \
	svlStrUtils.cpp

DEPRECATED_LIB_SRC =

DEVEL_LIB_SRC = svlThreadPool.cpp

ML_LIB_SRC = svlConfusionMatrix.cpp svlLogistic.cpp

PGM_LIB_SRC = svlFactor.cpp svlFactorOperations.cpp svlFactorTemplate.cpp \
	svlClusterGraph.cpp svlMessagePassing.cpp \
	svlPairwiseCRF.cpp svlGeneralCRF.cpp svlGraphUtils.cpp

VISION_LIB_SRC = svlCameraIntrinsics.cpp svlCameraExtrinsics.cpp \
	svlConvolution.cpp svlImageProjector.cpp svlImageSequence.cpp \
	svlObjectList.cpp svlPatchDefinition.cpp svlPatchDictionary.cpp \
	svlPatchBasedObjectDetector.cpp svlPointCloudData.cpp \
	svlRegionFeatures.cpp svlSegImage.cpp \
	svlSlidingWindowDetector.cpp svlSoftEdgeMap.cpp \
	svlSuperResolution.cpp svlVisionUtils.cpp \
	svlSpinImages.cpp

# ------------------------------------------------------------------------

LIB_SRC = ${addprefix base/,$(BASE_LIB_SRC)} \
	${addprefix deprecated/,$(DEPRECATED_LIB_SRC)} \
	${addprefix devel/,$(DEVEL_LIB_SRC)} \
	${addprefix ml/,$(ML_LIB_SRC)} \
	${addprefix pgm/,$(PGM_LIB_SRC)} \
	${addprefix vision/,$(VISION_LIB_SRC)}

LIB_OBJ = $(LIB_SRC:.cpp=.o)

BASE_LIB_OBJ = ${addprefix base/,$(BASE_LIB_SRC:.cpp=.o)}
DEPRECATED_LIB_OBJ = ${addprefix deprecated/,$(DEPRECATED_LIB_SRC:.cpp=.o)}
DEVEL_LIB_OBJ = ${addprefix devel/,$(DEVEL_LIB_SRC:.cpp=.o)}
ML_LIB_OBJ = ${addprefix ml/,$(ML_LIB_SRC:.cpp=.o)}
PGM_LIB_OBJ = ${addprefix pgm/,$(PGM_LIB_SRC:.cpp=.o)}
VISION_LIB_OBJ = ${addprefix vision/,$(VISION_LIB_SRC:.cpp=.o)}

CFLAGS += -I$(SVL_PATH)/lib/base -I$(SVL_PATH)/lib/deprecated \
	-I$(SVL_PATH)/lib/devel -I$(SVL_PATH)/lib/ml \
	-I$(SVL_PATH)/lib/pgm -I$(SVL_PATH)/lib/vision

.PHONY: clean
.PRECIOUS: $(LIB_OBJ)

all: depend $(STATICSVL) $(SHAREDSVL)

# static libraries
$(BIN_PATH)/libsvlBase.a: $(BASE_LIB_OBJ)
	ar rcs $@ $(BASE_LIB_OBJ)

$(BIN_PATH)/libsvlDeprecated.a: $(DEPRECATED_LIB_OBJ)
	ar rcs $@ $(DEPRECATED_LIB_OBJ)

$(BIN_PATH)/libsvlDevel.a: $(DEVEL_LIB_OBJ)
	ar rcs $@ $(DEVEL_LIB_OBJ)

$(BIN_PATH)/libsvlML.a: $(ML_LIB_OBJ)
	ar rcs $@ $(ML_LIB_OBJ)

$(BIN_PATH)/libsvlPGM.a: $(PGM_LIB_OBJ)
	ar rcs $@ $(PGM_LIB_OBJ)

$(BIN_PATH)/libsvlVision.a: $(VISION_LIB_OBJ)
	ar rcs $@ $(VISION_LIB_OBJ)

# shared libraries
$(BIN_PATH)/libsvlBase.so.1.0.0: $(BASE_LIB_OBJ)
	gcc ${CFLAGS} -shared -Wl,-soname,libsvlBase.so.1 -o $(BIN_PATH)/libsvlBase.so.1.0.0 $(BASE_LIB_OBJ)
	-ldconfig -n $(BIN_PATH)

$(BIN_PATH)/libsvlDeprecated.so.1.0.0: $(DEPRECATED_LIB_OBJ)
	gcc ${CFLAGS} -shared -Wl,-soname,libsvlDeprecated.so.1 -o $(BIN_PATH)/libsvlDeprecated.so.1.0.0 $(DEPRECATED_LIB_OBJ)
	-ldconfig -n $(BIN_PATH)

$(BIN_PATH)/libsvlDevel.so.1.0.0: $(DEVEL_LIB_OBJ)
	gcc ${CFLAGS} -shared -Wl,-soname,libsvlDevel.so.1 -o $(BIN_PATH)/libsvlDevel.so.1.0.0 $(DEVEL_LIB_OBJ)
	-ldconfig -n $(BIN_PATH)

$(BIN_PATH)/libsvlML.so.1.0.0: $(ML_LIB_OBJ)
	gcc ${CFLAGS} -shared -Wl,-soname,libsvlML.so.1 -o $(BIN_PATH)/libsvlML.so.1.0.0 $(ML_LIB_OBJ)
	-ldconfig -n $(BIN_PATH)

$(BIN_PATH)/libsvlPGM.so.1.0.0: $(PGM_LIB_OBJ)
	gcc ${CFLAGS} -shared -Wl,-soname,libsvlPGM.so.1 -o $(BIN_PATH)/libsvlPGM.so.1.0.0 $(PGM_LIB_OBJ)
	-ldconfig -n $(BIN_PATH)

$(BIN_PATH)/libsvlVision.so.1.0.0: $(VISION_LIB_OBJ)
	gcc ${CFLAGS} -shared -Wl,-soname,libsvlVision.so.1 -o $(BIN_PATH)/libsvlVision.so.1.0.0 $(VISION_LIB_OBJ)
	-ldconfig -n $(BIN_PATH)

# object files
.cpp.o:
	${CCC} ${CFLAGS} -c $< -o $@

depend:
	g++ ${CFLAGS} -MM $(LIB_SRC) >depend

clean:
	-rm $(LIB_OBJ)
	-rm $(STATICSVL)
	-rm $(SHAREDSVL)
	-rm depend

-include depend

