all: installed

INSTALL_DIR = $(shell rospack find gazebo)/gazebo
TEST_INSTALL_DIR = $(shell rospack find gazebo)/test-gazebo

SVN_DIR = gazebo-svn
SVN_URL = https://playerstage.svn.sourceforge.net/svnroot/playerstage/code/gazebo/trunk
SVN_REVISION = -r 8232
SVN_PATCH =  gazebo_new_patch.diff
include $(shell rospack find mk)/svn_checkout.mk

OGRE_PKG = $(shell rospack find ogre)/ogre/lib/pkgconfig
PLAYER_PKG = $(shell rospack find player)/player/lib/pkgconfig
ODE_PKG = $(shell rospack find opende)/opende/lib/pkgconfig
ODE_PATH = $(shell rospack find opende)/opende/bin

BOOST_INCLUDE_DIRS = $(shell rosboost-cfg --include_dirs)
BOOST_LIBRARY_DIRS = $(shell rosboost-cfg --lib_dirs signals,thread)
BOOST_LIBRARIES = $(shell rosboost-cfg --lflags signals,thread)

BULLET_INCLUDE_DIRS   = $(shell rospack find bullet)/build/include
BULLET_DYNAMICS_DIRS  = $(shell rospack find bullet)/build/lib
BULLET_COLLISION_DIRS = $(shell rospack find bullet)/build/lib
BULLET_SOFTBODY_DIRS  = $(shell rospack find bullet)/build/lib
BULLET_MATH_DIRS      = $(shell rospack find bullet)/build/lib


PKG_PATHS = PKG_CONFIG_PATH=${ODE_PKG}:${PLAYER_PKG}:${OGRE_PKG}:${PKG_CONFIG_PATH}
PATHS = PATH=${ODE_PATH}:${PATH}

CMAKE = cmake
CMAKE_ARGS =	-D CMAKE_INSTALL_PREFIX=$(INSTALL_DIR) \
		-D boost_include_dirs=$(BOOST_INCLUDE_DIRS) \
		-D boost_library_dirs=$(BOOST_LIBRARY_DIRS) \
		-D boost_libraries="$(BOOST_LIBRARIES)" \
		-D CMAKE_BUILD_TYPE="release" \
		-D bullet_include_dirs=$(BULLET_INCLUDE_DIRS) \
		-D bullet_dynamics_dirs=$(BULLET_DYNAMICS_DIRS) \
		-D bullet_collision_dirs=$(BULLET_COLLISION_DIRS) \
		-D INCLUDE_PLAYER=off \
		-D bullet_softbody_dirs=$(BULLET_SOFTBODY_DIRS) \
		-D bullet_math_dirs=$(BULLET_MATH_DIRS) \
		-D CMAKE_CXX_LINK_FLAGS="-lfltk -lfltk_gl"\
		-D ENABLE_SHADOWS=ON

CMAKE_CMD = $(PKG_PATHS) $(PATHS) $(CMAKE) $(CMAKE_ARGS)

#build: SVN_UP_REVERT_PATCH gazebo
installed: $(SVN_DIR) patched
	if [ ! -d $(SVN_DIR)/build ]; then mkdir -p $(SVN_DIR)/build; fi
	cd $(SVN_DIR)/build && $(CMAKE_CMD) ..
	cd $(SVN_DIR)/build && $(PKG_PATHS) $(PATHS) make $(ROS_PARALLEL_JOBS) && make install
	rm -f $(SVN_DIR)/build/server/gazebo

clean:
	-cd $(SVN_DIR) && PKG_CONFIG_PATH=${ODE_PKG}:${PLAYER_PKG}:${OGRE_PKG}:${PKG_CONFIG_PATH} PATH=${ODE_PATH}:${PATH} make clean
	rm -rf gazebo installed
wipe: clean
	rm -rf $(SVN_DIR) gazebo


# these flags used to build opende
# ipo seems to break ogre and opende, use ip instead
#INTEL_OPTIONS = CC='/opt/intel/cc/10.1.008/bin/icc' CXX='/opt/intel/cc/10.1.008/bin/icpc' LD='/opt/intel/cc/10.1.008/bin/xild'
INTEL_C_FLAGS     =  -march=core2 -msse3 -ip  -no-prec-div         -parallel -O3 -fPIC -I$(ROOT)/include
INTEL_C_FLAGS_OMP =  -march=core2 -msse3 -ip  -no-prec-div -openmp -parallel -O3 -fPIC -I$(ROOT)/include
INTEL_LD_FLAGS     = -Wl,-rpath,/opt/intel/cc/10.1.008/lib,-rpath,$(ROOT)/lib -L/opt/intel/cc/10.1.008/lib -L$(ROOT)/lib -lguide -lcxaguard -limf -lsvml -lirc -pthread -lintlc
INTEL_LD_FLAGS_OMP = -Wl,-rpath,/opt/intel/cc/10.1.008/lib,-rpath,$(ROOT)/lib -L/opt/intel/cc/10.1.008/lib -L$(ROOT)/lib -lguide -lcxaguard -limf -lsvml -lirc -pthread -lintlc -lompstub -liomp5 -lomp_db

