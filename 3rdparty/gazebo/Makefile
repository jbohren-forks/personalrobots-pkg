#all: gazebo
all: check_svn

BUILD_TARGET = gazebo
SVN_TARGET = gazebo-checkout
SVN_DIR = gazebo-svn
include $(shell rospack find mk)/check_svn.mk

REVISION = -r 6897

OGRE_PKG = $(shell rospack find ogre)/ogre/lib/pkgconfig
PLAYER_PKG = $(shell rospack find player)/player/lib/pkgconfig
ODE_PATH = $(shell rospack find opende)/opende
#FI_LD = $(shell rospack export/cpp/lflags freeimage)
FI_LD = $(shell rospack find freeimage)/freeimage/lib
ROOT = $(shell rospack find gazebo)/gazebo
TESTROOT = $(shell rospack find gazebo)/test-gazebo

$(SVN_TARGET):
	@if [ ! -d $(SVN_DIR) ]; then svn co $(REVISION) https://playerstage.svn.sourceforge.net/svnroot/playerstage/code/gazebo/trunk $(SVN_DIR); fi
	-cd $(SVN_DIR) && svn up $(REVISION) 
	-svn diff $(SVN_DIR) > tmp.diff
	-cd $(SVN_DIR) && svn revert -R *
	-patch -N -p0 < gazebo_patch.diff

.PHONY: $(BUILD_TARGET)
$(BUILD_TARGET): $(SVN_TARGET)
	if [ ! -d $(BUILD_TARGET) ]; then mkdir -p $(BUILD_TARGET); mkdir -p $(BUILD_TARGET)/share; mkdir -p $(BUILD_TARGET)/share/gazebo; mkdir -p $(BUILD_TARGET)/share/gazebo/Media; fi
	cd $(SVN_DIR) && export PKG_CONFIG_PATH=${PLAYER_PKG}:${OGRE_PKG}:${PKG_CONFIG_PATH} && \
	export LIBPATH="${FI_LD}" && \
	export PATH=${ODE_PATH}/bin:${PATH} && scons prefix=$(ROOT) && scons prefix=$(ROOT) install
        # Gazebo's install appears to be broken.  It actually works, but
        # throws an error at the end.  So we ignore it.
        #-scons prefix=$(ROOT) install
        #scons prefix=$(ROOT) install

test-$(SVN_TARGET):
	@if [ ! -d test-$(SVN_DIR) ]; then svn co $(REVISION) https://playerstage.svn.sourceforge.net/svnroot/playerstage/code/gazebo/trunk test-$(SVN_DIR); fi
	-cd test-$(SVN_DIR) && svn up #$(REVISION) 
	#-patch -N -p0 < gazebo_patch.diff

test-gazebo: test-$(SVN_TARGET)
	if [ ! -d test-gazebo ]; then mkdir -p test-gazebo; fi
	cd test-$(SVN_DIR) && export PKG_CONFIG_PATH=${PLAYER_PKG}:${OGRE_PKG}:${PKG_CONFIG_PATH} && \
	export LIBPATH="${FI_LD}" && \
	export PATH=${ODE_PATH}/bin:${PATH} && scons prefix=$(TESTROOT) && scons prefix=$(TESTROOT) install
	touch test-gazebo

#	export LD_LIBRARY_PATH="/usr/lib" && \

clean: check_svn_clean
	-cd $(SVN_DIR) && scons --clean
	rm -rf gazebo
wipe: clean
	rm -rf $(SVN_DIR) gazebo

download: $(SVN_TARGET)

