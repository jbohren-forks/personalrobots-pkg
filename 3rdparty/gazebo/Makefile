all: gazebo

SVN_DIR = gazebo-svn
SVN_URL = https://playerstage.svn.sourceforge.net/svnroot/playerstage/code/gazebo/trunk
SVN_REVISION = -r 6915 #revision with wall extrusion working, Ogre camera save frame (TP)
SVN_PATCH = gazebo_patch.diff
include $(shell rospack find mk)/svn_checkout.mk


OGRE_PKG = $(shell rospack find ogre)/ogre/lib/pkgconfig
PLAYER_PKG = $(shell rospack find player)/player/lib/pkgconfig
ODE_PATH = $(shell rospack find opende)/opende
FI_PATH = $(shell rospack find freeimage)
FI_LD = $(shell rospack find freeimage)/freeimage/lib
ROOT = $(shell rospack find gazebo)/gazebo
TESTROOT = $(shell rospack find gazebo)/test-gazebo

.PHONY: gazebo
gazebo: SVN_UP
	if [ ! -d gazebo ]; then mkdir -p gazebo; mkdir -p gazebo/share; mkdir -p gazebo/share/gazebo; mkdir -p gazebo/share/gazebo/Media; fi
	cd $(SVN_DIR) && PKG_CONFIG_PATH=${FI_PATH}:${PLAYER_PKG}:${OGRE_PKG}:${PKG_CONFIG_PATH} \
	LIBPATH="${FI_LD}:`rospack find opende`/opende/lib" scons prefix=$(ROOT) 
	-cd $(SVN_DIR) && PKG_CONFIG_PATH=${FI_PATH}:${PLAYER_PKG}:${OGRE_PKG}:${PKG_CONFIG_PATH} \
	LIBPATH="${FI_LD}:`rospack find opende`/opende/lib" scons prefix=$(ROOT) install

test-$(SVN_TARGET):
	@if [ ! -d test-$(SVN_DIR) ]; then svn co $(SVN_REVISION) https://playerstage.svn.sourceforge.net/svnroot/playerstage/code/gazebo/trunk test-$(SVN_DIR); fi
	-cd test-$(SVN_DIR) && svn up #$(REVISION) 
	#-patch -N -p0 < gazebo_patch.diff

test-gazebo: test-$(SVN_TARGET)
	if [ ! -d test-gazebo ]; then mkdir -p test-gazebo; fi
	cd test-$(SVN_DIR) && export PKG_CONFIG_PATH=${PLAYER_PKG}:${OGRE_PKG}:${PKG_CONFIG_PATH} && \
	export LIBPATH="${FI_LD}" && \
	export PATH=${ODE_PATH}/bin:${PATH} && scons prefix=$(TESTROOT) && scons prefix=$(TESTROOT) install
	touch test-gazebo

#	export LD_LIBRARY_PATH="/usr/lib" && \

clean:
	-cd $(SVN_DIR) && scons --clean
	rm -rf gazebo
wipe: clean
	rm -rf $(SVN_DIR) gazebo
