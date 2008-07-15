all: gazebo

REVISION = -r 6870

OGRE_PKG = $(shell rospack find ogre)/ogre/lib/pkgconfig
PLAYER_PKG = $(shell rospack find player)/player/lib/pkgconfig
ODE_PATH = $(shell rospack find opende)/opende
#FI_LD = $(shell rospack export/cpp/lflags freeimage)
FI_LD = $(shell rospack find freeimage)/freeimage/lib
ROOT = $(shell rospack find gazebo)/gazebo
TESTROOT = $(shell rospack find gazebo)/test-gazebo

gazebo-checkout:
	@if [ ! -d gazebo-svn ]; then svn co $(REVISION) https://playerstage.svn.sf.net/svnroot/playerstage/code/gazebo/trunk gazebo-svn; fi
	-cd gazebo-svn && svn up $(REVISION) 
	-svn diff gazebo-svn > tmp.diff
	-cd gazebo-svn && svn revert -R *
	-patch -N -p0 < gazebo_patch.diff

gazebo: gazebo-checkout
	if [ ! -d gazebo ]; then mkdir -p gazebo; fi
	cd gazebo-svn && export PKG_CONFIG_PATH=${PLAYER_PKG}:${OGRE_PKG}:${PKG_CONFIG_PATH} && \
	export LIBPATH="${FI_LD}" && \
	export PATH=${ODE_PATH}/bin:${PATH} && scons prefix=$(ROOT) && scons prefix=$(ROOT) install
	touch gazebo

test-gazebo-checkout:
	@if [ ! -d test-gazebo-svn ]; then svn co $(REVISION) https://playerstage.svn.sf.net/svnroot/playerstage/code/gazebo/trunk test-gazebo-svn; fi
	-cd test-gazebo-svn && svn up #$(REVISION) 
	#-patch -N -p0 < gazebo_patch.diff

test-gazebo: test-gazebo-checkout
	if [ ! -d test-gazebo ]; then mkdir -p test-gazebo; fi
	cd test-gazebo-svn && export PKG_CONFIG_PATH=${PLAYER_PKG}:${OGRE_PKG}:${PKG_CONFIG_PATH} && \
	export LIBPATH="${FI_LD}" && \
	export PATH=${ODE_PATH}/bin:${PATH} && scons prefix=$(TESTROOT) && scons prefix=$(TESTROOT) install
	touch test-gazebo

#	export LD_LIBRARY_PATH="/usr/lib" && \

clean:
	cd gazebo-svn && scons --clean
	rm -rf gazebo
wipe:
	rm -rf gazebo-svn gazebo

download: gazebo-checkout

