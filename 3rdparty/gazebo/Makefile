all: build

SVN_DIR = gazebo-svn-ogre-v1.4.9
#SVN_URL = https://playerstage.svn.sourceforge.net/svnroot/playerstage/code/gazebo/trunk
SVN_URL = https://playerstage.svn.sourceforge.net/svnroot/playerstage/code/gazebo/branches/ogre-1.4.9
SVN_REVISION = -r 7168
SVN_PATCH = gazebo_patch.diff
include $(shell rospack find mk)/svn_checkout.mk


OGRE_PKG = $(shell rospack find ogre)/ogre/lib/pkgconfig
PLAYER_PKG = $(shell rospack find player)/player/lib/pkgconfig
FI_PKG = $(shell rospack find freeimage)/freeimage/lib/pkgconfig
ODE_PKG = $(shell rospack find opende)/opende/lib/pkgconfig
CV_PKG = $(shell rospack find opencv_latest)/opencv/lib/pkgconfig

ODE_PATH = $(shell rospack find opende)/opende/bin
FI_LD = $(shell rospack find freeimage)/freeimage/lib
ROOT = $(shell rospack find gazebo)/gazebo
TESTROOT = $(shell rospack find gazebo)/test-gazebo

BOOST_CFLAGS = $(shell rosboost-cfg --cflags)
BOOST_LFLAGS = $(shell rosboost-cfg --lflags signals,thread)

build: SVN_UP_REVERT_PATCH gazebo

.PHONY: gazebo
gazebo: $(SVN_DIR)
	if [ ! -d gazebo ]; then mkdir -p gazebo; mkdir -p gazebo/share; mkdir -p gazebo/share/gazebo; mkdir -p gazebo/share/gazebo/worlds; mkdir -p gazebo/share/gazebo/Media; fi
	cd $(SVN_DIR) && PKG_CONFIG_PATH=${CV_PKG}:${ODE_PKG}:${FI_PKG}:${PLAYER_PKG}:${OGRE_PKG}:${PKG_CONFIG_PATH} PATH=${ODE_PATH}:${PATH} \
	scons mode=optimized prefix=$(ROOT) boost_cflags="$(BOOST_CFLAGS)" boost_lflags="$(BOOST_LFLAGS)"
	-cd $(SVN_DIR) && PKG_CONFIG_PATH=${CV_PKG}:${ODE_PKG}:${FI_PKG}:${PLAYER_PKG}:${OGRE_PKG}:${PKG_CONFIG_PATH} PATH=${ODE_PATH}:${PATH} \
	scons mode=optimized prefix=$(ROOT) boost_cflags="$(BOOST_CFLAGS)" boost_lflags="$(BOOST_LFLAGS)" install
	touch gazebo

ROOT_ORIG = $(shell rospack find gazebo)/gazebo-original
SVN_ORIG_DIR = gazebo-svn-original
.PHONY: gazebo-original
gazebo-original: $(SVN_ORIG_DIR)
	if [ ! -d gazebo-original ]; then mkdir -p gazebo-original; mkdir -p gazebo-original/share; mkdir -p gazebo-original/share/gazebo/worlds; mkdir -p gazebo-original/share/gazebo; mkdir -p gazebo-original/share/gazebo/Media; fi
	cd $(SVN_ORIG_DIR) && PKG_CONFIG_PATH=${CV_PKG}:${ODE_PKG}:${FI_PKG}:${PLAYER_PKG}:${OGRE_PKG}:${PKG_CONFIG_PATH} PATH=${ODE_PATH}:${PATH} \
	scons prefix=$(ROOT_ORIG) 
	-cd $(SVN_ORIG_DIR) && PKG_CONFIG_PATH=${CV_PKG}:${ODE_PKG}:${FI_PKG}:${PLAYER_PKG}:${OGRE_PKG}:${PKG_CONFIG_PATH} PATH=${ODE_PATH}:${PATH} \
	scons prefix=$(ROOT_ORIG) install
	touch gazebo-original

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
	-cd $(SVN_DIR) && PKG_CONFIG_PATH=${CV_PKG}:${ODE_PKG}:${FI_PKG}:${PLAYER_PKG}:${OGRE_PKG}:${PKG_CONFIG_PATH} PATH=${ODE_PATH}:${PATH} scons --clean
	rm -rf gazebo
wipe: clean
	rm -rf $(SVN_DIR) gazebo
