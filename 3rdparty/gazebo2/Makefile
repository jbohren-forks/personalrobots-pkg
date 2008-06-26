all: gazebo2

REVISION = -r 6696

OGRE_PATH = $(shell rospack find ogre)/ogre
ODE_PATH = $(shell rospack find opende)/opende
ROOT = $(shell rospack find gazebo2)/gazebo

gazebo-checkout:
	@if [ ! -d gazebo-svn ]; then svn co $(REVISION) https://playerstage.svn.sf.net/svnroot/playerstage/code/gazebo/trunk gazebo-svn; fi
	-cd gazebo-svn && svn up $(REVISION) 

gazebo2: gazebo-checkout
	if [ ! -d gazebo ]; then mkdir -p gazebo; fi
	cd gazebo-svn && export PKG_CONFIG_PATH=${OGRE_PATH}/lib/pkgconfig:${PKG_CONFIG_PATH} && export PATH=${ODE_PATH}/bin:${PATH} && scons prefix=$(ROOT) && scons prefix=$(ROOT) install
	touch gazebo

clean:
	cd gazebo-svn && scons --clean
	rm -rf gazebo
wipe:
	rm -rf gazebo-svn gazebo

download: gazebo-checkout

