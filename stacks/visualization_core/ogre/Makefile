all: ogre

OGRE_VERSION = ogre-v1-6-3
TARBALL = build/$(OGRE_VERSION).tar.bz2
#TARBALL_URL = http://superb-west.dl.sourceforge.net/sourceforge/ogre/$(OGRE_VERSION).tar.bz2
TARBALL_URL = http://pr.willowgarage.com/downloads/$(OGRE_VERSION).tar.bz2
SOURCE_DIR = build/$(OGRE_VERSION)
UNPACK_CMD = tar xf
#TARBALL_PATCH = resize.diff
INITIAL_DIR = build/ogre
include $(shell rospack find mk)/download_unpack_build.mk

ROOT = $(shell rospack find ogre)
OGREROOT = $(shell rospack find ogre)/ogre

CFLAGS = -g -O3 \
         $(shell rosboost-cfg --cflags)

LFLAGS = $(shell rosboost-cfg --lflags thread) \
         -lpthread

CONFIGURE_FLAGS = --prefix=$(OGREROOT) --with-pic --with-platform=GLX --with-gui=gtk  --disable-ogre-demos --with-arch=nocona --enable-release

$(SOURCE_DIR)/Makefile: $(SOURCE_DIR)/unpacked changelist.txt
	make clean
	if test `uname` = Darwin ; then \
	  touch $(SOURCE_DIR)/Makefile; \
	else \
	  cd $(SOURCE_DIR) && \
	  export LDFLAGS="$(LFLAGS)" && \
	  export CFLAGS="$(CFLAGS)" && \
	  export CXXFLAGS="$(CFLAGS)" && \
	  ./configure $(CONFIGURE_FLAGS) && \
	  touch $(ROOT)/$(SOURCE_DIR)/Makefile && \
	  mkdir -p $(ROOT)/ogre && \
	  mkdir -p $(ROOT)/ogre/lib && \
	  mkdir -p $(ROOT)/ogre/include && \
	  mkdir -p $(ROOT)/ogre/share && \
	  mkdir -p $(ROOT)/ogre/bin; \
	fi

.PHONY: ogre
# This target tries to be smart about checking for a system install of OGRE
# at a particular version.  If it finds the right version, it creates a
# local symlink into /usr/lib/OGRE, on the assumption that the OGRE plugin
# .so's are in there.  If it's not found, then we build OGRE from source
# here.
#
#ogre: ogre/lib/pkgconfig/OGRE.pc
ogre:
	@if test `uname` = Darwin ; then \
	  echo "OS X detected; assuming that you have installed the OGRE SDK"; \
	elif ! PKG_CONFIG_PATH= pkg-config --exact-version 1.6.1 OGRE; then \
	  echo "You don't have OGRE locally.  I'm going to build it in ros-pkg."; \
	  if [ -h ogre/lib/OGRE ]; then \
	    rm ogre/lib/OGRE; \
	  fi; \
	  make ogre/lib/pkgconfig/OGRE.pc; \
	else \
	  echo "You have OGRE 1.6.1 locally, making a soft link to the system version."; \
	  rm -rf ogre; \
	  mkdir -p ogre/lib; \
	  ln -sf `pkg-config --variable=prefix OGRE`/lib/OGRE ogre/lib/OGRE; \
	fi

ogre/lib/pkgconfig/OGRE.pc: $(SOURCE_DIR)/Makefile $(SOURCE_DIR)
	# building OGRE
	if test `uname` = Darwin ; then \
	  cd $(ROOT)/$(SOURCE_DIR)/Mac/Ogre && \
	  export CPATH=Developer/SDKs/MacOSX10.5.sdk/usr/X11/include:/Developer/SDKs/MacOSX10.5.sdk/usr/X11/include/freetype2:/opt/local/include && \
	  export LIBRARY_PATH=/opt/local/lib && \
	  xcodebuild ARCHS=i386 -target Ogre && \
	  xcodebuild ARCHS=i386 -target "GL Render System" && \
	  xcodebuild ARCHS=i386 -target "ParticleFX Plugin" && \
	  xcodebuild ARCHS=i386 -target "BSPSceneManager Plugin" && \
	  xcodebuild ARCHS=i386 -target "OctreeSceneManager Plugin" && \
	  xcodebuild ARCHS=i386 -target "Bundle Resources"; \
	else \
	  cd $(ROOT)/$(SOURCE_DIR) && make $(JOBC) $(ROS_PARALLEL_JOBS); \
	  cd $(ROOT)/$(SOURCE_DIR) && make install; \
	fi
	@echo "patch OGRE.pc to pass -Wl,-rpath,-L{libdir}"
	-mv ogre/lib/pkgconfig/OGRE.pc ogre/lib/pkgconfig/OGRE.bak
#	sed 's%exec_prefix=$${prefix}%exec_prefix=$${prefix}\n' ogre/lib/pkgconfig/OGRE.bak > ogre/lib/pkgconfig/OGRE.tmp
	sed 's%Libs: -L$${libdir} -lOgreMain%Libs: -Wl,-rpath,$${libdir},-rpath,$${libdir}/OGRE -L$${libdir} -lOgreMain%g' ogre/lib/pkgconfig/OGRE.bak > ogre/lib/pkgconfig/OGRE.pc
	@echo "done"

.PHONY: clean wipe

clean: 
	-cd $(ROOT)/$(SOURCE_DIR) && make clean
	-rm -rf ogre

wipe: 
	-rm -rf build


