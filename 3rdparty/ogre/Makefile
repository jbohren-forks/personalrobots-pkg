all: ogre

OGRE_VERSION = ogre-v1-4-9
TARBALL = $(OGRE_VERSION).tar.bz2
TARBALL_URL = http://pr.willowgarage.com/downloads/$(TARBALL)
SOURCE_DIR = $(OGRE_VERSION)
UNPACK_CMD = tar xf
TARBALL_PATCH = warnings.patch
INITIAL_DIR = ogre
include $(shell rospack find mk)/download_unpack.mk

ROOT = $(shell rospack find ogre)
OGREROOT = $(shell rospack find ogre)/ogre

FIROOT = $(shell rospack find freeimage)/freeimage
CGROOT = $(shell rospack find Cg)/Cg

CFLAGS = $(shell rospack --lang=cpp --attrib=cflags export freeimage) \
         $(shell rospack --lang=cpp --attrib=cflags export player) \
         $(shell rospack --lang=cpp --attrib=cflags export opende) \
         $(shell rospack --lang=cpp --attrib=cflags export Cg)

LFLAGS = $(shell rospack --lang=cpp --attrib=lflags export freeimage) \
         $(shell rospack --lang=cpp --attrib=lflags export player) \
         $(shell rospack --lang=cpp --attrib=lflags export opende) \
         $(shell rospack --lang=cpp --attrib=lflags export Cg)

CONFIGURE_FLAGS = --with-arch=nocona --enable-release CXXFLAGS='-O3 -I$(FIROOT)/include -I$(CGROOT)/include' LDFLAGS='-Wl,-rpath,$(FIROOT)/lib,-rpath,$(CGROOT)/lib -L$(FIROOT)/lib -L$(CGROOT)/lib' CFLAGS='-O3 -I$(FIROOT)/include -I$(OISROOT)/include -I$(CGROOT)/include'

$(OGRE_VERSION)/Makefile: $(OGRE_VERSION)
	if test `uname` = Darwin ; then \
	  touch $(OGRE_VERSION)/Makefile; \
	else \
	  cd $(OGRE_VERSION) && \
	  export LIB="$(LFLAGS)" && \
	  export CFLAGS="$(CFLAGS)" && \
	  ./configure --prefix=$(OGREROOT) --enable-threading --with-pic --with-platform=GLX --with-gui=gtk  --disable-ogre-demos $(CONFIGURE_FLAGS); \
	  touch $(ROOT)/$(OGRE_VERSION)/Makefile; \
	  mkdir -p $(ROOT)/ogre; \
	  mkdir -p $(ROOT)/ogre/lib; \
	  mkdir -p $(ROOT)/ogre/include; \
	  mkdir -p $(ROOT)/ogre/share; \
	  mkdir -p $(ROOT)/ogre/bin; \
	fi

.PHONY: ogre
ogre: ogre/lib/pkgconfig/OGRE.pc

ogre/lib/pkgconfig/OGRE.pc: $(OGRE_VERSION)/Makefile $(OGRE_VERSION)
	# building OGRE
	if test `uname` = Darwin ; then \
	  cd $(ROOT)/$(OGRE_VERSION)/Mac/Ogre && \
	  export CPATH=Developer/SDKs/MacOSX10.5.sdk/usr/X11/include:/Developer/SDKs/MacOSX10.5.sdk/usr/X11/include/freetype2:`rospack find freeimage`/freeimage/include:/opt/local/include && \
	  export LIBRARY_PATH=`rospack find freeimage`/freeimage/lib:/opt/local/lib && \
	  xcodebuild ARCHS=i386 -target Ogre && \
	  xcodebuild ARCHS=i386 -target "GL Render System" && \
	  xcodebuild ARCHS=i386 -target "ParticleFX Plugin" && \
	  xcodebuild ARCHS=i386 -target "BSPSceneManager Plugin" && \
	  xcodebuild ARCHS=i386 -target "OctreeSceneManager Plugin" && \
	  xcodebuild ARCHS=i386 -target "Bundle Resources"; \
	else \
	  cd $(ROOT)/$(OGRE_VERSION) && make $(JOBC); \
	  cd $(ROOT)/$(OGRE_VERSION) && make install; \
	fi
	@echo "patch OGRE.pc to pass -Wl,-rpath,-L{libdir}"
	-mv ogre/lib/pkgconfig/OGRE.pc ogre/lib/pkgconfig/OGRE.bak
	sed 's%exec_prefix=$${prefix}%exec_prefix=$${prefix}\ncglibdir=$${prefix}/../../Cg/Cg/lib%g' ogre/lib/pkgconfig/OGRE.bak > ogre/lib/pkgconfig/OGRE.tmp
	sed 's%Libs: -L$${libdir} -lOgreMain%Libs: -Wl,-rpath,$${libdir},-rpath,$${libdir}/OGRE,-rpath,$${cglibdir} -L$${libdir} -lOgreMain%g' ogre/lib/pkgconfig/OGRE.tmp > ogre/lib/pkgconfig/OGRE.pc
	@echo "done"

.PHONY: clean wipe

clean: 
	-cd $(ROOT)/$(OGRE_VERSION) && make clean

wipe: 
	-rm -rf $(OGRE_VERSION)
	-rm -rf $(OGRE_VERSION).tar.bz2
	-rm -rf ogre

