all: ogre

ogre-v1-4-8.tar.bz2: 
	wget http://downloads.sourceforge.net/ogre/ogre-v1-4-8.tar.bz2

OGRE_VERSION = ogre-v1-4-8

download: ogre-v1-4-8.tar.bz2

$(OGRE_VERSION): $(OGRE_VERSION).tar.bz2
	tar xf $(OGRE_VERSION).tar.bz2
	mv ogre $(OGRE_VERSION)

ROOT = $(shell rospack find ogre)
OGREROOT = $(shell rospack find ogre)/ogre

FIROOT = $(shell rospack find freeimage)/freeimage
OISROOT = $(shell rospack find ois)/ois
CGROOT = $(shell rospack find Cg)/Cg

CFLAGS = $(shell rospack export/cpp/cflags freeimage) \
         $(shell rospack export/cpp/cflags player) \
         $(shell rospack export/cpp/cflags opende) \
         $(shell rospack export/cpp/cflags ois) \
         $(shell rospack export/cpp/cflags Cg)

LFLAGS = $(shell rospack export/cpp/lflags freeimage) \
         $(shell rospack export/cpp/lflags player) \
         $(shell rospack export/cpp/lflags opende) \
         $(shell rospack export/cpp/lflags ois) \
         $(shell rospack export/cpp/lflags Cg)

CONFIGURE_FLAGS = --with-arch=nocona --enable-release CXXFLAGS='-O3 -I$(FIROOT)/include -I$(OISROOT)/include -I$(CGROOT)/include' LDFLAGS=' -L$(FIROOT)/lib -L$(OISROOT)/lib -L$(CGROOT)/lib' CFLAGS='-O3 -I$(FIROOT)/include -I$(OISROOT)/include -I$(CGROOT)/include'

$(OGRE_VERSION)/Makefile: $(OGRE_VERSION)
	if [ `uname` = Darwin ]; then \
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

ogre: $(OGRE_VERSION)/Makefile
	# building OGRE
	if [ `uname` = Darwin ]; then \
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
	@echo "done"

.PHONY: clean wipe

clean: 
	-cd $(ROOT)/$(OGRE_VERSION) && make clean

wipe: 
	-rm -rf $(OGRE_VERSION)
	-rm -rf $(OGRE_VERSION).tar.bz2
	-rm -rf ogre

