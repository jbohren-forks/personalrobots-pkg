INSTALL_DIR = opencv

all: installed

PKG_NAME 	= svl-src-1.1-1564
TARBALL     = build/$(PKG_NAME).tar.gz
TARBALL_URL = http://ai.stanford.edu/~sgould/svl/media/$(PKG_NAME).tar.gz
UNPACK_CMD  = tar -xvzf 
SOURCE_DIR  = build/$(PKG_NAME)

include $(shell rospack find mk)/download_unpack_build.mk

installed: wiped $(SOURCE_DIR)/unpacked
	echo "Building STAIR Vision Library..."
	# it seems that SVL does not build with the wxGTK included in ubuntu 8.04, os we are compiling this version specially for it
	(cd build && wget "http://prdownloads.sourceforge.net/wxwindows/wxGTK-2.8.7.tar.gz" && tar xzvf wxGTK-2.8.7.tar.gz && rm wxGTK-2.8.7.tar.gz && cd wxGTK-2.8.7 && mkdir buildgtk && cd buildgtk && ../configure --disable-shared --with-opengl --enable-monolithic && make)
	ln -sf ../../wxGTK-2.8.7 $(SOURCE_DIR)/external/wxGTK
	ln -sf $(shell rospack find opencv_latest)/opencv $(SOURCE_DIR)/external
	(cd $(SOURCE_DIR) && make)
	(cd build && ln -sf $(PKG_NAME) svl)
	(cd $(SOURCE_DIR)/external/lbfgs && ln -sf lbfgs.a liblbfgs.a)
	(cd $(SOURCE_DIR)/external/xmlParser && ln -sf xmlParser.a libxmlParser.a)
	touch installed

wiped: Makefile
	make wipe
	touch wiped

wipe:
	rm -rf build installed

.PHONY : clean download

