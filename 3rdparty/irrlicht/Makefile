all: irrlicht


TARBALL = irrlicht-1.4.zip
TARBALL_URL = http://pr.willowgarage.com/downloads/irrlicht-1.4.zip
SOURCE_DIR = irrlicht-1.4
MD5SUM_FILE = irrlicht-1.4.zip.md5sum
UNPACK_CMD = unzip
include $(shell rospack find mk)/download_unpack.mk

.PHONY: irrlicht
irrlicht: $(SOURCE_DIR)
	@if [ ! -d /usr/include/X11 ]; then echo "you don't appear to have X. On Ubuntu, this is fixed with 'sudo apt-get install xorg-dev'"; false; fi  # This check doesn't fail correctly
	@if [ ! -f /usr/include/GL/gl.h -a ! -f /usr/X11/include/GL/gl.h ]; then echo "you don't appear to have OpenGL. On Ubuntu, this is fixed with 'sudo apt-get install libgl1-mesa-dev'"; false; fi
	@if [ ! -f /usr/include/GL/glu.h -a ! -f /usr/X11/include/GL/glu.h ]; then echo "you don't appear to have the OpenGL GLU library. On Ubuntu, this can be fixed with 'sudo apt-get install libglu1-mesa-dev'"; false; fi
	cd $(SOURCE_DIR)/source/Irrlicht && make
	touch irrlicht

clean:
	-rm -rf irrlicht $(SOURCE_DIR)

wipe: 
	rm -rf $(TARBALL)

.PHONY : clean
