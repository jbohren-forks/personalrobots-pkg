all: check_done

BUILD_TARGET = SDL
include $(shell rospack find mk)/check_done.mk

TARBALL = SDL-1.2.13.tar.gz
TARBALL_URL = http://libsdl.org/release/SDL-1.2.13.tar.gz
SOURCE_DIR = SDL-1.2.13
MD5SUM_FILE = SDL-1.2.13.tar.gz.md5sum
UNPACK_CMD = tar xzf
include $(shell rospack find mk)/download_unpack.mk

.PHONY: SDL
SDL: $(SOURCE_DIR)
	@if [ ! -d /usr/include/X11 ]; then echo "you don't appear to have X. On Ubuntu, this is fixed with 'sudo apt-get install xorg-dev'"; false; fi ## THis check doens't fail correctly
	@if [ ! -f /usr/include/GL/gl.h -a ! -f /usr/X11/include/GL/gl.h ]; then echo "you don't appear to have OpenGL. On Ubuntu, this is fixed with 'sudo apt-get install libgl1-mesa-dev'"; false; fi
	@if [ ! -f /usr/include/GL/glu.h -a ! -f /usr/X11/include/GL/glu.h ]; then echo "you don't appear to have the OpenGL GLU library. On Ubuntu, this can be fixed with 'sudo apt-get install libglu1-mesa-dev'"; false; fi
	cd SDL-1.2.13 && ./configure --prefix=$(PWD)/SDL
	cd SDL-1.2.13 && make && make install

clean: check_done_clean
	-rm -rf SDL-1.2.13 SDL 

wipe: clean
	rm -rf SDL-1.2.13.tar.gz

.PHONY : clean wipe
