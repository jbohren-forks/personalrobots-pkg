all: irrlicht

irrlicht-1.4.zip: 
	wget -nc http://downloads.sourceforge.net/irrlicht/irrlicht-1.4.zip

download: irrlicht-1.4.zip

irrlicht-1.4: irrlicht-1.4.zip
	unzip -u irrlicht-1.4.zip

irrlicht: irrlicht-1.4
	@if [ ! -d /usr/include/X11 ]; then echo "you don't appear to have X. On Ubuntu, this is fixed with 'sudo apt-get install xorg-dev'"; false; fi  # This check doesn't fail correctly
	@if [ ! -f /usr/include/GL/gl.h -a ! -f /usr/X11/include/GL/gl.h ]; then echo "you don't appear to have OpenGL. On Ubuntu, this is fixed with 'sudo apt-get install libgl1-mesa-dev'"; false; fi
	@if [ ! -f /usr/include/GL/glu.h -a ! -f /usr/X11/include/GL/glu.h ]; then echo "you don't appear to have the OpenGL GLU library. On Ubuntu, this can be fixed with 'sudo apt-get install libglu1-mesa-dev'"; false; fi
	cd SDL-1.2.13 && ./configure --prefix=$(PWD)/SDL
	cd irrlicht-1.4/source/Irrlicht && make

clean:
	-rm -rf irrlicht irrlicht-1.4

wipe: 
	rm -rf  irrlicht-1.4.zip

.PHONY : clean
