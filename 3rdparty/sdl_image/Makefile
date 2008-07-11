all: SDL_image

SDL_image-1.2.6.tar.gz:
	wget http://www.libsdl.org/projects/SDL_image/release/SDL_image-1.2.6.tar.gz

download: SDL_image-1.2.6.tar.gz

SDL_image-1.2.6: SDL_image-1.2.6.tar.gz
	#md5sum -c SDL_image-1.2.6.tar.gz.md5sum
	tar xzf SDL_image-1.2.6.tar.gz

SDL_image: SDL_image-1.2.6
	cd SDL_image-1.2.6 && ./configure --prefix=$(PWD)/SDL_image --with-sdl-prefix="$(shell rospack find sdl)/SDL" CPPFLAGS="-I$(shell rospack find ijg_libjpeg)/include" LDFLAGS="-L$(shell rospack find ijg_libjpeg)/lib"
	cd SDL_image-1.2.6 && make && make install
# change the include paths so it works with our packaging system...
	sed -e 's:SDL.h:SDL/SDL.h:' -e 's:SDL_version.h:SDL/SDL_version.h:' -e 's:begin_code.h:SDL/begin_code.h:' -e 's:close_code.h:SDL/close_code.h:' <SDL_image/include/SDL/SDL_image.h >SDL_image/include/SDL/SDL_image_mod.h
	mv SDL_image/include/SDL/SDL_image_mod.h SDL_image/include/SDL/SDL_image.h

clean:
	-rm -rf SDL_image-1.2.6 SDL_image

wipe: clean
	rm -rf SDL_image-1.2.6.tar.gz 

.PHONY : clean wipe
