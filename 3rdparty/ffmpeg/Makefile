all: build

SVN_DIR = ffmpeg-latest
SVN_URL = svn://svn.mplayerhq.hu/ffmpeg/trunk 
SVN_REVISION = -r 14758
include $(shell rospack find mk)/svn_checkout.mk

build: SVN_UP ffmpeg

configured: $(SVN_DIR) Makefile
	cd $(SVN_DIR) && ./configure --prefix=$(PWD)/ffmpeg --enable-shared
	touch configured

.PHONY: ffmpeg
ffmpeg: configured
	cd $(SVN_DIR) && make $(PR2_JOBS) && make install
	mkdir -p ffmpeg/include/ffmpeg
	cd ffmpeg/include/ffmpeg && ln -sf ../libavcodec/avcodec.h avcodec.h
	cd ffmpeg/include/ffmpeg && ln -sf ../libavformat/avformat.h avformat.h
	touch configured

clean:
	-cd $(SVN_DIR) && make clean
	rm -rf ffmpeg configured

wipe: clean
	rm -rf $(SVN_DIR)

.PHONY : clean wipe
