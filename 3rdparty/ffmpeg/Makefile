all: ffmpeg

SVN_DIR = ffmpeg-latest
SVN_URL = svn://svn.mplayerhq.hu/ffmpeg/trunk 
SVN_REVISION =
include $(shell rospack find mk)/svn_checkout.mk

ffmpeg: SVN_UP
	cd $(SVN_DIR) && ./configure --prefix=$(PWD)/ffmpeg --enable-shared
	cd $(SVN_DIR) && make && make install
	mkdir -p ffmpeg/include/ffmpeg
	cd ffmpeg/include/ffmpeg && ln -sf ../libavcodec/avcodec.h avcodec.h
	cd ffmpeg/include/ffmpeg && ln -sf ../libavformat/avformat.h avformat.h

clean:
	-rm -rf ffmpeg 

wipe: clean
	rm -rf $(SVN_DIR)

.PHONY : clean wipe ffmpeg
