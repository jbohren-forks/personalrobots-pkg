all: ffmpeg

ffmpeg-latest:
	svn export svn://svn.mplayerhq.hu/ffmpeg/trunk ffmpeg-latest

download: ffmpeg-latest

ffmpeg: ffmpeg-latest
	cd ffmpeg-latest && ./configure --prefix=$(PWD)/ffmpeg --enable-shared
	cd ffmpeg-latest && make && make install
	mkdir ffmpeg/include/ffmpeg
	cd ffmpeg/include/ffmpeg && ln -s ../libavcodec/avcodec.h avcodec.h
	cd ffmpeg/include/ffmpeg && ln -s ../libavformat/avformat.h avformat.h

clean:
	-rm -rf ffmpeg 

wipe: clean
	rm -rf ffmpeg-latest

.PHONY : clean wipe
