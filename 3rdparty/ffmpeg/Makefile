all: ffmpeg

ffmpeg-export-2008-03-18:
	tar xjf ffmpeg-export-snapshot.tar.bz2

ffmpeg: ffmpeg-export-2008-03-18
	cd ffmpeg-export-2008-03-18 && ./configure --prefix=$(PWD)/ffmpeg
	cd ffmpeg-export-2008-03-18 && make && make install
	mkdir ffmpeg/include/ffmpeg
	cd ffmpeg/include && ln -s libavcodec/avcodec.h avcodec.h
	cd ffmpeg/include && ln -s libavformat/avformat.h avformat.h

clean:
	-rm -rf ffmpeg ffmpeg-export-2008-03-18

.PHONY : clean
