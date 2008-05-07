all: ffmpeg

ffmpeg-latest:
	svn export svn://svn.mplayerhq.hu/ffmpeg/trunk ffmpeg-latest

ffmpeg: ffmpeg-latest
	cd ffmpeg-latest && ./configure --prefix=$(PWD)/ffmpeg --enable-shared
	cd ffmpeg-latest && make && make install
	mkdir ffmpeg/include/ffmpeg
	cd ffmpeg/include/ffmpeg && ln -s ../libavcodec/avcodec.h avcodec.h
	cd ffmpeg/include/ffmpeg && ln -s ../libavformat/avformat.h avformat.h

clean:
	-rm -rf ffmpeg 
	@echo "Local build products cleaned, but the SVN export from ffmpeg's repository was retained. If you want to delete that too, do 'make superclean'"

superclean:
	-rm -rf ffmpeg-latest

.PHONY : clean
