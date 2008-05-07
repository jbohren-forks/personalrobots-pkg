all: ffmpeg

ffmpeg-2008-03-18:
	svn export --revision {2008-03-18} svn://svn.mplayerhq.hu/ffmpeg/trunk ffmpeg-2008-03-18

ffmpeg: ffmpeg-2008-03-18
	cd ffmpeg-2008-03-18 && ./configure --prefix=$(PWD)/ffmpeg --enable-shared
	cd ffmpeg-2008-03-18 && make && make install
	mkdir ffmpeg/include/ffmpeg
	cd ffmpeg/include/ffmpeg && ln -s ../libavcodec/avcodec.h avcodec.h
	cd ffmpeg/include/ffmpeg && ln -s ../libavformat/avformat.h avformat.h

clean:
	-rm -rf ffmpeg 
	@echo "Local build products cleaned, but the SVN export from ffmpeg's repository was retained. If you want to delete that too, do 'make superclean'"

superclean:
	-rm -rf ffmpeg-2008-03-18

.PHONY : clean
