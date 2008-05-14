all: opencv

opencv-1.0.0.tar.gz: 
	wget http://internap.dl.sourceforge.net/sourceforge/opencvlibrary/opencv-1.0.0.tar.gz

opencv-1.0.0: opencv-1.0.0.tar.gz
	tar xzf opencv-1.0.0.tar.gz

download: opencv-1.0.0.tar.gz

FFMPEG := $(shell rospack find ffmpeg)
opencv: opencv-1.0.0
	export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${FFMPEG}/ffmpeg/lib && cd opencv-1.0.0 && ./configure --without-quicktime --with-ffmpeg --prefix=$(PWD)/opencv CFLAGS="-I${FFMPEG}/ffmpeg/include -I${FFMPEG}/ffmpeg/include/libavutils -I${FFMPEG}/include/libavformat" CPPFLAGS="-I${FFMPEG}/ffmpeg/include -I${FFMPEG}/ffmpeg/include/libavutils -I${FFMPEG}/ffmpeg/include/libavformat" LDFLAGS=-L${FFMPEG}/ffmpeg/lib --with-gnu-ld --with-x
# hack up the opencv <=> ffmpeg interface so we can use the latest ffmpeg
	cd opencv-1.0.0/otherlibs/highgui && patch -N -p0 <../../../cvcap_ffmpeg.patch ; true
	export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${FFMPEG}/ffmpeg/lib && cd opencv-1.0.0 && make && make install

clean:
	-rm -rf opencv opencv-1.0.0 opencv-1.0.0.tar.gz

.PHONY : clean download
