all: opencv

opencv-1.0.0.tar.gz: 
	wget http://internap.dl.sourceforge.net/sourceforge/opencvlibrary/opencv-1.0.0.tar.gz

opencv-1.0.0: opencv-1.0.0.tar.gz
	md5sum -c opencv-1.0.0.tar.gz.md5sum
	tar xzf opencv-1.0.0.tar.gz

download: opencv-1.0.0.tar.gz

ffmpeg:
	@if [ ! -d `rospack find ffmpeg`/ffmpeg ]; then rosmake ffmpeg; fi

FFMPEG := $(shell rospack find ffmpeg)

#opencv: ffmpeg opencv-1.0.0
opencv: opencv-1.0.0
	@echo "making it"
	@if [ ! -d /usr/include/gtk-2.0 ]; then echo "you don't appear to have the GTK+2.0 headers. On Ubuntu, this is fixed with 'sudo apt-get install libgtk2.0-dev'"; false; fi
	export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${FFMPEG}/ffmpeg/lib && cd opencv-1.0.0 && ./configure --without-quicktime --with-ffmpeg --prefix=$(PWD)/opencv CFLAGS="-I${FFMPEG}/ffmpeg/include -I${FFMPEG}/ffmpeg/include/libavutils -I${FFMPEG}/include/libavformat" CPPFLAGS="-I${FFMPEG}/ffmpeg/include -I${FFMPEG}/ffmpeg/include/libavutils -I${FFMPEG}/ffmpeg/include/libavformat" LDFLAGS=-L${FFMPEG}/ffmpeg/lib --with-gnu-ld --with-x
# hack up the opencv <=> ffmpeg interface so we can use the latest ffmpeg
	cd opencv-1.0.0/otherlibs/highgui && patch -N -p0 <../../../cvcap_ffmpeg.patch ; true
	export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${FFMPEG}/ffmpeg/lib && cd opencv-1.0.0 && make && make install
	touch opencv

clean:
	-rm -rf opencv opencv-1.0.0 

wipe: clean
	rm -rf opencv-1.0.0.tar.gz

.PHONY : clean download ffmpeg
