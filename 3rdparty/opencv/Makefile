all: opencv

TARBALL = build/opencv-1.0.0.tar.gz
TARBALL_URL = http://pr.willowgarage.com/downloads/opencv-1.0.0.tar.gz
SOURCE_DIR = build/opencv-1.0.0
MD5SUM_FILE = opencv-1.0.0.tar.gz.md5sum
UNPACK_CMD = tar xzf
include $(shell rospack find mk)/download_unpack_build.mk

ffmpeg:
	@if [ ! -d `rospack find ffmpeg`/ffmpeg ]; then rosmake ffmpeg; fi

FFMPEG := $(shell rospack find ffmpeg)

configured: $(SOURCE_DIR) Makefile
	@if [ ! -d /usr/include/gtk-2.0 ]; then echo "you don't appear to have the GTK+2.0 headers. On Ubuntu, this is fixed with 'sudo apt-get install libgtk2.0-dev'"; false; fi
	patch -N -p0 <../opencv.patch ; true
	export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${FFMPEG}/ffmpeg/lib && cd $(SOURCE_DIR) && ./configure --without-quicktime --with-ffmpeg --prefix=$(PWD)/opencv CFLAGS="-I${FFMPEG}/ffmpeg/include -I${FFMPEG}/ffmpeg/include/libavutils -I${FFMPEG}/include/libavformat" CPPFLAGS="-I${FFMPEG}/ffmpeg/include -I${FFMPEG}/ffmpeg/include/libavutils -I${FFMPEG}/ffmpeg/include/libavformat" LDFLAGS="-L${FFMPEG}/ffmpeg/lib" --with-gnu-ld --with-x
# hack up the opencv <=> ffmpeg interface so we can use the latest ffmpeg
	touch configured

.PHONY: opencv
opencv: configured
	@echo "making it"
	export LD_RUN_PATH=${LD_RUN_PATH}:${FFMPEG}/ffmpeg/lib && export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${FFMPEG}/ffmpeg/lib && cd $(SOURCE_DIR) && make $(PARALLEL_JOBS) && make install

clean:
	-rm -rf opencv $(SOURCE_DIR) configured

wipe: clean
	rm -rf $(TARBALL)

.PHONY : clean download ffmpeg
