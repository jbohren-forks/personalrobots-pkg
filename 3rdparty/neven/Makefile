INSTALL_DIR = neven

all: installed

GIT_DIR = nevensrc
GIT_URL = git://android.git.kernel.org/platform/external/neven.git

$(GIT_DIR):
	git clone $(GIT_URL) $(GIT_DIR)
	touch rospack_nosubdirs

installed: $(GIT_DIR)
	@mkdir -p build
	@mkdir -p include/neven
	cd build && cmake -DCMAKE_TOOLCHAIN_FILE=`rospack find rosbuild`/rostoolchain.cmake ..
	cd build && make $(ROS_PARALLEL_JOBS)
	cp nevensrc/FaceRecEm/common/src/b_FDSDK/fd_emb_sdk.h include/neven/

clean:
	cd build && make clean
	rm -rf build
	rm -rf include

wipe: clean
	rm -rf nevensrc

.PHONY : clean download
