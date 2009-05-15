INSTALL_DIR = neven

all: installed

GIT_DIR = nevensrc
GIT_URL = git://android.git.kernel.org/platform/external/neven.git
GIT_PATCH = 64bit.patch

$(GIT_DIR):
	git clone $(GIT_URL) $(GIT_DIR)
	touch rospack_nosubdirs

patched: $(GIT_PATCH) Makefile
	cd $(GIT_DIR) && git reset --hard
ifneq ($(strip $(GIT_PATCH)),)
	$(foreach PATCH, $(GIT_PATCH), patch -d $(GIT_DIR) -p0 < $(PATCH) && ) echo patched
	touch rospack_nosubdirs
	touch patched
endif   

installed: $(GIT_DIR) patched
	@mkdir -p build
	@mkdir -p data
	cd build && cmake -DCMAKE_TOOLCHAIN_FILE=`rospack find rosbuild`/rostoolchain.cmake ..
	cd build && make $(ROS_PARALLEL_JOBS)
clean:
	-cd build && make clean
	-rm -rf build
	-rm -rf data
	-rm .build_version
	-rm .rosgcov_files

wipe: clean
	-rm -rf nevensrc

test: all
	if cd build && make -k $@; then make test-results; else make test-results && exit 1; fi
tests: all
	cd build && make $@
test-future: all
	cd build && make -k $@

.PHONY : clean download
