all: installed

TARBALL = build/ply.tar.gz
TARBALL_URL = http://www.cc.gatech.edu/projects/large_models/files/ply.tar.gz
SOURCE_DIR = build/ply
UNPACK_CMD = tar xvfz
TARBALL_PATCH=

include $(shell rospack find mk)/download_unpack_build.mk

ROOT = $(shell rospack find ply)/ply

installed: wiped $(SOURCE_DIR)/unpacked
	@echo "making it"
	@echo "ROOT is: $(ROOT)"
	-mkdir -p $(ROOT)
	-mkdir -p $(ROOT)/bin
	cd $(SOURCE_DIR) ; make $(ROS_PARALLEL_JOBS);
	cp $(SOURCE_DIR)/ply2iv $(ROOT)/bin/ply2iv
	touch installed

wiped: Makefile
	make wipe
	touch wiped

clean:
	-cd $(SOURCE_DIR) && make clean
	rm -rf $(ROOT) installed

wipe: clean
	rm -rf build

.PHONY : clean download wipe
