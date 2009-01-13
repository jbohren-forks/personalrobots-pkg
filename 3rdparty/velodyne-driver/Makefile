all: installed

TARBALL = build/velodyne-driver-1.0.tar.gz
TARBALL_URL = http://pr.willowgarage.com/downloads/velodyne-driver-1.0.tar.gz
SOURCE_DIR = build/velodynedriver-1.0
UNPACK_CMD = tar xzf
include $(shell rospack find mk)/download_unpack_build.mk

installed: $(SOURCE_DIR) Makefile
	cd $(SOURCE_DIR) && ./configure --prefix=$(PWD)/velodyne-driver
	cd $(SOURCE_DIR) && make && make install
	touch installed

clean:
	-rm -rf velodynedriver-1.0 velodyne-driver
	rm -f *~ installed

.PHONY : clean wipe

wipe: clean
	rm -f velodyne-driver-1.0.tar.gz
