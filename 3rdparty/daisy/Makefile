
all: daisy
ifneq ($(wildcard daisy_makefile),) #Clean up cruft from previous makefile.
	ls | grep -v Makefile | grep -v Makefile.old | grep -v makefile.patch | grep -v manifest.xml | grep -v build | xargs rm -r; fi
endif

TARBALL = build/daisy-1.7.0.tar.gz
TARBALL_URL = http://cvlab.epfl.ch/~tola/research/08/daisy/daisy-1.7.0.tar.gz
SOURCE_DIR = build/daisy-1.7.0
TARBALL_PATCH = makefile.patch
include $(shell rospack find mk)/download_unpack_build.mk

daisy: $(SOURCE_DIR)/unpacked
ifeq ($(wildcard $(SOURCE_DIR)/lib/libdaisy.so),) 
	cd $(SOURCE_DIR) && make -f makefile slib
endif
	cd $(SOURCE_DIR) && make -f makefile dox

clean: 
	-rm -rf $(SOURCE_DIR)

wipe: clean
	-rm -rf build
