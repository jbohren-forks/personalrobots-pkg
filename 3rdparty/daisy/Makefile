

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
# Change to BSD.  This is with Engin Tola's permission, and he is changing the license on his website soon.  
	chmod a+x block_replace.py
	./block_replace.py licenses/gpl-makefile licenses/bsd-makefile build/daisy-1.7.0
	./block_replace.py licenses/gpl licenses/bsd build/daisy-1.7.0
	./block_replace.py licenses/gpl2 licenses/bsd build/daisy-1.7.0
	./block_replace.py licenses/gpl3 licenses/bsd build/daisy-1.7.0
	./block_replace.py licenses/gpl4 licenses/bsd build/daisy-1.7.0
	./block_replace.py licenses/gpl5 licenses/bsd build/daisy-1.7.0
	./block_replace.py licenses/gpl6 licenses/bsd build/daisy-1.7.0
	./block_replace.py licenses/gpl7 licenses/bsd build/daisy-1.7.0
	./block_replace.py licenses/gpl8 licenses/bsd build/daisy-1.7.0
	./block_replace.py licenses/gpl9 licenses/bsd build/daisy-1.7.0
	./block_replace.py licenses/gpl10 licenses/bsd build/daisy-1.7.0
	./block_replace.py licenses/gpl11 licenses/bsd build/daisy-1.7.0

clean: 
	-rm -rf $(SOURCE_DIR)

wipe: clean
	-rm -rf build
