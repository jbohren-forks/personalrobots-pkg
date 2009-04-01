default: installed all

ARCH = $(shell uname -m)
TARBALL = build/Prosilica\ GigE\ SDK\ Linux\ 1.18.tgz
TARBALL_URL = http://pr.willowgarage.com/downloads/Prosilica%20GigE%20SDK%20Linux%201.18.tgz
SOURCE_DIR = build/Prosilica_GigE_SDK
INITIAL_DIR = build/Prosilica\ GigE\ SDK
# Spaces in tarball file name screw this up, awesome
#MD5SUM_FILE = $(TARBALL).md5sum
include $(shell rospack find mk)/download_unpack_build.mk
include $(shell rospack find mk)/cmake.mk

installed: wiped $(SOURCE_DIR)/unpacked
	-mkdir include
	cp $(SOURCE_DIR)/inc-pc/PvApi.h $(SOURCE_DIR)/inc-pc/PvRegIo.h include
	-mkdir lib
ifeq ($(ARCH), i686)
	cp $(SOURCE_DIR)/bin-pc/x86/libPvAPI.so lib
else ifeq ($(ARCH), i386)
	cp $(SOURCE_DIR)/bin-pc/x86/libPvAPI.so lib
else ifeq ($(ARCH), x86_64)
	cp $(SOURCE_DIR)/bin-pc/x64/libPvAPI.so lib
else ifeq ($(ARCH), ppc)
	cp $(SOURCE_DIR)/bin-pc/ppc/libPvAPI.so lib
else
	@echo "Unsupported architecture $(ARCH)"
	@exit 2
endif
	touch installed

wiped: Makefile
	make wipe
	touch wiped

wipe: clean
	rm -rf include lib bin
	rm -f *~ installed
