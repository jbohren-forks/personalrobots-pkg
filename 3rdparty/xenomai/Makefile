all: build

VERSION = xenomai_svn-r4220
TARBALL = build/$(VERSION).tar.bz2
TARBALL_URL = http://pr.willowgarage.com/downloads/$(VERSION).tar.bz2
SOURCE_DIR = build/xenomai-svn
UNPACK_CMD = tar xf
SVN_DIR = build/xenomai-svn


XENOMAI_REVISION = 4220
#SVN_DIR = xenomai-svn
SVN_URL = http://svn.gna.org/svn/xenomai/tags/v2.4.5 
SVN_REVISION = -r $(XENOMAI_REVISION)
#SVN_PATCH = xenomai-r$(XENOMAI_REVISION).patch
#include $(shell rospack find mk)/svn_checkout.mk

include $(shell rospack find mk)/download_unpack_build.mk


#build: SVN_UP xenomai
build: $(SOURCE_DIR) xenomai

configured: $(SVN_DIR) build/linux Makefile
	cd $(SVN_DIR) && mkdir -p build && cd build && ../configure --prefix=$(PWD)/xenomai --enable-x86-sep --enable-x86-tsc --enable-smp --disable-dox-doc
	cd $(SVN_DIR) && \
	./scripts/prepare-kernel.sh --linux=../linux --arch=i386 \
		--adeos=ksrc/arch/x86/patches/adeos-ipipe-2.6.24-x86-2.0-07.patch 
	cd build/linux && \
	cp ../../config-$(LINUX_VERSION)-AOpen .config && \
	make oldconfig && make prepare scripts
	touch configured $(SVN_DIR)

.PHONY: xenomai
xenomai: configured
	cd $(SVN_DIR)/build && make $(PARALLEL_JOBS) SUDO=false install

clean:
	-cd $(SVN_DIR)/build && make clean
	rm -rf xenomai configured

wipe: 
	rm -rf $(SVN_DIR) xenomai $(TARBALL)


LINUX_VERSION = 2.6.24.3
empty :=
space := $(empty) $(empty)
MAJOR_VERSION = v$(subst $(space),.,$(wordlist 1, 2, $(subst .,$(space),$(LINUX_VERSION))))
LINUX_ARCHIVE = linux-$(LINUX_VERSION).tar.bz2

download_kernel: $(LINUX_ARCHIVE)

kernel: configured
	cd build; make  $(PARALLEL_JOBS) -C linux

install_kernel: xenomai
	sudo make  $(PARALLEL_JOBS) -C $(SVN_DIR)/build devices
	cd build; sudo make  $(PARALLEL_JOBS) -C linux install modules_install
	cd /boot && sudo update-initramfs -c -k 2.6.24.3 && sudo update-grub

build/$(LINUX_ARCHIVE): 
	cd build; wget http://pr.willowgarage.com/downloads/$(LINUX_ARCHIVE)

build/linux: build/$(LINUX_ARCHIVE)
	cd build; tar xjf $(LINUX_ARCHIVE)
	mv $(subst .tar.bz2,,build/$(LINUX_ARCHIVE)) $@
	touch linux

clean_kernel:
	cd build/linux && make clean

wipe_kernel:
	rm -rf build/$(LINUX_ARCHIVE) build/linux
