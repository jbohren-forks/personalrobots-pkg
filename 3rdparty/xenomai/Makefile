all: check_done

BUILD_TARGET = xenomai
include $(shell rospack find mk)/check_done.mk

XENOMAI_REVISION = 4007
LINUX_VERSION = 2.6.24.3

empty :=
space := $(empty) $(empty)
MAJOR_VERSION = v$(subst $(space),.,$(wordlist 1, 2, $(subst .,$(space),$(LINUX_VERSION))))
LINUX_ARCHIVE = linux-$(LINUX_VERSION).tar.bz2

download: xenomai-checkout $(LINUX_ARCHIVE)

xenomai-checkout: 
	@if [ ! -d xenomai-svn ]; then svn co -r $(XENOMAI_REVISION) http://svn.gna.org/svn/xenomai/trunk xenomai-svn ; \
	else \
		cd xenomai-svn && svn up -r $(XENOMAI_REVISION) ; \
	fi
	-cd xenomai-svn && patch -N -p0 < ../xenomai-r$(XENOMAI_REVISION).patch

xenomai: xenomai-checkout linux
	cd xenomai-svn && \
	./scripts/prepare-kernel.sh --linux=../linux --arch=i386 \
		--adeos=ksrc/arch/x86/patches/adeos-ipipe-2.6.24-x86-2.0-07.patch && \
	mkdir -p build && cd build && ../configure --prefix=$(PWD)/xenomai --enable-x86-sep && make SUDO=false install
	cd linux && \
	cp ../config-$(LINUX_VERSION)-AOpen .config && \
	make oldconfig && \
	make 


install: xenomai
	sudo make -C xenomai-svn/build devices
	sudo make -C linux install modules_install
	cd /boot && sudo update-initramfs -c -k 2.6.24.3 && sudo update-grub

$(LINUX_ARCHIVE): 
	wget http://www.us.kernel.org/pub/linux/kernel/$(MAJOR_VERSION)/$(LINUX_ARCHIVE)

linux: $(LINUX_ARCHIVE)
	tar xjf $(LINUX_ARCHIVE)
	mv $(subst .tar.bz2,,$(LINUX_ARCHIVE)) $@
	touch linux

clean: check_done_clean
	cd xenomai-svn/build && make clean
	cd linux && make clean
	
wipe: check_done_clean
	rm -rf xenomai-svn xenomai
	rm -rf $(LINUX_ARCHIVE) linux

