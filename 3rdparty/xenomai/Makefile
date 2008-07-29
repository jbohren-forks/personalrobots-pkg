all: xenomai


XENOMAI_REVISION = 4063
LINUX_VERSION = 2.6.24.3

SVN_DIR = xenomai-svn
SVN_URL = http://svn.gna.org/svn/xenomai/trunk 
SVN_REVISION = -r $(XENOMAI_REVISION)
#SVN_PATCH = xenomai-r$(XENOMAI_REVISION).patch
include $(shell rospack find mk)/svn_checkout.mk

empty :=
space := $(empty) $(empty)
MAJOR_VERSION = v$(subst $(space),.,$(wordlist 1, 2, $(subst .,$(space),$(LINUX_VERSION))))
LINUX_ARCHIVE = linux-$(LINUX_VERSION).tar.bz2

download: SVN_UP $(LINUX_ARCHIVE)

xenomai: SVN_UP linux
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

clean:
	cd xenomai-svn/build && make clean
	cd linux && make clean
	
wipe:
	rm -rf xenomai-svn xenomai
	rm -rf $(LINUX_ARCHIVE) linux

