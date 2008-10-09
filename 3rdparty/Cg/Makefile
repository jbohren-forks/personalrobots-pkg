all: Cg

CG_SOURCE_I686   = http://pr.willowgarage.com/downloads/Cg-2.0_May2008_x86.tgz
CG_SOURCE_X86_64 = http://pr.willowgarage.com/downloads/Cg-2.0_May2008_x86_64.tgz
CG_TAR_I686   = Cg-2.0_May2008_x86.tgz
CG_TAR_X86_64 = Cg-2.0_May2008_x86_64.tgz

MTYPE = $(shell (if [ `uname -m` = "x86_64" ]; then echo x86_64; else echo x86; fi))
BITS = $(shell (if [ `uname -m` = "x86_64" ]; then echo 64; fi))
ROOT = $(shell rospack find Cg)/Cg

Cg-checkout:
	if [ `uname -m` = x86_64 ]; then \
	  wget -nc $(CG_SOURCE_X86_64); \
	else \
	  wget -nc $(CG_SOURCE_I686); \
	fi

Cg: Cg-checkout
	@echo "making dir"
	@if [ ! -d Cg ]; then mkdir Cg; fi
	if [ `uname -m` = x86_64 ]; then \
	  tar xvf $(CG_TAR_X86_64) -C$(ROOT); \
	else \
	  tar xvf $(CG_TAR_I686) -C$(ROOT); \
	fi
	touch $(ROOT)/usr/lib$(BITS)/libCg.so
	@if [ ! -d $(ROOT)/lib ]; then mkdir $(ROOT)/lib; fi
	@if [ ! -d $(ROOT)/include ]; then mkdir $(ROOT)/include; fi
	cd $(ROOT)/lib; ln -sf ../usr/lib$(BITS)/* .
	cd $(ROOT)/include; ln -sf ../usr/include/* .

clean:
	rm -rf Cg
wipe:
	rm -rf Cg

download: Cg-checkout

