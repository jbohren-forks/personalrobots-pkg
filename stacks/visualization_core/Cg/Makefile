all: Cg

CG_SOURCE = $(shell (if [ `uname -m` = "x86_64" ]; then echo http://pr.willowgarage.com/downloads/Cg-2.2_April2009_x86_64.tgz; else echo http://pr.willowgarage.com/downloads/Cg-2.2_April2009_x86.tgz; fi))
CG_TARBALL = $(shell (if [ `uname -m` = "x86_64" ]; then echo Cg-2.2_April2009_x86_64.tgz; else echo Cg-2.2_April2009_x86.tgz; fi))

MTYPE = $(shell (if [ `uname -m` = "x86_64" ]; then echo x86_64; else echo x86; fi))
BITS = $(shell (if [ `uname -m` = "x86_64" ]; then echo 64; fi))
ROOT = $(shell rospack find Cg)/Cg

$(CG_TARBALL):
	wget $(CG_SOURCE)

Cg: $(CG_TARBALL)
	@echo "making dir"
	@if [ ! -d Cg ]; then mkdir Cg; fi
	tar xf $(CG_TARBALL) -C$(ROOT);
	touch $(ROOT)/usr/lib$(BITS)/libCg.so
	@if [ ! -d $(ROOT)/lib ]; then mkdir $(ROOT)/lib; fi
	@if [ ! -d $(ROOT)/include ]; then mkdir $(ROOT)/include; fi
	cd $(ROOT)/lib; ln -sf ../usr/lib$(BITS)/* .
	cd $(ROOT)/include; ln -sf ../usr/include/* .

clean:
	rm -rf Cg
wipe: clean
	rm -rf $(CG_TARBALL)

download: $(CG_TARBALL)

