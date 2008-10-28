all: wxswig

TARBALL = SWIG-1.3.29-wx.tar.gz
TARBALL_URL = http://pr.willowgarage.com/downloads/$(TARBALL)
SOURCE_DIR = SWIG-1.3.29
UNPACK_CMD = tar xzf
include $(shell rospack find mk)/download_unpack.mk

$(VERSION)/Makefile: $(VERSION)/Makefile.in $(VERSION)/configure $(VERSION)/configure.in
	cd $(VERSION) && \
		./configure --prefix=$(WXSWIG_ROOT)

wxswig: $(SOURCE_DIR)
	cd $(SOURCE_DIR) && ./configure --prefix=$(shell rospack find wxswig) && make $(PARALLEL_JOBS) && make install
	touch wxswig
	
clean:
	rm -rf bin share $(SOURCE_DIR)
	
wipe: clean
	rm -rf $(TARBALL)
