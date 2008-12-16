all: wxmpl

TARBALL = build/wxmpl-1.2.9.tar.gz
TARBALL_URL = http://pr.willowgarage.com/downloads/wxmpl-1.2.9.tar.gz
SOURCE_DIR = build/wxmpl-1.2.9
UNPACK_CMD = tar xzf
include $(shell rospack find mk)/download_unpack_build.mk

wxmpl: $(SOURCE_DIR)
	-mkdir src
	cd $(SOURCE_DIR) && python setup.py build 
	mv $(SOURCE_DIR)/build/lib/* src/
	touch wxmpl
clean:
	rm -rf src wxmpl
wipe: clean
	rm -rf $(SOURCE_DIR) $(TARBALL) build
