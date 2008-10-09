all: pil

TARBALL     = Imaging-1.1.6.tar.gz
TARBALL_URL = http://pr.willowgarage.com/downloads/Imaging-1.1.6.tar.gz
SOURCE_DIR  = Imaging-1.1.6
UNPACK_CMD  = tar xzf
include $(shell rospack find mk)/download_unpack.mk

pil: $(SOURCE_DIR)
	-rm -rf src
	mkdir -p src
	cd $(SOURCE_DIR) && \
			   python setup.py build && \
			   python setup.py install --prefix ../src 
	mv src/lib/python2.5/site-packages/* src/ 
	rm -rf src/bin src/lib
	touch pil

clean:
	rm -rf src pil $(SOURCE_DIR)

wipe: clean
	rm -rf $(TARBALL)
