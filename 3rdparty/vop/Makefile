all: vop

TARBALL     = vop-1.0.tar.gz
TARBALL_URL = http://www.excamera.com/files/vop-1.0.tar.gz
SOURCE_DIR  = vop-1.0
UNPACK_CMD  = tar xzf
include $(shell rospack find mk)/download_unpack.mk

vop: $(SOURCE_DIR)
	-rm -rf src
	mkdir -p src
	cd $(SOURCE_DIR) && \
			   python setup.py build && \
			   python setup.py install --prefix ../src 
	mv src/lib/python2.5/site-packages/* src/ 
	rm -rf src/bin src/lib
	touch vop

clean:
	rm -rf src vop $(SOURCE_DIR)

wipe: clean
	rm -rf $(TARBALL)
