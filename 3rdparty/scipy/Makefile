all: scipy

TARBALL = scipy-0.6.0.tar.gz
TARBALL_URL = http://pr.willowgarage.com/downloads/scipy-0.6.0.tar.gz
SOURCE_DIR = scipy-0.6.0
MD5SUM_FILE =  scipy-0.6.0.tar.gz.md5sum
UNPACK_CMD = tar xzf
NUMPY_PATH = `rospack find numpy`/src
include $(shell rospack find mk)/download_unpack.mk

scipy: $(SOURCE_DIR)
	mkdir -p src
	cd $(SOURCE_DIR) && PYTHONPATH=${PYTHONPATH}:${NUMPY_PATH} python setup.py build && PYTHONPATH=${PYTHONPATH}:${NUMPY_PATH}  python setup.py install --prefix ../src
	mv src/lib/python2.5/site-packages/* src/
	rm -rf src/bin src/lib
	touch scipy
clean:
	rm -rf src scipy $(SOURCE_DIR)
wipe: clean
	rm -rf $(TARBALL)
all: scipy

