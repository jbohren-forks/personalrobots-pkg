all: python-twitter

SVN_DIR = python-twitter_svn
SVN_URL = http://python-twitter.googlecode.com/svn/trunk/
SVN_REVISION = -r 138
include $(shell rospack find mk)/svn_checkout.mk

SOURCE_DIR = python-twitter_svn

python-twitter: $(SOURCE_DIR)
	mkdir -p src/lib/python2.5/site-packages
	cd $(SOURCE_DIR) && python setup.py build && PYTHONPATH=${PYTHONPATH}:${PWD}/src/lib/python2.5/site-packages/ python setup.py install --prefix ../src
	mv src/lib/python2.5/site-packages/* src
	rm -rf src/lib
	touch python-twitter
clean:
	rm -rf src python-twitter $(SOURCE_DIR)
wipe: clean
	rm -rf build
all: python-twitter

