all: build



TARBALL = build/octave-3.0.3.tar.gz
TARBALL_URL = ftp://ftp.octave.org/pub/octave/octave-3.0.3.tar.gz
INITIAL_DIR = build/octave-3.0.3
SOURCE_DIR = build/octave
UNPACK_CMD = tar xvfz

include $(shell rospack find mk)/download_unpack_build.mk

build: octave

ROOT = $(shell rospack find octave)/octave

.PHONY: octave
octave: $(SOURCE_DIR)
	@echo "making it"
	@echo "ROOT is: $(ROOT)"
	-mkdir -p $(ROOT)
	-mkdir -p $(ROOT)/bin
	cd $(SOURCE_DIR) && ./configure --prefix=$(ROOT)
	cd $(SOURCE_DIR) && make && make install

clean:
	-cd $(SOURCE_DIR) && make clean
	rm -rf $(ROOT)

wipe: clean
	rm -rf $(SOURCE_DIR) build
