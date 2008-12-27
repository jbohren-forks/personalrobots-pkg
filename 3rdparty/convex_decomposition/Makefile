all: convex_decomposition

TARBALL = build/ConvexDecomposition.zip
TARBALL_URL = http://www.amillionpixels.us/ConvexDecomposition.zip
SOURCE_DIR = build/convex_decomposition
INITIAL_DIR = build/ConvexDecomposition
UNPACK_CMD = unzip

include $(shell rospack find mk)/download_unpack_build.mk

ROOT = $(shell rospack find convex_decomposition)/convex_decomposition

convex_decomposition: $(SOURCE_DIR)
	@echo "making it"
	@echo "ROOT is: $(ROOT)"
	-mkdir -p $(ROOT)
	-mkdir -p $(ROOT)/bin
	cd $(SOURCE_DIR) ; patch -p0 -N < ../../convex_decomposition.patch || echo
	cd $(SOURCE_DIR) ; make $(PARALLEL_JOBS); make install

clean:
	-cd $(SOURCE_DIR) && make clean
	rm -rf $(ROOT)

wipe:
	rm -rf build
	rm -rf $(ROOT)

.PHONY : clean download wipe
