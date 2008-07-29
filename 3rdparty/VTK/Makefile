all: vtk

TARBALL = vtk-5.0.4.tar.gz
TARBALL_URL = http://www.vtk.org/files/release/5.0/vtk-5.0.4.tar.gz
SOURCE_DIR = VTK
UNPACK_CMD = tar zxf
include $(shell rospack find mk)/download_unpack.mk

vtk: $(SOURCE_DIR)
	mkdir -p vtk-build
	cd vtk-build && `rospack find cmake`/cmake/bin/cmake -D CMAKE_INSTALL_PREFIX=$(PWD)/vtk-install ../$(SOURCE_DIR) && make install

clean:
	rm -rf vtk-build vtk-install
wipe: clean
	rm -rf vtk-5.0.4 vtk-5.0.4.tar.gz
