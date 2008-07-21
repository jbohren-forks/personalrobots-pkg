SYMLINK= npm
SYMTARGET= src

all: symlinks build/Makefile
	$(MAKE) -C build

symlinks:
	$(MAKE) SYMLINK=npm SYMTARGET=src symlink
	$(MAKE) SYMLINK=ros0.sh SYMTARGET=src/simul/ros0.sh symlink
	$(MAKE) SYMLINK=robots.ros0 SYMTARGET=src/simul/robots.ros0 symlink
	$(MAKE) SYMLINK=layout.ros0 SYMTARGET=src/simul/layout.ros0 symlink

symlink:
	test -L $(SYMLINK) || ln -s $(SYMTARGET) $(SYMLINK)

build/Makefile: CMakeLists.txt manifest.xml
	mkdir -p build
	cd build && `rospack find cmake`/cmake/bin/cmake -DCMAKE_TOOLCHAIN_FILE=`rospack find rosbuild`/rostoolchain.cmake ..

clean:
	rm -rf build
