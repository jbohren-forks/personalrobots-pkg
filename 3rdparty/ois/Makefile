all: ois

#REVISION = -r 6641

ois-checkout:
	@if [ ! -d ois-cvs ]; then cvs -z3 -d:pserver:anonymous@wgois.cvs.sourceforge.net:/cvsroot/wgois co -P ois ; mv ois ois-cvs; fi
	-cd ois-cvs && cvs up $(REVISION)

CONFIGURE_FLAGS = --with-arch=nocona --enable-release CXXFLAGS='-O3   -I$(ROOT)/include' LDFLAGS='-L$(ROOT)/lib' CFLAGS='-O3   -I$(ROOT)/include -I$(ROOT)/usr/include'

ROOT = $(shell rospack find ois)/ois

ois: ois-checkout
	# The bootstrap breaks on OS X.  Use autoreconf instead.
	#cd ois-cvs && ./bootstrap
	#
	# automake, as invoked by autoreconf, requires these files to be
	# present
	if test `uname` = Darwin ; then \
	  unset LIBRARY_PATH;\
	  cd ois-cvs/Mac/Xcode-2.2 && xcodebuild && cd ../../..; \
	  mkdir -p ois/lib; \
	  mkdir -p ois/include/OIS;\
	  cp ois-cvs/Mac/XCode-2.2/build/Release/OIS.framework/Versions/A/Headers/*.h ois/include/OIS; \
	  cp ois-cvs/Mac/XCode-2.2/build/Release/libOIS.a ois/lib; \
	else \
	  mkdir -p ois/lib; \
	  mkdir -p ois/include/OIS;\
	  cd ois-cvs && touch NEWS README AUTHORS ChangeLog; \
	  autoreconf -i -s; \
	  ./configure --prefix=$(ROOT) --with-trimesh=opcode $(CONFIGURE_FLAGS); \
	  make; \
	  make install; \
	  cd ..; \
	fi
	touch ois

clean:
	cd ois-cvs && make clean
	rm -rf ois
wipe:
	rm -rf ois-cvs ois

download: ois-checkout

