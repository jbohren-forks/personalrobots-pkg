# change the next flag to 'no' if drawstuff is not to be built
WITH_DRAWSTUFF = yes

CFG_OPTIONS = --with-arch=nocona --enable-release --disable-asserts --with-pic --enable-double-precision
SVN_REVISION = -r 1550

SVN_PATCH = opende_patch.diff

ifeq ($(WITH_DRAWSTUFF), yes)
CFG_OPTIONS += --with-drawstuff=X11
endif

all: build

SVN_DIR = opende-svn
SVN_URL = https://opende.svn.sourceforge.net/svnroot/opende/trunk 
include $(shell rospack find mk)/svn_checkout.mk

CONFIGURE_FLAGS = $(CFG_OPTIONS) CXXFLAGS='-ggdb -fPIC -I$(ROOT)/include' LDFLAGS='-L$(ROOT)/lib' CFLAGS='-ggdb -fPIC -I$(ROOT)/include -I$(ROOT)/usr/include'
#CONFIGURE_FLAGS = $(CFG_OPTIONS) CC='/opt/intel/cc/10.1.008/bin/icc' CXX='/opt/intel/cc/10.1.008/bin/icpc' LD='/opt/intel/cc/10.1.008/bin/xild' CXXFLAGS='-march=core2 -xT -parallel -O3 -fPIC -I$(ROOT)/include' LDFLAGS='-L$(ROOT)/lib' CFLAGS='-march=core2 -xT -parallel -O3 -fPIC -I$(ROOT)/include -I$(ROOT)/usr/include' LDFLAGS='-Wl,-rpath,/opt/intel/cc/10.1.008/lib -L/opt/intel/cc/10.1.008/lib -lguide -lcxaguard -limf -lsvml -lirc -pthread'
#CONFIGURE_FLAGS = $(CFG_OPTIONS) CC='/opt/intel/cc/10.1.008/bin/icc' CXX='/opt/intel/cc/10.1.008/bin/icpc' LD='/opt/intel/cc/10.1.008/bin/xild' CXXFLAGS='-march=core2 -xT -msse3 -openmp -parallel -O3 -fPIC -I$(ROOT)/include' LDFLAGS='-L$(ROOT)/lib' CFLAGS='-march=core2 -xT -msse3 -openmp -parallel -O3 -fPIC -I$(ROOT)/include -I$(ROOT)/usr/include' LDFLAGS='-Wl,-rpath,/opt/intel/cc/10.1.008/lib -L/opt/intel/cc/10.1.008/lib -lomp_db -lguide -lcxaguard -limf -lsvml -lompstub -lirc -liomp5 -lintlc -pthread'

ROOT = $(shell rospack find opende)/opende

build: SVN_UP_REVERT_PATCH opende

configured: $(SVN_DIR) Makefile
	cd $(SVN_DIR) && sh ./autogen.sh
	cd $(SVN_DIR) && ./configure --prefix=$(ROOT) --with-trimesh=opcode --enable-new-trimesh --enable-shared $(CONFIGURE_FLAGS)
	touch configured

.PHONY: opende
opende: configured
	cd $(SVN_DIR) && make $(PR2_JOBS)
	cd $(SVN_DIR) && make install
	@echo "patch ode-config to pass -Wl,-rpath,-L{exec_prefix}/lib"
	sed 's/echo  -L$${exec_prefix}\/lib -lode/echo  -Wl,-rpath,$${exec_prefix}\/lib -L$${exec_prefix}\/lib -lode/g' opende-svn/ode-config > opende/bin/ode-config
	#sed 's/echo  -L$${exec_prefix}\/lib -lode/echo  -Wl,-rpath,$${exec_prefix}\/lib -L$${exec_prefix}\/lib -lode -Wl,-rpath,\/opt\/intel\/cc\/10.1.008\/lib -L\/opt\/intel\/cc\/10.1.008\/lib -lguide -lcxaguard -limf -lsvml -lirc -pthread/g' opende-svn/ode-config > opende/bin/ode-config
	@echo "patch ode.pc to pass -Wl,-rpath,-L{libdir}"
	sed 's/Libs: -L$${libdir} -lode/Libs: -Wl,-rpath,$${libdir} -L$${libdir} -lode/g' opende-svn/ode.pc > opende/lib/pkgconfig/ode.pc
	#sed 's/Libs: -L$${libdir} -lode/Libs: -Wl,-rpath,$${libdir} -L$${libdir} -lode -Wl,-rpath,\/opt\/intel\/cc\/10.1.008\/lib -L\/opt\/intel\/cc\/10.1.008\/lib -lguide -lcxaguard -limf -lsvml -lirc -pthread/g' opende-svn/ode.pc > opende/lib/pkgconfig/ode.pc

clean:
	-cd $(SVN_DIR) && make clean
	rm -rf opende configured
wipe: clean
	rm -rf $(SVN_DIR)
