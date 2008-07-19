SRCDIR= $(PWD)/estar
BUILDDIR= $(PWD)/build
INSTDIR= $(PWD)/local

#CMAKE= `rospack find cmake`/cmake/bin/cmake 
#CMAKE_ARGS= -Wno-dev -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=$(INSTDIR)

all: $(SRCDIR) $(SRCDIR)/configure $(BUILDDIR)/config.h
	$(MAKE) -C $(BUILDDIR) install

$(SRCDIR):
	@ echo "SRCDIR should be handled by svn:externals"
	svn co https://estar.svn.sourceforge.net/svnroot/estar/trunk/estar

$(SRCDIR)/configure: $(SRCDIR)/configure.ac $(SRCDIR)/bootstrap-buildsystem.sh
	cd $(SRCDIR) && ./bootstrap-buildsystem.sh

$(BUILDDIR)/config.h: $(SRCDIR)/configure $(SRCDIR)/config.h.in
	test -d $(BUILDDIR) || mkdir -p $(BUILDDIR)
	cd $(BUILDDIR) && $(SRCDIR)/configure --prefix=$(INSTDIR)

clean:
	$(MAKE) -C $(BUILDDIR) clean

distclean:
	$(MAKE) -C $(BUILDDIR) distclean

wipe:
	rm -rf $(SRCDIR) $(BUILDDIR) $(INSTDIR)
