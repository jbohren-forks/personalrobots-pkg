SRCDIR= $(PWD)/sunflower
BUILDDIR= $(PWD)/build
INSTDIR= $(PWD)/sfl
ESTARDIR= `rospack find estar`/local

all: $(SRCDIR) $(SRCDIR)/configure $(BUILDDIR)/config.h
	$(MAKE) -C $(BUILDDIR) install

$(SRCDIR):
	@ echo "SRCDIR should be handled by svn:externals"
	svn co https://libsunflower.svn.sourceforge.net/svnroot/libsunflower/trunk/sunflower

$(SRCDIR)/configure: $(SRCDIR)/configure.ac $(SRCDIR)/bootstrap-buildsystem.sh
	cd $(SRCDIR) && ./bootstrap-buildsystem.sh

$(BUILDDIR)/config.h: $(SRCDIR)/configure $(SRCDIR)/config.h.in
	test -d $(BUILDDIR) || mkdir -p $(BUILDDIR)
	cd $(BUILDDIR) && $(SRCDIR)/configure --prefix=$(INSTDIR) --with-estar=$(ESTARDIR)

clean:
	$(MAKE) -C $(BUILDDIR) clean

distclean:
	$(MAKE) -C $(BUILDDIR) distclean

wipe:
	svn revert -R $(SRCDIR)
	svn st --no-ignore $(SRCDIR) | awk '{print $2}' | xargs rm -rvf
	rm -rf $(BUILDDIR) $(INSTDIR)
