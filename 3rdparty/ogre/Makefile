all: Ogre

ogre-v1-4-8.tar.bz2: 
	wget http://downloads.sourceforge.net/ogre/ogre-v1-4-8.tar.bz2

OGRE_VERSION = ogre-v1-4-8

download: ogre-v1-4-8.tar.bz2

$(OGRE_VERSION): $(OGRE_VERSION).tar.bz2
	tar xf $(OGRE_VERSION).tar.bz2
	mv ogrenew $(OGRE_VERSION)

ROOT = $(shell rospack find ogre)/Ogre

$(OGRE_VERSION)/Makefile: $(OGRE_VERSION)
	cd $(OGRE_VERSION); export LIB="-L$(ROOT)/lib -L$(ROOT)/usr/lib"; \
	./configure --prefix=$(ROOT) --enable-threading --with-pic --with-platform=GLX --with-gui=gtk  --disable-ogre-demos $(CONFIGURE_FLAGS)
	touch $(OGRE_VERSION)/Makefile
	mkdir Ogre
	mkdir Ogre/lib
	mkdir Ogre/include
	mkdir Ogre/share
	mkdir Ogre/bin

Ogre: $(OGRE_VERSION)/Makefile
	# building OGRE
	cd $(OGRE_VERSION); make $(JOBC)
	cd $(OGRE_VERSION); make install
	echo "done"

.PHONY: clean wipe

clean: 
	(cd $(OGRE_VERSION); make clean;)

wipe: 
	rm -rf $(OGRE_VERSION)
	rm -rf Ogre

