all: installed

TARBALL = build/loki-0.1.7.tar.bz2
TARBALL_URL = http://downloads.sourceforge.net/loki-lib/loki-0.1.7.tar.bz2
UNPACK_CMD  = tar xj --exclude=doc --exclude=test -f
SOURCE_DIR  = build/loki-0.1.7

include $(shell rospack find mk)/download_unpack_build.mk


installed: wiped $(SOURCE_DIR)/unpacked
	cp Makefile.common $(SOURCE_DIR)
	cd $(SOURCE_DIR)/src && make install
	touch installed

wiped: Makefile
	make wipe
	touch wiped

clean:
	-rm -rf $(SOURCE_DIR) lib
	rm -f *~ installed

.PHONY : clean wipe

wipe: clean
	rm -f $(TARBALL) 
	rm -rf lib build


