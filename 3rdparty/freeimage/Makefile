all: FreeImage

FreeImage3100.zip: 
	wget http://internap.dl.sourceforge.net/sourceforge/freeimage/FreeImage3100.zip

FreeImage-3.10.0: FreeImage3100.zip
	unzip $^
	mv FreeImage $@

download: FreeImage3100.zip

ROOT = $(shell rospack find freeimage)/FreeImage

FreeImage: FreeImage-3.10.0
	@echo "making it"
	@echo "ROOT is: $(ROOT)"
	mkdir FreeImage
	mkdir FreeImage/lib
	mkdir FreeImage/include
	mkdir FreeImage/bin
	# Fix up the makefile, which doesn't make good use of any environment variables
	# We also don't need to install with -o root -g root
	# We also shouldn't run ldconfig, as we don't have permission for that either.
	cd FreeImage-3.10.0 ; mv Makefile.gnu Makefile.gnu-orig ; cat Makefile.gnu-orig | sed -e 's%/usr/lib%$(ROOT)/lib%g' | sed -e  's%/usr/include%$(ROOT)/include%g' | sed -e 's/-o root -g root//' | grep -v ldconfig > Makefile.gnu
	cd FreeImage-3.10.0 ; make ; make install

clean:
	(cd FreeImage-3.10.0; make clean;)

wipe:
	rm -rf FreeImage-3.10.0
	rm -rf FreeImage

.PHONY : clean download wipe
