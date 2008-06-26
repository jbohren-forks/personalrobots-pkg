all: freeimage

FreeImage3100.zip: 
	wget http://internap.dl.sourceforge.net/sourceforge/freeimage/FreeImage3100.zip

freeimage-3.10.0: FreeImage3100.zip
	unzip $^
	mv FreeImage $@

download: FreeImage3100.zip

ROOT = $(shell rospack find freeimage)/freeimage

freeimage: freeimage-3.10.0
	@echo "making it"
	@echo "ROOT is: $(ROOT)"
	-mkdir -p freeimage
	-mkdir -p freeimage/lib
	-mkdir -p freeimage/include
	-mkdir -p freeimage/bin
	# Fix up the makefile, which doesn't make good use of any environment variables
	# We also don't need to install with -o root -g root
	# We also shouldn't run ldconfig, as we don't have permission for that either.
	cd freeimage-3.10.0 ; mv Makefile.gnu Makefile.gnu-orig ; cat Makefile.gnu-orig | sed -e 's%/usr/lib%$(ROOT)/lib%g' | sed -e  's%/usr/include%$(ROOT)/include%g' | sed -e 's/-o root -g root//' | grep -v ldconfig > Makefile.gnu
	cd freeimage-3.10.0 ; make ; make install

clean:
	-cd freeimage-3.10.0 && make clean

wipe:
	rm -rf freeimage-3.10.0
	rm -rf freeimage

.PHONY : clean download wipe
