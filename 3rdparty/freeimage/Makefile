all: freeimage

FreeImage3100.zip: 
	wget http://pr.willowgarage.com/downloads/FreeImage3100.zip

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
	# Here's some other nasty hackery to build on OS X.  This will only
	# work on modern Intel-based Macs, and it will not build shared
	# libraries.
	#
	# TODO: Replace this sed madness with a proper patch
	cd freeimage-3.10.0 ; mv Makefile.osx Makefile.osx-orig ; cat Makefile.osx-orig | sed -e 's%/usr/lib%$(ROOT)/lib%g' | sed -e  's%/usr/include%$(ROOT)/include%g' | sed -e 's/-o root -g wheel//' | sed -e 's%/usr/local%$(ROOT)%g'| sed -e 's%FreeImage: $$(STATICLIB) $$(SHAREDLIB)%FreeImage: $$(STATICLIB)%' | sed -e 's%$$(STATICLIB): $$(STATICLIB)-ppc%$$(STATICLIB):%' | sed -e 's%$$(LIPO) -create $$(STATICLIB)-ppc%$$(LIPO) -create%' | sed -e 's%install -m 644  $$(SHAREDLIB)%install -m 644 %' | sed -e 's%ln -sf%#ln -sf%' | sed -e 's%all: dist%all: FreeImage%' | grep -v ldconfig > Makefile.osx
	cd freeimage-3.10.0 ; make ; make install
	mkdir -p freeimage/lib/pkgconfig
	sed 's%FREEIMAGE_PATH%$(shell rospack find freeimage)%g' freeimage.pc > freeimage/lib/pkgconfig/freeimage.pc

clean:
	-cd freeimage-3.10.0 && make clean
	rm -rf freeimage

wipe:
	rm -rf freeimage-3.10.0
	rm -rf freeimage

.PHONY : clean download wipe
