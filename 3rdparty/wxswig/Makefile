all: wxswig

VERSION = SWIG-1.3.29-wx
WXSWIG_ROOT = $(shell rospack find wxswig)

$(VERSION)/Makefile: $(VERSION)/Makefile.in $(VERSION)/configure
	cd $(VERSION) && \
		./autogen.sh && \
		./configure --prefix=$(WXSWIG_ROOT)

$(VERSION): $(VERSION)/Makefile
	cd $(VERSION) && \
		make && \
		make install

wxswig: $(VERSION) 

clean:
	cd $(VERSION) && \
		make clean && \
		rm -f Makefile
	rm -rf bin
	rm -rf share

wipe: clean

.PHONY: clean