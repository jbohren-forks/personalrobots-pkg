all: wxswig

VERSION = SWIG-1.3.29-wx
WXSWIG_ROOT = $(shell rospack find wxswig)

$(VERSION)/configure:
	cd $(VERSION) && \
	./autogen.sh

$(VERSION)/Makefile: $(VERSION)/Makefile.in $(VERSION)/configure $(VERSION)/configure.in
	cd $(VERSION) && \
		./configure --prefix=$(WXSWIG_ROOT)

$(VERSION): $(VERSION)/Makefile
	cd $(VERSION) && \
		make && \
		make install

wxswig: $(VERSION) 

clean:
	cd $(VERSION) && \
		make clean && \
		rm -f Makefile && \
		rm -f configure
	rm -rf bin
	rm -rf share

wipe: clean

.PHONY: clean