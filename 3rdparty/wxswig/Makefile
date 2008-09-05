all: wxswig

VERSION = SWIG-1.3.29-wx
WXSWIG_ROOT = $(shell rospack find wxswig)

$(VERSION)/Makefile:
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
		rm -f Makefile
	rm -rf bin

wipe: clean

.PHONY: clean