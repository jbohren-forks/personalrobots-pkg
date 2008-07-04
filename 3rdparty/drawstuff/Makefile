OPENDE = $(shell rospack find opende)

all: drawstuff

drawstuff:
	-mkdir -p include/drawstuff && cp $(OPENDE)/opende-svn/include/drawstuff/*.h include/drawstuff/
	-mkdir -p lib && cp $(OPENDE)/opende-svn/drawstuff/src/.libs/* lib/ && chmod +x lib/libdrawstuff.la

clean:
	rm -rf include lib

wipe:	clean

download:
