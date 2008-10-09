all: rtnet

RTNET_VERSION = 0.9.10
RTNET_ARCHIVE = rtnet-$(RTNET_VERSION).tar.bz2

download: $(RTNET_ARCHIVE)

rtnet: rtnet-src
	cd rtnet-src && \
	cp ../rtnet_config .rtnet_config && \
    make oldconfig && \
	make install

$(RTNET_ARCHIVE): 
	wget http://pr.willowgarage.com/downloads/$(RTNET_ARCHIVE)

rtnet-src: $(RTNET_ARCHIVE)
	tar xjf $(RTNET_ARCHIVE)
	mv $(subst .tar.bz2,,$(RTNET_ARCHIVE)) $@
	touch $@

clean:
	cd rtnet-src && make clean
	
wipe:
	rm -rf rtnet rtnet-src $(RTNET_ARCHIVE)

