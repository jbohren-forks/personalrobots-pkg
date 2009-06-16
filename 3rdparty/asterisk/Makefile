all: installed

VERSION		= 1.6.1.0
TARBALL     = build/asterisk-$(VERSION).tar.gz
TARBALL_URL = http://downloads.asterisk.org/pub/telephony/asterisk/releases/asterisk-$(VERSION).tar.gz
UNPACK_CMD  = tar -xvzf
SOURCE_DIR  = build/asterisk-$(VERSION)

include $(shell rospack find mk)/download_unpack_build.mk

installed: wiped $(SOURCE_DIR)/unpacked
	echo "Building Asterisk PBX..."
	cd $(SOURCE_DIR) && ./configure --prefix=`pwd`/../asterisk &&  make && make install && make samples && rm main/asterisk  # detele asterisk from source folder so that rosrun will not find it 
	cp files/extensions.ael build/asterisk/etc/asterisk
	-cp files/sip.conf build/asterisk/etc/asterisk
	cp files/*.py build/asterisk/var/lib/asterisk/agi-bin
	touch installed

wiped: Makefile
	make wipe
	touch wiped

wipe:
	rm -rf build installed
