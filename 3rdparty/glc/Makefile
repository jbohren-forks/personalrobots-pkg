all:
	cd packetstream && make
	cd elfhacks && make
	cd glc && make
	cp glc/glc-0.5.7/scripts/encode.sh bin/glc-encode
	chmod +x bin/glc-encode

clean:
	cd packetstream && make clean
	cd elfhacks && make clean
	cd glc && make clean

wipe:
	cd packetstream && make wipe
	cd elfhacks && make wipe
	cd glc && make wipe
	rm -rf bin lib include

