all:	freesolid

FreeSOLID-2.1.1.zip:
	wget -nc http://downloads.sourceforge.net/freesolid/FreeSOLID-2.1.1.zip

download: FreeSOLID-2.1.1.zip

FreeSOLID-2.1.1:	FreeSOLID-2.1.1.zip
	unzip -u $^

freesolid:	FreeSOLID-2.1.1
	cd FreeSOLID-2.1.1 && sh ./configure --prefix=$(PWD)/freesolid && make && make install

clean:
	rm -rf freesolid
	-cd FreeSOLID-2.1.1 && make clean

wipe:
	rm -rf freesolid FreeSOLID-2.1.1 FreeSOLID-2.1.1.zip

.PHONY:	download clean wipe
