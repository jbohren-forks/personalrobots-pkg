all: newmat-install

newmat-retrieve:
	@if [ ! -f newmat10.tar.gz ]; then wget http://www.robertnz.net/ftp/newmat10.tar.gz; fi
	@if [ ! -d newmat-source ]; then mkdir newmat-source && cd newmat-source && tar xzf ../newmat10.tar.gz && patch -p0 < ../newmat.patch; fi

newmat-build: newmat-retrieve
	cd newmat-source && make -f nm_gnu.mak

newmat-install: newmat-build
	mkdir -p newmat
	mkdir -p newmat/include/newmat
	mkdir -p newmat/lib
	cp newmat-source/*.h newmat/include/newmat
	cp newmat-source/*.a newmat/lib

clean:
	rm -rf newmat newmat-source
