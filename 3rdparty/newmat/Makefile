all: newmat-install

newmat-retrieve:
	@if [ ! -d newmat-source ]; then wget http://www.robertnz.net/ftp/newmat10.tar.gz && mkdir newmat-source && cd newmat-source && tar xzf ../newmat10.tar.gz; fi

newmat-build: newmat-retrieve
	cd newmat-source && make -f nm_gnu.mak

newmat-install: newmat-build
	mkdir -p newmat
	mkdir -p newmat/include
	mkdir -p newmat/lib
	cp newmat-source/*.h newmat/include
	cp newmat-source/*.a newmat/lib

clean:
	rm -rf newmat newmat-source newmat10.tar.gz
