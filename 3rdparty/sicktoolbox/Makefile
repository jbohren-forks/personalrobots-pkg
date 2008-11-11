all: sicktoolbox

download:
	wget -nc -nv http://pr.willowgarage.com/downloads/sicktoolbox-1.0.tar.gz

sicktoolbox-1.0: download
	#md5sum -c sicktoolbox-1.0.tar.gz.md5sum
	tar xzf sicktoolbox-1.0.tar.gz && patch -p0 <stdlib_include.patch

sicktoolbox: sicktoolbox-1.0
	cd sicktoolbox-1.0 && ./configure --prefix=$(PWD)/sicktoolbox
	cd sicktoolbox-1.0 && make && make install

clean:
	-rm -rf sicktoolbox-1.0 sicktoolbox
	rm -f *~

.PHONY : clean wipe

wipe: clean
	rm -f sicktoolbox-1.0.tar.gz
