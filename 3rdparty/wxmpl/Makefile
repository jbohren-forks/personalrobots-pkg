all: wxmpl

wxmpl-1.2.9.tar.gz: 
	wget wget http://agni.phys.iit.edu/~kmcivor/wxmpl/downloads/wxmpl-1.2.9.tar.gz

wxmpl-1.2.9: wxmpl-1.2.9.tar.gz
	@if [ ! -d wxmpl-1.2.9 ]; then tar xfz wxmpl-1.2.9.tar.gz; fi

wxmpl: wxmpl-1.2.9
	-mkdir src
	cd wxmpl-1.2.9 && python setup.py build 
	mv wxmpl-1.2.9/build/lib/* src/
	touch wxmpl
clean:
	rm -rf src wxmpl
wipe: clean
	rm -rf wxmpl-1.2.9 wxmpl-1.2.9.tar.gz

download: wxmpl-1.2.9.tar.gz
