
all: installed


installed:
	wget http://cvlab.epfl.ch/~tola/research/08/daisy/daisy-1.7.0.tar.gz
	tar xvzf daisy-1.7.0.tar.gz
	mv daisy-1.7.0/* .
	rm -rf daisy-1.7.0
	patch makefile < makefile.patch
	mv makefile daisy_makefile
	make -f daisy_makefile slib
	touch installed

clean: 
	ls | grep -v Makefile | grep -v patch | grep -v manifest.xml | xargs rm -r