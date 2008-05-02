all: sicktoolbox

sicktoolbox-1.0:
	tar xzf sicktoolbox-1.0.tar.gz

sicktoolbox: sicktoolbox-1.0
	cd sicktoolbox-1.0 && ./configure --prefix=$(PWD)/sicktoolbox
	cd sicktoolbox-1.0 && make && make install

clean:
	-rm -rf sicktoolbox-1.0 sicktoolbox
	rm -f *~

.PHONY : clean
