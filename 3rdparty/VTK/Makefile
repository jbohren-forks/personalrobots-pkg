all: vtk

vtk-5.0.4.tar.gz:
	wget http://www.vtk.org/files/release/5.0/vtk-5.0.4.tar.gz

vtk-5.0.4: vtk-5.0.4.tar.gz
	#md5sum -c vtk-5.0.4.tar.gz.md5sum
	tar xfz vtk-5.0.4.tar.gz
	mv VTK vtk-5.0.4
	touch vtk-5.0.4

vtk: vtk-5.0.4
	mkdir -p vtk-build
	cd vtk-build && `rospack find cmake`/cmake/bin/cmake -D CMAKE_INSTALL_PREFIX=$(PWD)/vtk-install ../vtk-5.0.4 && make install

clean:
	rm -rf vtk-build vtk-install
wipe: clean
	rm -rf vtk-5.0.4 vtk-5.0.4.tar.gz

download: vtk-5.0.4.tar.gz
