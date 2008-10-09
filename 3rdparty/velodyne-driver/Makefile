all: velodyne-driver

download: 
	wget -nc -nv http://pr.willowgarage.com/downloads/velodyne-driver-1.0.tar.gz

velodyne-driver-1.0: download
	tar -xzf velodyne-driver-1.0.tar.gz

velodyne-driver: velodyne-driver-1.0
	cd velodynedriver-1.0 && ./configure --prefix=$(PWD)/velodyne-driver
	cd velodynedriver-1.0 && make && make install

clean:
	-rm -rf velodynedriver-1.0 velodyne-driver
	rm -f *~

.PHONY : clean wipe

wipe: clean
	rm -f velodyne-driver-1.0.tar.gz
