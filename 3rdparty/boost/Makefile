all: boost_1_35_0

boost_1_35_0.tar.gz:
	wget http://voxel.dl.sourceforge.net/sourceforge/boost/boost_1_35_0.tar.gz

download: boost_1_35_0.tar.gz

boost_1_35_0: boost_1_35_0.tar.gz
	tar xzf boost_1_35_0.tar.gz

clean:
	-rm -rf boost_1_35_0

distclean: clean
	-rm -rf boost_1_35_0.tar.gz

.PHONY : clean distclean download
