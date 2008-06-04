all: KNI_3.9.2/lib/linux/libKNIBase.a

KNI_3.9.2.tar.gz:
	wget http://www.neuronics.ch/cms_de/mediabase/KNI_3.9.2.tar.gz

download: KNI_3.9.2.tar.gz

KNI_3.9.2:
	tar xzf KNI_3.9.2.tar.gz

KNI_3.9.2/lib/linux/libKNIBase.a: KNI_3.9.2
	cd KNI_3.9.2 && make

clean:
	-rm -rf KNI_3.9.2

distclean: clean
	-rm -rf KNI_3.9.2.tar.gz

.PHONY : clean distclean download
