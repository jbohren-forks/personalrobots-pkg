all: KNI_3.9.2/lib/linux/libKNIBase.a

KNI_3.9.2.tar.gz:
	wget http://pr.willowgarage.com/downloads/KNI_3.9.2.tar.gz

download: KNI_3.9.2.tar.gz

KNI_3.9.2: KNI_3.9.2.tar.gz
	tar xzf KNI_3.9.2.tar.gz
	cd KNI_3.9.2 && patch -p0 <../lm_boost_cxxflags.patch
	cd KNI_3.9.2 && patch -p0 <../cdlCOM.h_linux_anydevice.patch
	cd KNI_3.9.2 && patch -p0 <../cdlCOM.cpp_linux_anydevice.patch
	cd KNI_3.9.2 && patch -p0 <../kmlExt.h.patch
	cd KNI_3.9.2 && patch -p0 <../kmlExt.cpp.patch
	cd KNI_3.9.2 && patch -p0 <../lmBase.cpp.patch
	cd KNI_3.9.2 && patch -p0 <../lmBase.h.patch
	touch KNI_3.9.2

KNI_3.9.2/lib/linux/libKNIBase.a: KNI_3.9.2
	cd KNI_3.9.2 && make

clean:
#	-rm -rf KNI_3.9.2

distclean: clean
	-rm -rf KNI_3.9.2 KNI_3.9.2.tar.gz

.PHONY : clean distclean download
