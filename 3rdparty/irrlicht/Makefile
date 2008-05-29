all: irrlicht

irrlicht-1.4.zip:
	wget http://downloads.sourceforge.net/irrlicht/irrlicht-1.4.zip

download: irrlicht-1.4.zip

irrlicht-1.4: irrlicht-1.4.zip
	unzip irrlicht-1.4.zip

irrlicht: irrlicht-1.4
	cd irrlicht-1.4/source/Irrlicht && make

clean:
	-rm -rf irrlicht irrlicht-1.4

wipe: 
	rm -rf  irrlicht-1.4.zip

.PHONY : clean
