all: trex_demos
include $(shell rospack find mk)/cmake.mk

trex_demos: env.jam
	@if [ -z `which jam` ]; then echo "You don't appear to have jam installed.\nPlease install it (e.g., sudo apt-get install jam)" && false; fi

env.jam: installEnv.sh manifest.xml
	./installEnv.sh

