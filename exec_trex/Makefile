# This make file just uses the underlying jam system to build everything. It is provided for
# uniformity so that the trex executive can be built using rosmake.
all: trex

trex: env.jam
	@if [ -z `which jam` ]; then echo "You don't appear to have jam installed.\nPlease install it (e.g., sudo apt-get install jam)"; else ./make_trex; fi

env.jam: installEnv 
	./installEnv

installEnv: Install.cc manifest.xml
	g++ -o installEnv Install.cc