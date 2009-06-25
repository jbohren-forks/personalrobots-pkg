# This make file just uses the underlying jam system to build everything. It is provided for
# uniformity so that the trex executive can be built using rosmake.
all: trex
include $(shell rospack find mk)/cmake.mk

trex:
	@if [ -z `which jam` ]; then echo "You don't appear to have jam installed.\nPlease install it (e.g., sudo apt-get install jam)" && false; fi
