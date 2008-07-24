#!/bin/bash

# A tiny program to generate an include file for Jam

echo "# This is an auto-generated file built using rospack" > env.jam
echo "TREX_PKG_ROOT = `rospack find trex` ;" >> env.jam
echo "" >> env.jam
echo "PKG_CPP_FLAGS = `rospack export/cpp/cflags exec_trex` `pkg-config --cflags gdk-pixbuf-2.0` ;" >> env.jam
echo "" >> env.jam
echo "PKG_LINK_FLAGS = `rospack export/cpp/lflags exec_trex` `pkg-config --libs gdk-pixbuf-2.0` ;" >> env.jam
echo "" >> env.jam
