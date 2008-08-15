#!/bin/bash

# A tiny program to generate an include file for Jam

echo "# This is an auto-generated file built using rospack" > env.jam
echo "TREX_PKG_ROOT = `rospack find trex` ;" >> env.jam
echo "" >> env.jam
