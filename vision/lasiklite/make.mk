# MAKE INCLUDE FILE FOR STAIR VISION PROJECT
# Copyright (c) 2007-2008, Stephen Gould
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Stanford University nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
# EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <copyright holder> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

ROOT_PATH = $(LASIK_PATH)/..
BIN_PATH = $(LASIK_PATH)/bin
LIB_PATH = $(LASIK_PATH)/lib

OPENCV_CFLAGS=$(shell rospack --lang=cpp --attrib=cflags export opencv_latest)
OPENCV_LFLAGS=$(shell rospack --lang=cpp --attrib=lflags export opencv_latest)

LIBLASIK = $(BIN_PATH)/liblasiklite.a

CFLAGS = -g -O3 -m32 $(OPENCV_CFLAGS) -Wall
LFLAGS = -g -lm -m32 -lpthread $(OPENCV_LFLAGS) $(LIBLASIK)

CCC = g++
OBJ = $(SRC:.cpp=.o)

