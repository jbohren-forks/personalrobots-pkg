# STAIR VISION APPLICATIONS MAKEFILE
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

LASIK_PATH := $(shell pwd)/..

-include $(LASIK_PATH)/make.mk

#######################################################################
# add application source files here
#######################################################################
APP_SRC = buildPatchDictionary.cpp buildPatchResponseCache.cpp \
	classifyImage.cpp trainObjectClassifier.cpp

#######################################################################

APP_PROG_NAMES = $(APP_SRC:.cpp=)
APP_OBJ = $(APP_SRC:.cpp=.o)

LIB_CFLAGS = -I$(LASIK_PATH)/lib 

CFLAGS += $(LIB_CFLAGS)

.PHONY: clean
.PRECIOUS: $(APP_OBJ)

all: depend ${addprefix ${BIN_PATH}/,$(APP_PROG_NAMES)}

$(BIN_PATH)/%: %.o $(LIBLASIK)
	${CCC} $*.o -o $(@:.o=) $(LIBLASIK) $(LFLAGS)

.cpp.o:
	${CCC} ${CFLAGS} -c $< -o $@

depend:
	g++ ${CFLAGS} -MM ${APP_SRC} >depend

clean:
	-rm $(APP_OBJ)
	-rm ${addprefix ${BIN_PATH}/,$(APP_PROG_NAMES)}
	-rm depend

-include depend

