# * 
# ****************************************************************************
# * <P> XML.c - implementation file for basic XML parser written in ANSI C++ 
# * for portability.
# * It works by using recursion and a node tree for breaking down the elements
# * of an XML document.  </P>
# *
# * @version     V2.24
# *
# * @author      Frank Vanden Berghen
# * based on original implementation by Martyn C Brown
# *
# * BSD license:
# * Copyright (c) 2002, Frank Vanden Berghen
# * All rights reserved.
# * Redistribution and use in source and binary forms, with or without
# * modification, are permitted provided that the following conditions are met:
# *
# *     * Redistributions of source code must retain the above copyright
# *       notice, this list of conditions and the following disclaimer.
# *     * Redistributions in binary form must reproduce the above copyright
# *       notice, this list of conditions and the following disclaimer in the
# *       documentation and/or other materials provided with the distribution.
# *     * Neither the name of the Frank Vanden Berghen nor the
# *       names of its contributors may be used to endorse or promote products
# *       derived from this software without specific prior written permission.
# *
# * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
# * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# * DISCLAIMED. IN NO EVENT SHALL THE REGENTS AND CONTRIBUTORS BE LIABLE FOR ANY
# * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# * 
# ****************************************************************************
#
#makefile for the xmlParser library
#
all : xmlParser.a # xmlEasyTest xmlTest xmlTestUnicode

# to force 32-bit compile (set to 1 in make.local)
FORCE32BIT = 0
ifeq (,$(findstring x86_64,$(MACHTYPE)))
  FORCE32BIT = 1
endif

# include local user make.local to override any settings
ifeq ($(wildcard ../../make.local), ../../make.local)
  -include ../../make.local
endif

ifeq ($(FORCE32BIT), 0)
  GPP=g++ -fPIC
else
  GPP=g++ -m32
endif

xmlEasyTest : xmlParser.cpp xmlParser.h xmlTest.cpp
	$(GPP) -g -Wall -DXML_NO_WIDE_CHAR -o xmlEasyTest \
                 xmlParser.cpp xmlTest.cpp
xmlTest : xmlParser.cpp xmlParser.h xmlTest.cpp
	$(GPP) -g -Wall -o xmlTest \
                 xmlParser.cpp xmlTest.cpp
xmlTestUnicode : xmlParser.cpp xmlParser.h xmlTestUnicode.cpp
	$(GPP) -g -Wall -D_UNICODE -DUNICODE -o xmlTestUnicode \
                 xmlParser.cpp xmlTestUnicode.cpp

xmlParser.a: xmlParser.o
	ar rv $@ $?
	ranlib $@

xmlParser.o: xmlParser.cpp xmlParser.h
	$(GPP) -c -o xmlParser.o xmlParser.cpp

clean:
	-@rm xmlEasyTest          >/dev/null 2>/dev/null
	-@rm xmlTest              >/dev/null 2>/dev/null
	-@rm xmlTestUnicode       >/dev/null 2>/dev/null
	-@rm test.xml             >/dev/null 2>/dev/null
	-@rm testUnicode.xml      >/dev/null 2>/dev/null
	-@rm outputTestUTF8.xml   >/dev/null 2>/dev/null
	-@rm *~                   >/dev/null 2>/dev/null
	-@rm *.o                  >/dev/null 2>/dev/null
	-@rm *.a                  >/dev/null 2>/dev/null
	@echo "Clean OK."
