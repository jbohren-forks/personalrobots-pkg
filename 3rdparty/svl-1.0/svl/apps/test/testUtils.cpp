/*****************************************************************************
** STAIR VISION LIBRARY
** Copyright (c) 2007-2008, Stephen Gould
** All rights reserved.
**
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are met:
**     * Redistributions of source code must retain the above copyright
**       notice, this list of conditions and the following disclaimer.
**     * Redistributions in binary form must reproduce the above copyright
**       notice, this list of conditions and the following disclaimer in the
**       documentation and/or other materials provided with the distribution.
**     * Neither the name of the Stanford University nor the
**       names of its contributors may be used to endorse or promote products
**       derived from this software without specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
** EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
** WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
** DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY
** DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
** (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
** LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
** ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
** SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
******************************************************************************
** FILENAME:    testUtils.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**  Regression tests for utilities.
**
*****************************************************************************/

#include <cstdlib>
#include <cassert>
#include <iostream>
#include <fstream>
#include <iomanip>

#include "svlBase.h"

using namespace std;

int main(int argc, char *argv[])
{
    // test successors
    vector<int> v;
    
    v.resize(5);
    fill(v.begin(), v.end(), 0);
    
    for (int i = 0; i < 100; i++) {
        cout << toString(v) << endl;
        successor(v, 2);
    }

    // test filename string utilities
    const string fullpath = "/tmp/lasik.test/lasik.file0042.txt";
    cout << "FULLPATH:  " << fullpath << "\n"
	 << "DIRECTORY: " << strDirectory(fullpath) << "\n"
	 << "FILENAME:  " << strFilename(fullpath) << "\n"
	 << "BASENAME:  " << strBaseName(fullpath) << "\n"
	 << "EXTENSION: " << strExtension(fullpath) << "\n"
	 << "INDEX:     " << strFileIndex(fullpath) << "\n"
	 << endl;

    // test profiler
    svlCodeProfiler::enabled = true;
    v.resize(32);
    fill(v.begin(), v.end(), 0); 

    int hSuccessor = svlCodeProfiler::getHandle("successor");
    for (int j = 0; j < 1e6; j++) {
	svlCodeProfiler::tic(hSuccessor);
	for (int i = 0; i < (0x01 << 31); i++) {
	    successor(v, 2);
	}
	svlCodeProfiler::toc(hSuccessor);
    }

    svlCodeProfiler::print(cerr);
    return 0;
}


