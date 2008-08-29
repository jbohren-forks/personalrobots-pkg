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
** FILENAME:    dir2imageSeq.cpp
** AUTHOR(S):   Sid Batra <sidbatra@cs.stanford.edu>
** DESCRIPTION:
**   Takes an image directory and converts all the images to an svlImageSequence object.
**
*****************************************************************************/

#include <cstdlib>
#include <cassert>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <sys/types.h>
#include <vector>

#if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
#include "win32/dirent.h"
#else
#include <dirent.h>
#endif

#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

#include "svlBase.h"
#include "svlML.h"
#include "svlVision.h"

using namespace std;



// Main -----------------------------------------------------------------------

void usage()
{
    cerr << SVL_USAGE_HEADER << endl;
    cerr << "USAGE: ./dir2imageSeq [OPTIONS] <image dir> <output filename>" << endl;
    cerr << "OPTIONS:" << endl
	 << "  -ext <string>           :: extension of the images to be read (default .JPG)" << endl         
	 << endl;
}

int main(int argc, char *argv[])
{
    // read commandline parameters
    const int NUM_REQUIRED_PARAMETERS = 2;

    const char *imageDir = NULL;
    const char *outputFilename = NULL;
    const char *extension = ".jpg";    
    
    char **args = argv + 1;

    while (--argc > NUM_REQUIRED_PARAMETERS) 
    {
        if (!strcmp(*args, "-ext")) 
        {
            extension = *(++args); argc--;
        }
        args++;
    }

    if (argc != NUM_REQUIRED_PARAMETERS) 
    {
        usage();
        return -1;
    }

    imageDir = args[0];
    outputFilename = args[1];


    //Load images of desired extension and save to file
    svlImageSequence result;

    result.dir(imageDir,extension);

    result.save(outputFilename);
    
}

