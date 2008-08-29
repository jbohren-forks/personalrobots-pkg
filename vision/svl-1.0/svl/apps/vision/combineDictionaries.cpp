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
** FILENAME:    combineDictionaries.cpp
** AUTHOR(S):   Sid Batra <sidbatra@cs.stanford.edu>
**
** DESCRIPTION:
**  Combines multiple patch dictionaries and sets different channels for each

*****************************************************************************/

#include <cstdlib>
#include <cassert>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <limits>
#include <string>

#include <sys/types.h>
#if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
#include "win32/dirent.h"
#include <windows.h>
#else
#include <dirent.h>
#endif

#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

#include "svlBase.h"
#include "svlVision.h"

using namespace std;


void usage()
{
    cerr << SVL_USAGE_HEADER << endl;
    cerr << "USAGE: ./combineDictionaries [OPTIONS] <outputDict> (<inputDict> <taregetChannelNumber>)+" << endl;
    cerr << "OPTIONS:" << endl
         << endl;
}


int main(int argc, char *argv[])
{
    // read commandline parameters
    const int NUM_REQUIRED_PARAMETERS = 3;
             
    char **args = argv + 1;
  
    argc--;

    if (argc < NUM_REQUIRED_PARAMETERS) 
    {
	    usage();
	    return -1;
    }

    svlPatchDictionary target;


    //Load output filename
    const char *outputDict = args[0];

            

    //Iterater over each of the input dictionaries and append them to the final version
    const char *inputDict = NULL;
    int targetChannel = -1;

    for( int i=1 ; i<argc ; i+=2)
    {
        inputDict = args[i];
        targetChannel = atoi(args[i+1]);

        cout<<"\n Input dictionary "<<inputDict<<" to channel index "<<targetChannel;
        
        
        svlPatchDictionary input;
        input.read(inputDict);

        //Change the valid channel for the entire dictionary
        for( int j=0 ; j<input.numEntries() ; j++)
            input.alterValidChannel(targetChannel);
        

        if( i == 1 )
            target = svlPatchDictionary(input);
        else
        {
            target.mergeDictionary(input);
        }
        
    }

    //save merged dictionary to disk
    target.write(outputDict);

    return 0;
}
