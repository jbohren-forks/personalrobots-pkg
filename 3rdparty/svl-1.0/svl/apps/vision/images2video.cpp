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
** FILENAME:    images2video.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**              Sid Batra <sidbatra@cs.stanford.edu>
** DESCRIPTION:
**  Application for creating video from images in a directory.
**
*****************************************************************************/

#include <cstdlib>
#include <cassert>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <limits>
#include <string>
#include <algorithm>

#include <sys/types.h>
#if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
#include "win32/dirent.h"
#else
#include <dirent.h>
#endif

#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

#include "svlBase.h"

using namespace std;

#define WINDOW_NAME "Video"

void usage()
{
    cerr << "USAGE: ./images2video [OPTIONS] <input dir> <output video>" << endl;
    cerr << "OPTIONS:" << endl
	 << "  -blanking <n>      :: number of initial blank frames" << endl
	 << "  -flip              :: flip frames horizontally" << endl
	 << "  -x                 :: display frames during encoding" << endl
	 << "  -ext               :: extension of the video frame images" << endl
	 << endl;
}

int main(int argc, char *argv[])
{
    // read commandline parameters
    const int NUM_REQUIRED_PARAMETERS = 2;

    const char *inputDirectory = NULL;
    const char *outputFilename = NULL;
	char *extension = ".jpg";
    bool bDisplayImage = false;
    int blankingCount = 0;
    bool bFlipFrames = false;

    char **args = argv + 1;
    while (--argc > NUM_REQUIRED_PARAMETERS) {
	if (!strcmp(*args, "-x")) {
	    bDisplayImage = true;
	} else if (!strcmp(*args, "-blanking")) {
	    blankingCount = atoi(*(++args)); argc -= 1;
	}
	else if (!strcmp(*args, "-ext")) {
	    extension = *(++args); argc -= 1;
	}else if (!strcmp(*args, "-flip")) {
	    bFlipFrames = true;
	} else {
	    cerr << "ERROR: unrecognized option " << *args << endl;
	    return -1;
	}
	args++;
    }

    if (argc != NUM_REQUIRED_PARAMETERS) {
	usage();
	return -1;
    }

    inputDirectory = args[0];
    outputFilename = args[1];

    if (bDisplayImage) {
	cvNamedWindow(WINDOW_NAME, 1);
    }

    CvVideoWriter *videoWriter = NULL;

    // create sorted list of image filenames
    vector<string> imageFilenames;
    DIR *dir = opendir(inputDirectory);
    assert(dir != NULL);
    struct dirent *e = readdir(dir);
    while (e != NULL) {
	if (strstr(e->d_name, extension) != NULL) {
	    imageFilenames.push_back(string(e->d_name));	    
	}
	e = readdir(dir);
    }
    closedir(dir);
    sort(imageFilenames.begin(), imageFilenames.end());


    for (vector<string>::const_iterator it = imageFilenames.begin();
	 it != imageFilenames.end(); ++it) {
	// load image
	string filename = string(inputDirectory).append(string("/")).append(*it);
        SVL_LOG(SVL_LOG_MESSAGE, filename);
	IplImage *frame = cvLoadImage(filename.c_str());
	if (bFlipFrames) 
	    cvFlip(frame, NULL, 1);

	// show image
	if (bDisplayImage) {
	    cvShowImage(WINDOW_NAME, frame);
	    cvWaitKey(30);
	}
	// encode image
	if (videoWriter == NULL) {
#if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
	    videoWriter = cvCreateVideoWriter(outputFilename, -1, 15.0, cvSize(frame->width, frame->height));
#else
	    videoWriter = cvCreateVideoWriter(outputFilename, 0, 15.0, cvSize(frame->width, frame->height));
#endif
	}
	
	if (videoWriter == NULL) {
            SVL_LOG(SVL_LOG_FATAL, "could not open AVI file for writing " << outputFilename);
	}
	
	if (blankingCount > 0) {
	    IplImage *blankFrame = cvCloneImage(frame);
	    cvZero(blankFrame);
	    while (blankingCount-- > 0) {
		cvWriteFrame(videoWriter, blankFrame);
	    }
	    cvReleaseImage(&blankFrame);
	}

	cvWriteFrame(videoWriter, frame);
	cvReleaseImage(&frame);
    }
   
    // free memory
    if (bDisplayImage) {
        cvDestroyWindow(WINDOW_NAME);
    }

    cvReleaseVideoWriter(&videoWriter);
    
    return 0;
}


