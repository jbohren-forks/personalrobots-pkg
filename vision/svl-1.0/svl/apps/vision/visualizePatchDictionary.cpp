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
** FILENAME:    visualizePatchDictionary.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**  Application for visualizing a patch dictionary.
**
*****************************************************************************/

#include <cstdlib>
#include <cassert>
#include <iostream>
#include <fstream>
#include <iomanip>

#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

#include "svlBase.h"
#include "svlVision.h"

using namespace std;

#define WINDOW_NAME "visualizePatchDictionary"

void usage()
{
    cerr << SVL_USAGE_HEADER << endl;
    cerr << "USAGE: ./visualizePatchDictionary [OPTIONS] <dictionary>" << endl;
    cerr << "OPTIONS:" << endl
         << "  -s <scale>         :: rescale image by <scale>" << endl
	 << "  -o <filename>      :: output to file" << endl
	 << endl;
}

int main(int argc, char *argv[])
{
    const int PATCH_ROWS = 20;
    const int PATCH_COLS = 15;

    // read commandline parameters
    const int NUM_REQUIRED_PARAMETERS = 1;

    const char *dictionaryFilename = NULL;
    const char *outputFilename = NULL;
    double scale = 1.0;

    char **args = argv + 1;
    while (--argc > NUM_REQUIRED_PARAMETERS) {
	if (!strcmp(*args, "-o")) {
	    outputFilename = *(++args); argc--;
        } else if (!strcmp(*args, "-s")) {
            scale = atof(*(++args)); argc--;
	} else {
	    SVL_LOG(SVL_LOG_FATAL, "unrecognized option " << *args);
	}
	args++;
    }

    if (argc != NUM_REQUIRED_PARAMETERS) {
	usage();
	return -1;
    }

    dictionaryFilename = args[0];

    // load dictionary
    svlPatchDictionary dictionary;
    SVL_LOG(SVL_LOG_MESSAGE, "Loading dictionary " << dictionaryFilename << "...");
    dictionary.read(dictionaryFilename);
    SVL_LOG(SVL_LOG_MESSAGE, "..." << dictionary.numEntries() << " entries in dictionary");

    // display patch features
    IplImage *image;
    image = cvCreateImage(cvSize(PATCH_ROWS * dictionary.windowWidth(),
	    PATCH_COLS * dictionary.windowHeight()), IPL_DEPTH_8U, 3);
    cvZero(image);

    cvNamedWindow(WINDOW_NAME, 1);    
    for (int i = 0; i < (int)dictionary.numEntries(); i++) {
	int x = (i % PATCH_ROWS) * dictionary.windowWidth();
	int y = ((i / PATCH_ROWS) % PATCH_COLS) * dictionary.windowHeight();

	// patch template
	IplImage *patch = dictionary.visualizePatch(i);
	cvSetImageROI(image, cvRect(x, y, dictionary.windowWidth(), dictionary.windowHeight()));
	cvCopyImage(patch, image);
	cvResetImageROI(image);
	cvReleaseImage(&patch);

	// boundary
	cvRectangle(image, cvPoint(x, y), cvPoint(x + dictionary.windowWidth(), y + dictionary.windowHeight()),
	    CV_RGB(0, 0, 255), 1);

	if ((i + 1) % (PATCH_ROWS * PATCH_COLS) == 0) {
	    cvShowImage(WINDOW_NAME, image);
	    cvWaitKey(0);
	    cvZero(image);
	}
    }

    if (dictionary.numEntries()  % (PATCH_ROWS * PATCH_COLS) != 0) {
        if (scale != 1.0) {
            resizeInPlace(&image, (int)(image->height * scale), (int)(image->width * scale));
        }
	cvShowImage(WINDOW_NAME, image);
	cvWaitKey(0);
    }	
    cvDestroyWindow(WINDOW_NAME);

    // output images
    if (outputFilename) {
        cvSaveImage(outputFilename, image);
    }

    // free memory
    cvReleaseImage(&image);

    return 0;
}


