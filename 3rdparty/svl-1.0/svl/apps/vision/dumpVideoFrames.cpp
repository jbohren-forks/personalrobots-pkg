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
** FILENAME:    dumpVideoFrames.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**  Dumps undistorted video frames as JPEGs to disk. Can also be used for
**  extracting objects out of video frames.
**
*****************************************************************************/

#include <cstdlib>
#include <string>
#include <limits>
#include <iomanip>
#include <map>

#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

#include "svlBase.h"
#include "svlVision.h"

using namespace std;

#define WINDOW_NAME "dumpVideoFrames"

void usage()
{
    cerr << "USAGE: ./dumpVideoFrames [OPTIONS] <video>" << endl;
    cerr << "OPTIONS:" << endl
	 << "  -stem <filestem>   :: filestem (default: frame)" << endl
	 << "  -undistort <file>  :: undistort the frames (using intrinsics in file)" << endl
	 << "  -start <timestamp> :: start frame" << endl
	 << "  -end <timestamp>   :: end frame" << endl
	 << "  -step <n>          :: step to every n-th frame (default 1)" << endl
	 << "  -objects <file>    :: snip objects out of frame" << endl
	 << "  -dir <directory>   :: output directory" << endl
	 << "  -v                 :: verbose (show video)" << endl
	 << endl;
}

int main(int argc, char *argv[])
{
    // read command line parameters
    const int NUM_REQUIRED_PARAMETERS = 1;

    const char *aviFilename = NULL;
    const char *outputFilestem = "frame";
    const char *outputDir = ".";

    bool bVerbose = false;
    int startFrame = 0;
    int endFrame = numeric_limits<int>::max();
    const char *intrinsicsFilename = NULL;
    const char *objectsFilename = NULL;
    int frameStep = 1;

    char **args = argv + 1;
    while (--argc > NUM_REQUIRED_PARAMETERS) {
	if (!strcmp(*args, "-stem")) {
	    outputFilestem = *(++args); argc--;
	} else if (!strcmp(*args, "-undistort")) {
	    intrinsicsFilename = *(++args); argc--;
	} else if (!strcmp(*args, "-start")) {
	    startFrame = atoi(*(++args)); argc--;
	} else if (!strcmp(*args, "-end")) {
	    endFrame = atoi(*(++args)); argc--;
	} else if (!strcmp(*args, "-step")) {
	    frameStep = atoi(*(++args)); argc--;
	    if (frameStep < 1) frameStep = 1;
	} else if (!strcmp(*args, "-objects")) {
	    objectsFilename = *(++args); argc--;
	} else if (!strcmp(*args, "-dir")) {
	    outputDir = *(++args); argc--;
	} else if (!strcmp(*args, "-v")) {
	    bVerbose = true;
	} else {
	    cerr << "ERROR: unrecognized option " << *args << endl;
	    return -1;
	}
	args++;
    }

    if ((argc != NUM_REQUIRED_PARAMETERS) || (startFrame > endFrame)) {
	usage();
	return -1;
    }

    aviFilename = args[0];

    svlCameraIntrinsics intrinsics;
    if (intrinsicsFilename != NULL) {
	intrinsics.initialize(intrinsicsFilename);
    }

    CvCapture *capture = cvCaptureFromAVI(aviFilename);    
    if (capture == NULL) {
	cerr << "ERROR: couldn't open video" << endl;
	return -1;
    }

    svlObject2dSequence objects;
    if (objectsFilename != NULL) {
	readObject2dSequence(objectsFilename, objects);
	cerr << "Read " << objects.size() << " labeled frames from " << objectsFilename << endl;
    }

    if (bVerbose) {
        cvNamedWindow(WINDOW_NAME, CV_WINDOW_AUTOSIZE);
    }

    // process video
    int index = 0;
    char *buffer = new char[strlen(outputFilestem) + 256];
    map<string, int> objectCount;
    while (index <= endFrame) {
	const IplImage *imgPtr;
	if ((imgPtr = cvQueryFrame(capture)) == NULL) {
	    break;
	}

	if (intrinsicsFilename != NULL) {
	    imgPtr = intrinsics.undistort(imgPtr);
	}
	if ((index >= startFrame) && (index % frameStep == 0)) {
	    // save frame
	    sprintf(buffer, "%s/%s%05d.jpg", outputDir, outputFilestem, index);
	    cerr << "Writing " << buffer << "..." << endl;
	    cvSaveImage(buffer, imgPtr);

	    // snip out objects and save images
            if (objects.find(toString(index)) != objects.end()) {
                svlObject2dFrame frameObjects = objects[toString(index)];
		IplImage *imgCopy = cvCloneImage(imgPtr);
		for (svlObject2dFrame::const_iterator it = frameObjects.begin();
		     it != frameObjects.end(); it++) {
		    if (objectCount.find(it->name) == objectCount.end()) {
			objectCount[it->name] = 0;
		    }
		    CvRect region = cvRect((int)it->x - 2, (int)it->y - 2, (int)it->w + 4, (int)it->h + 4);
		    svlClipRect(region, imgCopy);
		    IplImage *imgSnip = cvCreateImage(cvSize(region.width, region.height), 
			imgCopy->depth, imgCopy->nChannels);
		    cvSetImageROI(imgCopy, region);
		    cvCopy(imgCopy, imgSnip);
		    sprintf(buffer, "%s/%s%05d.jpg", outputDir, it->name.c_str(), objectCount[it->name]);
		    cerr << "Writing " << buffer << "..." << endl;
		    cvSaveImage(buffer, imgSnip);
		    objectCount[it->name] = objectCount[it->name] + 1;
		    cvReleaseImage(&imgSnip);
		}
		cvReleaseImage(&imgCopy);
	    }
	}
        
	if (bVerbose) {
	    cvShowImage(WINDOW_NAME, imgPtr);
	    if (cvWaitKey(5) == 27) {
		return 0;
	    }
	}

	index += 1;
    }

    if (bVerbose) {
        cvDestroyWindow(WINDOW_NAME);
    }

    delete buffer;
    cvReleaseCapture(&capture);
    
    return 0;
}

