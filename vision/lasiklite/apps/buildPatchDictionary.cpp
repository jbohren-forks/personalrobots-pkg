/*****************************************************************************
** STAIR VISION PROJECT
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
** DISCLAIMED. IN NO EVENT SHALL <copyright holder> BE LIABLE FOR ANY
** DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
** (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
** LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
** ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
** SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
*****************************************************************************
**
** FILENAME:    buildPatchDictionary.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**  Build patch dictionary for object recognition by taking random snips from
**  all images in a given directory.
**
*****************************************************************************/

#include <cstdlib>
#include <cassert>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <limits>
#include <string>
#include <dirent.h>

#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

#include "svlLib.h"

using namespace std;

#define WINDOW_NAME "buildPatchDictionary"

void usage()
{
    cerr << "USAGE: ./buildPatchDictionary [OPTIONS] <dir> <width> <height>" << endl;
    cerr << "OPTIONS:" << endl
	 << "  -debug             :: debug mode (visualize dictionary)" << endl
	 << "  -i <filename>      :: input dictionary file" << endl
	 << "  -o <filename>      :: output dictionary file" << endl
	 << "  -n <num>           :: number of patches per image (default: 10)" << endl
	 << "  -max_size <num>    :: maximum size of dictionary" << endl
	 << endl;
    cerr << "Copyright (c) 2007-2008, Stephen Gould" << endl << endl;
}

int main(int argc, char *argv[])
{
    // read commandline parameters
    const int NUM_REQUIRED_PARAMETERS = 3;

    const char *imageDir = NULL;
    unsigned width;
    unsigned height;

    bool bDebug = false;
    const char *inputFilename = NULL;
    const char *outputFilename = NULL;
    int numPatchesPerImage = 10;
    unsigned maxSize = numeric_limits<unsigned>::max();

    char **args = argv + 1;
    while (--argc > NUM_REQUIRED_PARAMETERS) {
	if (!strcmp(*args, "-debug")) {
	    bDebug = true;
	} else if (!strcmp(*args, "-i")) {
	    inputFilename = *(++args); argc--;
	} else if (!strcmp(*args, "-o")) {
	    outputFilename = *(++args); argc--;
	} else if (!strcmp(*args, "-n")) {
	    numPatchesPerImage = atoi(*(++args)); argc--;
	} else if (!strcmp(*args, "-max_size")) {
	    maxSize = atoi(*(++args)); argc--;
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

    imageDir = args[0];
    width = atoi(args[1]);
    height = atoi(args[2]);

    // read and resize images
    vector<IplImage *> images;
    DIR *dir = opendir(imageDir);
    if (dir == NULL) {
	cerr << "ERROR: could not open image directory " << imageDir << endl;
	exit(-1);
    }
    
    struct dirent *e = readdir(dir);

    while (e != NULL) 
	{
	if (strstr(e->d_name, ".jpg") != NULL) 
	{
	    cerr << "Adding " << e->d_name << " to image database..." << endl;
	    string imageFilename = string(imageDir) + string("/") + string(e->d_name);
	    IplImage *img = cvLoadImage(imageFilename.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
	    if (img == NULL) 
		{
		cerr << "...failed" << endl;
	    }
		else 
		{
		if ((img->width != (int)width) || (img->height != (int)height)) 
		{
		    IplImage *resizedImage = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
		    cvResize(img, resizedImage);
		    cvReleaseImage(&img);
		    img = resizedImage;
		}
		images.push_back(img);		
	    }
	}
	e = readdir(dir);
    }
    closedir(dir);

    cerr << "...read " << images.size() << " images from " << imageDir << endl;

    // build dictionary
    svlPatchDictionary dictionary(width, height);

    if (inputFilename != NULL) 
	{
	cerr << "Reading dictionary from " << inputFilename << endl;
	dictionary.read(inputFilename);
	cerr << "..." << dictionary.numEntries() << " entries read" << endl;
	assert((dictionary.windowWidth() == width) && (dictionary.windowHeight() == height));
	if (dictionary.numEntries() > maxSize) 
	{
	    dictionary.truncate(maxSize);
	}
    }
    cerr << "Building dictionary..." << endl;
    
	dictionary.buildDictionary(images, numPatchesPerImage);
    
	if (dictionary.numEntries() > maxSize)
	{
		dictionary.truncate(maxSize);
    }
    cerr << "..." << dictionary.numEntries() << " entries" << endl << "...done" << endl;

    if (outputFilename != NULL) 
	{
		cerr << "Writing dictionary to " << outputFilename << endl;
		dictionary.write(outputFilename);
    }

    if (bDebug) 
	{
	cvNamedWindow(WINDOW_NAME, 1);
	IplImage *dicImages = dictionary.visualizeDictionary();
	cvShowImage(WINDOW_NAME, dicImages);
	cvWaitKey(0);
	cvReleaseImage(&dicImages);    

	cvDestroyWindow(WINDOW_NAME);
    }

    // free memory
    for (unsigned i = 0; i < images.size(); i++) {
	cvReleaseImage(&images[i]);
    }

    return 0;
}

