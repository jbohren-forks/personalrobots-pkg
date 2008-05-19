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
** FILENAME:    buildPatchResponseCache.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**  Build a cache of patch responses.
**
*****************************************************************************/

#include <cstdlib>
#include <cassert>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <sys/types.h>
#include <dirent.h>

#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

#include "svlPatchDictionary.h"

using namespace std;

void usage()
{
    cerr << "USAGE: ./buildPatchResponseCache [OPTIONS] <image dir> <output dir> <dictionary>" << endl;
    cerr << "OPTIONS:" << endl
	 << "  -maxImages <num>   :: maximum training images to cache (default: 10000)" << endl
	 << endl;
    cerr << "Copyright (c) 2007-2008, Stephen Gould" << endl << endl;
}

int main(int argc, char *argv[])
{
    // read commandline parameters
    const int NUM_REQUIRED_PARAMETERS = 3;

    const char *imageDir = NULL;
    const char *outputDir = NULL;
    const char *dictionaryFilename = NULL;
    int maxImages = 10000;

    char **args = argv + 1;
    while (--argc > NUM_REQUIRED_PARAMETERS) {
        if (!strcmp(*args, "-maxImages")) {
            maxImages = atoi(*(++args)); argc--;
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
    outputDir = args[1];
    dictionaryFilename = args[2];

    // read dictionary
    svlPatchDictionary dictionary;
    dictionary.read(dictionaryFilename);
    cerr << dictionary.numEntries() << " patches in dictionary" << endl;

    int width = dictionary.windowWidth();
    int height = dictionary.windowHeight();

    // read, resize and process images
    DIR *dir = opendir(imageDir);
    if (dir == NULL) {
	cerr << "ERROR: could not open image directory " << imageDir << endl;
	exit(-1);
    }
    
    int numImages = 0;
    struct dirent *e = readdir(dir);

    while ((e != NULL) && (numImages < maxImages)) 
	{
	if (strstr(e->d_name, ".jpg") != NULL) 
	{
	    cerr << "Processing " << e->d_name << "..." << endl;
	    string imageFilename = string(imageDir) + string("/") + string(e->d_name);
	    IplImage *img = cvLoadImage(imageFilename.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
	    
		if (img == NULL) 
		{
		cerr << "...failed" << endl;
	    } 
		else 
		{
		// resize image 
		if ((img->width != width) || (img->height != height)) 
		{
		    IplImage *resizedImage = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
		    cvResize(img, resizedImage);
		    cvReleaseImage(&img);
		    img = resizedImage;
		}
		// process image
		vector<double> v = dictionary.patchResponse(img);
		// write feature vector
		string outputFilename = string(outputDir) + string("/") + string(e->d_name);
		outputFilename.replace(outputFilename.size() - 3, 3, "txt");
		cerr << "...writing output to " << outputFilename.c_str() << endl;
		ofstream ofs(outputFilename.c_str());
		if (ofs.fail()) {
		    cerr << "...failed" << endl;
		} else {
		    for (unsigned i = 0; i < v.size(); i++) {
			ofs << v[i] << endl;
		    }
		    numImages += 1;
		}
		ofs.close();
		// free memory
		cvReleaseImage(&img);
	    }
	}
	e = readdir(dir);
    }
    closedir(dir);

    return 0;
}
