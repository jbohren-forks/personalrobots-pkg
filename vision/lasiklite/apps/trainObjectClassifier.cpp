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
** FILENAME:    trainObjectClassifier.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**   Trains an object classifier using OpenCV's machine learning library.
**
*****************************************************************************/

#include <cstdlib>
#include <cassert>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <sys/types.h>
#include <vector>
#include <dirent.h>

#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

#include "svlLib.h"

using namespace std;

vector<vector<double> > readFromCache(const char* dirName, int maxImages);

void usage()
{
    cerr << "USAGE: ./trainObjectClassifier [OPTIONS] <+ dir> <- dir>" << endl;    
    cerr << "OPTIONS:" << endl
	 << "  -c <type>          :: classifier type: GABOR, PATCH (default)" << endl
	 << "  -cached            :: directories hold cached feature vectors (not images)" << endl
	 << "  -i <filename>      :: input classifier model (does validation only)" << endl
	 << "  -o <filename>      :: output classifier model" << endl
	 << "  -p <filename>      :: input parameter model (e.g. dictionary for patch-based classifiers" << endl
	 << "  -maxImages <num>   :: maximum training images for each class (default: 1000)" << endl
	 << "  -rounds <num>      :: number of rounds of boosting (default: 50)" << endl
	 << "  -splits <num>      :: number of splits in weak learners (default: 2)" << endl
	 << "  -subrects          :: include subrectangles as negatives at detector scale" << endl
	 << endl;
    cerr << "Copyright (c) 2007-2008, Stephen Gould" << endl << endl;
}

int main(int argc, char *argv[])
{
    // read commandline parameters
    const int NUM_REQUIRED_PARAMETERS = 2;

    const char *posDir = NULL;
    const char *negDir = NULL;

    const char *classifierType = "patch";
    bool bCached = false;
    const char *inputFilename = NULL;
    const char *outputFilename = NULL;
    const char *paramFilename = NULL;
    int boostingRounds = 50;
    int weakLearnerSplits = 2;
    int maxTrainingImages = 1000;
    bool bIncludeSubRects = false;

    char **args = argv + 1;
    while (--argc > NUM_REQUIRED_PARAMETERS) {
        if (!strcmp(*args, "-c")) {
            classifierType = *(++args); argc--;
        } else if (!strcmp(*args, "-c")) {
            bCached = true;
        } else if (!strcmp(*args, "-i")) {
            inputFilename = *(++args); argc--;
        } else if (!strcmp(*args, "-o")) {
            outputFilename = *(++args); argc--;
        } else if (!strcmp(*args, "-p")) {
            paramFilename = *(++args); argc--;
	} else if (!strcmp(*args, "-cached")) {
	    bCached = true;
        } else if (!strcmp(*args, "-rounds")) {
            boostingRounds = atoi(*(++args)); argc--;
        } else if (!strcmp(*args, "-splits")) {
            weakLearnerSplits = atoi(*(++args)); argc--;
        } else if (!strcmp(*args, "-maxImages")) {
            maxTrainingImages = atoi(*(++args)); argc--;
	} else if (!strcmp(*args, "-subrects")) {
	    bIncludeSubRects = true;
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

    posDir = args[0];
    negDir = args[1];

    // define classifier type
    svlObjectDetector *classifier = NULL;
    if (!strcasecmp(classifierType, "gabor")) {
	cerr << "NOT IMPLEMENTED" << endl;
	assert(false);
    } else if (!strcasecmp(classifierType, "patch")) {
        classifier = new svlPatchBasedClassifier();
        if (paramFilename != NULL) {
            ((svlPatchBasedClassifier *)classifier)->readDictionary(paramFilename);
        }
    }
    assert(classifier != NULL);

    if (inputFilename != NULL) {
        cerr << "Loading model file " << inputFilename << endl;
        classifier->readModel(inputFilename);
    }

    // set classifier learning options
    classifier->setOption("boostingRounds", boostingRounds);
    classifier->setOption("numSplits", weakLearnerSplits);
    classifier->setOption("maxTrainingImages", maxTrainingImages);
    classifier->setOption("trainOnNegSubRects", bIncludeSubRects);
    classifier->setOption("trainOnPosSubRects", bIncludeSubRects);

    if (bCached) {
        // load positive and negative training samples from cache
        vector<vector<double> > posSamples = readFromCache(posDir, maxTrainingImages);
        vector<vector<double> > negSamples = readFromCache(negDir, maxTrainingImages);
        classifier->learnModel(posSamples, negSamples);
    } else {
        // load positive and negative training samples from images
        classifier->learnModel(posDir, negDir, ".jpg");
    }

    if (outputFilename != NULL) {
        cerr << "Writing model to file " << outputFilename << endl;
        classifier->writeModel(outputFilename);
    }

    // free memory
    delete classifier;

    return 0;
}


vector<vector<double> > readFromCache(const char *dirName, int maxImages)
{
    vector<vector<double> > fv;

    DIR *dir = opendir(dirName);
    if (dir == NULL) {
	cerr << "ERROR: could not open image directory " << dirName << endl;
	exit(-1);
    }
    
    struct dirent *e = readdir(dir);
    while ((e != NULL) && ((int)fv.size() < maxImages)) {
	if (strstr(e->d_name, ".txt") != NULL) {
	    string inputFilename = string(dirName) + string("/") + string(e->d_name);
	    ifstream ifs(inputFilename.c_str());
	    if (ifs.fail()) {
		cerr << "...failed" << endl;
	    } else {
		vector<double> v;
		double d;
		while (1) {
		    ifs >> d;
		    if (ifs.fail()) break;
		    v.push_back(d);
		}
		fv.push_back(v);
	    }
	}
	e = readdir(dir);
    }
    closedir(dir);

    return fv;
}

