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
** FILENAME:    trainObjectDetector.cpp
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

// Prototypes -----------------------------------------------------------------

vector<vector<double> > readFromCache(const char* dirName, int maxImages);

// Main -----------------------------------------------------------------------

void usage()
{
    cerr << SVL_USAGE_HEADER << endl;
    cerr << "USAGE: ./trainObjectDetector [OPTIONS] <+ cache dir> <- cache dir>" << endl;
    cerr << "OPTIONS:" << endl
	 << "  -c <type>           :: classifier type: PATCH (default)" << endl
         << "  -i [<dict>] <model> :: evaluate input classifier model (for debugging)" << endl
	 << "  -o <filename>       :: output classifier model" << endl
	 << "  -maxInstances <n>   :: maximum training images for each class (default: 1000)" << endl
	 << "  -rounds <n>         :: number of rounds of boosting (default: 50)" << endl
	 << "  -splits <n>         :: number of splits in weak learners (default: 2)" << endl
         << "  -v                  :: verbose" << endl
	 << endl;
}

int main(int argc, char *argv[])
{
    // read commandline parameters
    const int NUM_REQUIRED_PARAMETERS = 2;

    const char *posCacheDir = NULL;
    const char *negCacheDir = NULL;

    const char *classifierType = "patch";
    const char *inputFilename = NULL;
    const char *dictionaryFilename = NULL;
    const char *outputFilename = NULL;    
    int boostingRounds = 50;
    int weakLearnerSplits = 2;
    int maxTrainingInstances = 1000;

    char **args = argv + 1;
    while (--argc > NUM_REQUIRED_PARAMETERS) {
        if (!strcmp(*args, "-c")) {
            classifierType = *(++args); argc--;
        } else if (!strcmp(*args, "-i")) {
            if (!strcasecmp(classifierType, "patch")) {
                dictionaryFilename = *(++args); argc--;
            }
            inputFilename = *(++args); argc--;
        } else if (!strcmp(*args, "-o")) {
            outputFilename = *(++args); argc--;
        } else if (!strcmp(*args, "-rounds")) {
            boostingRounds = atoi(*(++args)); argc--;
        } else if (!strcmp(*args, "-splits")) {
            weakLearnerSplits = atoi(*(++args)); argc--;
        } else if (!strcmp(*args, "-maxInstances")) {
            maxTrainingInstances = atoi(*(++args)); argc--;
        } else if (!strcmp(*args, "-v")) {
            svlLogger::setLogLevel(SVL_LOG_VERBOSE);
        } else {
            SVL_LOG(SVL_LOG_FATAL, "unrecognized option " << *args);
        }
        args++;
    }

    if (argc != NUM_REQUIRED_PARAMETERS) {
        usage();
        return -1;
    }

    posCacheDir = args[0];
    negCacheDir = args[1];

    // define classifier type
    svlSlidingWindowDetector *classifier = NULL;
    if (!strcasecmp(classifierType, "patch")) 
	{
        classifier = new svlPatchBasedObjectDetector();
    } 
	else 
	{
        SVL_LOG(SVL_LOG_FATAL, "classifier type " << classifierType << " not supported");
    }
    assert(classifier != NULL);

    // load positive and negative training samples from cache
    SVL_LOG(SVL_LOG_VERBOSE, "Reading positive samples...");
    vector<vector<double> > posSamples = readFromCache(posCacheDir, maxTrainingInstances);
    SVL_LOG(SVL_LOG_VERBOSE, "..." << posSamples.size() << " samples read");
    SVL_LOG(SVL_LOG_VERBOSE, "Reading negative samples...");
    vector<vector<double> > negSamples = readFromCache(negCacheDir, maxTrainingInstances);
    SVL_LOG(SVL_LOG_VERBOSE, "..." << negSamples.size() << " samples read");

    // evaluate model
    if (inputFilename != NULL)
    {
        classifier->readModel(inputFilename);
        if (!strcasecmp(classifierType, "patch")) 
        {
            assert(dictionaryFilename != NULL);
            ((svlPatchBasedObjectDetector *)classifier)->readDictionary(dictionaryFilename);
        }
        
        int tp = 0, tn = 0, fp = 0, fn = 0;
        for (unsigned i = 0; i < posSamples.size(); i++) {
            if (classifier->evaluateModel(posSamples[i]) > 0.0) {
                tp += 1;
            } else {
                fn += 1;
            }
        }
        for (unsigned i = 0; i < negSamples.size(); i++) {
            if (classifier->evaluateModel(negSamples[i]) > 0.0) {
                fp += 1;
            } else {
                tn += 1;
            }
        }
        SVL_LOG(SVL_LOG_MESSAGE, "Evaluation (TP, FN, TN, FP): " << tp << ", " << fn << ", " << tn << ", " << fp);        

        if (outputFilename == NULL) {
            delete classifier;
            return 0;
        }
    }

    // set classifier learning options
    classifier->setOption("boostingRounds", boostingRounds);
    classifier->setOption("numSplits", weakLearnerSplits);
    classifier->setOption("maxTrainingImages", maxTrainingInstances);

    SVL_LOG(SVL_LOG_VERBOSE, "Training classifier...");
    classifier->learnModel(posSamples, negSamples);

    if (outputFilename != NULL) {
        SVL_LOG(SVL_LOG_VERBOSE, "Writing model to file " << outputFilename);
        classifier->writeModel(outputFilename);
    }

    // free memory
    delete classifier;

    return 0;
}

// Helper functions ----------------------------------------------------------

vector<vector<double> > readFromCache(const char *dirName, int maxImages)
{
    vector<vector<double> > fv;

    DIR *dir = opendir(dirName);
    if (dir == NULL) {
        SVL_LOG(SVL_LOG_FATAL, "could not open image directory " << dirName);
    }
    
    struct dirent *e = readdir(dir);
    while ((e != NULL) && ((int)fv.size() < maxImages)) 
	{
	if (strstr(e->d_name, ".txt") != NULL) 
    {
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


