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
** FILENAME:    trainClassifier.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**  Application for training a logistic or boosted classifier.
**
*****************************************************************************/

#include <cstdlib>
#include <cassert>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <algorithm>
#include <vector>

#include "opencv/cv.h"
#include "opencv/cxcore.h"
#include "opencv/ml.h"

#include "svlBase.h"
#include "svlML.h"

using namespace std;

void usage()
{
    cerr << SVL_USAGE_HEADER << endl;
    cerr << "USAGE: ./trainClassifier [OPTIONS] <data> <labels>\n";
    cerr << "OPTIONS:\n"
	 << "  -c <type>         :: classifier type: LOGISTIC (default), BOOSTING\n"
         << "  -k <classes>      :: number of classes (default: auto)\n"
         << "  -n <n>            :: number of features (default: auto)\n"
	 << "  -m <n>            :: maximum iterations/boosting rounds (default: 100)\n"
         << "  -o <filename>     :: output model filename (default: none)\n"
         << "  -w <filename>     :: weights for each training instance\n"
         << SVL_STANDARD_OPTIONS_USAGE
         << "OPTIONS (LOGISTIC):\n"
	 << "  -r <n>            :: regularization (default: 1.0e-6)\n"
         << "OPTIONS (BOOSTING):\n"
	 << "  -splits <num>     :: number of splits in weak learners (default: 2)" << endl
	 << endl;
}

int main(int argc, char *argv[])
{
    svlLogger::setLogLevel(SVL_LOG_MESSAGE);

    // process commandline arguments
    const int NUM_REQUIRED_PARAMETERS = 2;

    // common options
    const char *classifierType = "LOGISTIC";
    int maxIterations = 1000;
    int nClasses = -1;
    int nFeatures = -1;
    const char *modelFilename = NULL;
    const char *weightsFilename = NULL;
    
    // logistic options
    double lambda = 1.0e-6;

    // boosting options
    int weakLearnerSplits = 2;

    if (argc <= 2) {
        usage();
        return -1;
    }

    char **args = argv + 1;
    while (--argc > NUM_REQUIRED_PARAMETERS) {
        SVL_PROCESS_STANDARD_OPTIONS(args, argc)
        else if (!strcmp(*args, "-c")) {
            classifierType = *(++args); argc--;
        } else if (!strcmp(*args, "-k")) {
            nClasses = atoi(*(++args)); argc--;
            assert(nClasses > 1);
        } else if (!strcmp(*args, "-n")) {
            nFeatures = atoi(*(++args)); argc--;
            assert(nFeatures > 1);
        } else if (!strcmp(*args, "-m")) {
	    maxIterations = atoi(*(++args)); argc--;
            assert(maxIterations > 1);
        } else if (!strcmp(*args, "-o")) {
            modelFilename = *(++args); argc--;
        } else if (!strcmp(*args, "-r")) {
	    lambda = atof(*(++args)); argc--;
        } else if (!strcmp(*args, "-w")) {
            weightsFilename = *(++args); argc--;
        } else if (!strcmp(*args, "-splits")) {
            weakLearnerSplits = atoi(*(++args)); argc--;
            assert(weakLearnerSplits > 0);
        } else {
	    SVL_LOG(SVL_LOG_ERROR, "unrecognized option " << *args);
	    usage();
	    return -1;
	}
        args++;
    }

    const char *DATAFILE = args[0];
    const char *LABELFILE = args[1];

    // load features
    SVL_LOG(SVL_LOG_VERBOSE, "Reading features from " << DATAFILE << "...");
    vector<vector<double> > features;
    ifstream ifs(DATAFILE);
    assert(!ifs.fail());
    while (1) {
        // determine number of features
        if (nFeatures < 0) {
            double fValue;
            nFeatures = 0;
            while (ifs.peek() != '\n') {
                ifs >> fValue;
                assert(!ifs.fail());
                nFeatures += 1;
            }
            ifs.seekg(0, ios::beg);
        }
        
        // read feature vector
        vector<double> v(nFeatures);
        for (int i = 0; i < nFeatures; i++) {
            ifs >> v[i];
        }
        if (ifs.fail()) break;
        features.push_back(v);
    }
    ifs.close();
    SVL_LOG(SVL_LOG_VERBOSE, "..." << features.size() 
        << " instances of size " << nFeatures << " read");

    // load training labels
    SVL_LOG(SVL_LOG_VERBOSE, "Reading training labels " << LABELFILE << "...");
    vector<int> labels(features.size(), -1);
    ifs.open(LABELFILE);
    assert(!ifs.fail());
    for (int i = 0; i < (int)labels.size(); i++) {
        ifs >> labels[i];
    }
    ifs.close();
    SVL_LOG(SVL_LOG_VERBOSE, "...done");
    
    // determine number of classes
    if (nClasses < 0) {
        nClasses = *max_element(labels.begin(), labels.end()) + 1;
        assert(nClasses > 1);
        SVL_LOG(SVL_LOG_VERBOSE, "...number of labels is " << nClasses);
    }

    // load training weights
    vector<double> weights(features.size(), 1.0);
    if (weightsFilename != NULL) {
        SVL_LOG(SVL_LOG_VERBOSE, "Reading training weights " << weightsFilename << "...");
        ifstream ifs(weightsFilename);
        assert(!ifs.fail());
        for (int i = 0; i < (int)weights.size(); i++) {
            ifs >> weights[i];
        }
        ifs.close();
        SVL_LOG(SVL_LOG_VERBOSE, "...done");
    }

    // train model
    SVL_LOG(SVL_LOG_VERBOSE, "Training model...");
    if (!strcasecmp(classifierType, "LOGISTIC")) {
        svlMultiClassLogistic classifier(nFeatures, nClasses, lambda);
        classifier.train(features, labels, weights, 1.0e-6, maxIterations);

        // write model
        if (modelFilename != NULL) {
            SVL_LOG(SVL_LOG_VERBOSE, "...writing parameters to " << modelFilename);
            classifier.save(modelFilename);
        }

    } else if (!strcasecmp(classifierType, "BOOSTING")) {

        // assemble design matrix for training
        CvMat *data = cvCreateMat((int)features.size(), nFeatures, CV_32FC1);
        CvMat *dataLabels = cvCreateMat((int)labels.size(), 1, CV_32SC1);
        assert((data != NULL) && (dataLabels != NULL));

        for (unsigned i = 0; i < features.size(); i++) {
	    for (int j = 0; j < nFeatures; j++) {
	        CV_MAT_ELEM(*data, float, i, j) = features[i][j];
	    }
            CV_MAT_ELEM(*dataLabels, int, i, 0) = (labels[i] > 0 ? 1 : -1);
        }

        // train the classifier
        CvBoostParams parameters(CvBoost::GENTLE, maxIterations, 0.9,
            weakLearnerSplits, false, NULL);
        CvMat *varType = cvCreateMat(nFeatures + 1, 1, CV_8UC1);
        for (int i = 0; i < nFeatures; i++) {
            CV_MAT_ELEM(*varType, unsigned char, i, 0) = CV_VAR_NUMERICAL;
        }
        CV_MAT_ELEM(*varType, unsigned char, nFeatures, 0) = CV_VAR_CATEGORICAL;
    
        CvBoost *classifier = new CvBoost(data, CV_ROW_SAMPLE, dataLabels,
            NULL, NULL, varType, NULL, parameters);
        cvReleaseMat(&varType);
        cvReleaseMat(&dataLabels);
        cvReleaseMat(&data);

        // write model
        if (modelFilename != NULL) {
            SVL_LOG(SVL_LOG_VERBOSE, "...writing parameters to " << modelFilename);
            classifier->save(modelFilename);
        }
        delete classifier;

    } else {
        SVL_LOG(SVL_LOG_FATAL, "unrecognized classifier type " << classifierType);
    }
    SVL_LOG(SVL_LOG_VERBOSE, "...done");

    // print profile information
    svlCodeProfiler::print(cerr);
    return 0;
}


