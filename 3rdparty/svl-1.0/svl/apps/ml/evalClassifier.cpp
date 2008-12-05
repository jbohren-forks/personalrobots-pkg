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
** FILENAME:    evalClassifier.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**  Application for evaluating a logistic or boosted classifier.
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
#include "ml.h"

#include "svlBase.h"
#include "svlML.h"

using namespace std;

void usage()
{
    cerr << SVL_USAGE_HEADER << endl;
    cerr << "USAGE: ./evalClassifier [OPTIONS] <parameters> <data>\n";
    cerr << "OPTIONS:\n"
	 << "  -c <type>         :: classifier type: LOGISTIC (default), BOOSTING\n"
         << "  -l <labels>       :: groundtruth label file for confusion matrix\n"
         << "  -n <n>            :: number of features (default: auto)\n"
	 << "  -o <filename>     :: output scores/marginal (default: STDOUT (with -verbose))" << endl
         << SVL_STANDARD_OPTIONS_USAGE
	 << endl;
}

int main(int argc, char *argv[])
{
    svlLogger::setLogLevel(SVL_LOG_MESSAGE);

    // process commandline arguments
    const int NUM_REQUIRED_PARAMETERS = 2;

    // common options
    const char *classifierType = "LOGISTIC";
    const char *outputFilename = NULL;
    const char *labelsFilename = NULL;
    int nFeatures = -1;
    
    if (argc <= 2) {
        usage();
        return -1;
    }

    char **args = argv + 1;
    while (--argc > NUM_REQUIRED_PARAMETERS) {
        SVL_PROCESS_STANDARD_OPTIONS(args, argc)
        else if (!strcmp(*args, "-c")) {
            classifierType = *(++args); argc--;
        } else if (!strcmp(*args, "-l")) {
            labelsFilename = *(++args); argc--;
        } else if (!strcmp(*args, "-n")) {
            nFeatures = atoi(*(++args)); argc--;
            assert(nFeatures > 1);
        } else if (!strcmp(*args, "-o")) {
            outputFilename = *(++args); argc--;
	} else {
	    SVL_LOG(SVL_LOG_ERROR, "unrecognized option " << *args);
	    usage();
	    return -1;
	}
        args++;
    }

    const char *PARAMFILE = args[0];
    const char *DATAFILE = args[1];

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
    SVL_LOG(SVL_LOG_VERBOSE, "..." << features.size() << " of size " 
        << nFeatures << " instances read");
    
    // evaluate model
    vector<int> predictedLabels(features.size(), -1);
    int nPredicted = 2;
    SVL_LOG(SVL_LOG_VERBOSE, "Evaluating model...");
    if (!strcasecmp(classifierType, "LOGISTIC")) {
        svlMultiClassLogistic classifier;
        if (!classifier.load(PARAMFILE)) {
            SVL_LOG(SVL_LOG_FATAL, "could not load parameters from " << PARAMFILE);
        }

        vector<vector<double> > marginals;
        classifier.evaluateMarginal(features, marginals);
        predictedLabels = argmaxs(marginals);
        nPredicted = classifier.numClasses();

        // write output
        if (outputFilename != NULL) {
            ofstream ofs(outputFilename);
            assert(!ofs.fail());
            for (unsigned i = 0; i < marginals.size(); i++) {
                ofs << toString(marginals[i]) << "\n";
            }
            ofs.close();
        } else {
            for (unsigned i = 0; i < marginals.size(); i++) {
                SVL_LOG(SVL_LOG_VERBOSE, toString(marginals[i]));
            }
        }

    } else if (!strcasecmp(classifierType, "BOOSTING")) {

        SVL_LOG(SVL_LOG_VERBOSE, "Loading model parameters...");
        CvBoost *classifier = new CvBoost();
        assert(classifier != NULL);
        classifier->load(PARAMFILE);
        SVL_LOG(SVL_LOG_VERBOSE, "...done");

        ofstream *ofs = NULL;
        if (outputFilename != NULL) {
            ofs = new ofstream(outputFilename);
            assert((ofs != NULL) && (!ofs->fail()));
        }

        CvMat *featureVector = cvCreateMat(1, nFeatures, CV_32FC1);
        CvMat *weakResponses = cvCreateMat(classifier->get_params().weak_count, 1, CV_32FC1);
        for (unsigned i = 0; i < features.size(); i++) {
            for (int j = 0; j < nFeatures; j++) {
                CV_MAT_ELEM(*featureVector, float, 0, j) = (float)features[i][j];
            }

            int classLabel = (int)classifier->predict(featureVector, NULL, weakResponses);
            predictedLabels[i] = classLabel < 0 ? 0 : 1;
            double score = cvSum(weakResponses).val[0];
            if (ofs != NULL) {
                *ofs << score << "\n";
            } else {
                SVL_LOG(SVL_LOG_VERBOSE, score);
            }
        }

        // close output file
        if (ofs != NULL) {
            ofs->close();
            delete ofs;
        }

        // release memory
        cvReleaseMat(&weakResponses);
        cvReleaseMat(&featureVector);
        delete classifier;

    } else {
        SVL_LOG(SVL_LOG_FATAL, "unrecognized classifier type " << classifierType);
    }
    SVL_LOG(SVL_LOG_VERBOSE, "...done");

    // load groundtruth labels (for scoring)
    if (labelsFilename) {
        SVL_LOG(SVL_LOG_VERBOSE, "Reading groundtruth labels " << labelsFilename << "...");
        vector<int> actualLabels(features.size(), -1);
        ifs.open(labelsFilename);
        assert(!ifs.fail());
        for (int i = 0; i < (int)actualLabels.size(); i++) {
            ifs >> actualLabels[i];
        }
        ifs.close();
        SVL_LOG(SVL_LOG_VERBOSE, "...done");

        assert(predictedLabels.size() == actualLabels.size());
        int nActual = *max_element(actualLabels.begin(), actualLabels.end()) + 1;

        // build confusion matrix
        svlConfusionMatrix confusionMatrix(nActual, nPredicted);
        confusionMatrix.accumulate(actualLabels, predictedLabels);
        
        confusionMatrix.printRowNormalized();
        SVL_LOG(SVL_LOG_MESSAGE, "Classifier accuracy: " 
            << confusionMatrix.accuracy());
    }


    // print profile information
    svlCodeProfiler::print(cerr);
    return 0;
}


