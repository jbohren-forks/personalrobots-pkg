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
** FILENAME:    mexEvalClassifier.cpp
** AUTHOR(S):   Ian Quek <ianquek@stanford.edu>
**              Stephen Gould <sgould@stanford.edu>
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

#include "cv.h"
#include "cxcore.h"
#include "ml.h"

// Matlab
#include "mex.h"
#include "matrix.h"
#include "matlabUtils.h"

#include "svlBase.h"
#include "svlML.h"

using namespace std;

void usage()
{
    mexPrintf(SVL_USAGE_HEADER);
    mexPrintf("\n");
    mexPrintf("USAGE: marginals = mexEvalClassifier(parameters, data);\n");
    mexPrintf("OPTIONS:\n");
    mexPrintf("  options.verbose       :: verbose output (default: 0)\n");
    mexPrintf("\n");
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    if (nrhs == 0) {
        usage();
        return;
    }
    if ((nrhs != 2) && (nrhs != 3)) {
        usage();
        mexErrMsgTxt("incorrect number of input arguments");
    }
    if ((!mxIsNumeric(prhs[0])) || (!mxIsNumeric(prhs[1]))) { 
        mexErrMsgTxt("incorrect input argument types");
    }

    svlLogger::setLogLevel(SVL_LOG_MESSAGE);

    // process commandline arguments
    const int NUM_REQUIRED_PARAMETERS = 2;

    // common options
    const char *classifierType = "LOGISTIC";
    const char *outputFilename = NULL;
    const char *labelsFilename = NULL;
    int nFeatures = -1;

    // set svlLogger callbacks
    svlLogger::showErrorCallback = mexErrMsgTxt;
    svlLogger::showWarningCallback = mexWarnMsgTxt;
    svlLogger::showMessageCallback = mexMsgTxt;

    // parse options
    map<string, string> options;
    options[string("verbose")] = string("0");
    if (nrhs == 3) {
        parseOptions(prhs[2], options);
    }

    if (atoi(options[string("maxIterations")].c_str()) != 0) {
        svlLogger::setLogLevel(SVL_LOG_VERBOSE);
    }

    // read features data from input mxArray
    vector<vector<double> > features;
    mxArrayToVector(prhs[1], features);

    nFeatures = mxGetM(prhs[1]);  // i.e., number of rows
    SVL_LOG(SVL_LOG_VERBOSE, "..." << features.size() << " of size " 
        << nFeatures << " instances read");
    
    // evaluate model
    vector<int> predictedLabels(features.size(), -1);
    int nPredicted = 2;
    SVL_LOG(SVL_LOG_VERBOSE, "Evaluating model...");
    if (!strcasecmp(classifierType, "LOGISTIC")) {
        svlMultiClassLogistic classifier;

        vector<vector<double> > param;
        mxArrayToVector(prhs[0], param);
        classifier.setTheta(param);

        vector<vector<double> > marginals;
        classifier.evaluateMarginal(features, marginals);
        predictedLabels = argmaxs(marginals);
        nPredicted = classifier.numClasses();

        // write output
        int rows = marginals.size();
        int cols = marginals[0].size();
        mxArray *output = mxCreateDoubleMatrix(rows, cols, mxREAL);
        double *outputPtr = mxGetPr(output);
        for (int n=0; n<cols; n++) {
            for (int m=0; m<rows; m++) {
                outputPtr[n*rows + m] = marginals[m][n]; 
            }
        }
        plhs[0] = output;

    } /* else if (!strcasecmp(classifierType, "BOOSTING")) {

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

    } */ else {
        SVL_LOG(SVL_LOG_FATAL, "unrecognized classifier type " << classifierType);
    }
    SVL_LOG(SVL_LOG_VERBOSE, "...done");


    // print profile information
    svlCodeProfiler::print(cerr);
}
