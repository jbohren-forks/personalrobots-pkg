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
** FILENAME:    mexTrainClassifier.cpp
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

#include "svlBase.h"
#include "svlML.h"

#include "matlabUtils.h"

using namespace std;

void usage()
{
    //mexPrintf(SVL_USAGE_HEADER);
    mexPrintf("\n");
    mexPrintf("USAGE: model = mexTrainClassifier(data, labels, [weights, [options]]);\n");
    mexPrintf("OPTIONS:\n");
    mexPrintf("  options.maxIterations :: maximum training iterations (default = 1000)\n");
    mexPrintf("  options.nClasses      :: number of classes (default = auto)\n");
    mexPrintf("  options.lambda        :: regularization   (default = 1e-6)\n");
    mexPrintf("  options.verbose       :: verbose output (default: 0)\n");
    mexPrintf("\n");
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    if (nrhs == 0) {
        usage();
        return;
    }
    if ((nrhs != 2) && (nrhs != 3) && (nrhs != 4)) {
        usage();
        mexErrMsgTxt("incorrect number of input arguments");
    }
    if ((nrhs == 4) && (!mxIsStruct(prhs[3]))) { 
        mexErrMsgTxt("incorrect input argument type for OPTION");
    }
	
    // process commandline arguments
    const int NUM_REQUIRED_PARAMETERS = 2;

    // common options
    const char *classifierType = "LOGISTIC";
    const char *modelFilename = NULL;
    const char *weightsFilename = NULL;

    // set svlLogger callbacks
    svlLogger::showErrorCallback = mexErrMsgTxt;
    svlLogger::showWarningCallback = mexWarnMsgTxt;
    svlLogger::showMessageCallback = mexMsgTxt;

    // parse options
    map<string, string> options;
    options[string("maxIterations")] = string("1000");
    options[string("nClasses")] = string("-1");
    options[string("lambda")] = string("1.0e-6");
    options[string("verbose")] = string("0");
    if (nrhs == 4) {
        parseOptions(prhs[3], options);
    }

    int maxIterations = atoi(options[string("maxIterations")].c_str());
    int nClasses = atoi(options[string("nClasses")].c_str());
    int nFeatures = -1;

    if (atoi(options[string("maxIterations")].c_str()) != 0) {
        svlLogger::setLogLevel(SVL_LOG_VERBOSE);
    }

    // logistic options
    double lambda = atof(options[string("lambda")].c_str());

    // boosting options
    int weakLearnerSplits = 2;

    // error checking: check that rows(data) = rows(labels) and cols(labels) = 1
    if (mxGetM(prhs[0]) != mxGetM(prhs[1])){
        usage();
        mexErrMsgTxt("data and/or labels matrices are of the wrong size.");
    }

    // read features data from input mxArray	
    int numElementsData = mxGetNumberOfElements(prhs[0]);
    int numElementsLabels = mxGetNumberOfElements(prhs[1]);

    vector<vector<double> > features;
    int nInstances = mxGetM(prhs[0]); // i.e., number of rows
    if(nFeatures < 0) {
        nFeatures = mxGetN(prhs[0]);
    }

    mxArrayToVector(prhs[0], features);

    // read labels data from input mxArray
    vector<int> labels(nInstances, -1);
    if (nClasses < 0) {
        for (int k=0; k<nInstances; k++) {
            labels[k] = (int)mxGetPr(prhs[1])[k];
            if (labels[k] >= nClasses) nClasses = labels[k] + 1;
        }
    } else {
        for (int k=0; k<nInstances; k++) {
            labels[k] = (int)mxGetPr(prhs[1])[k];        
        }
    }

    // load training weights [OPTION]
    vector<double> weights(features.size(), 1.0);
    if (nrhs >= 3) {
        if (mxGetNumberOfElements(prhs[2]) != 0) {
            if (mxGetNumberOfElements(prhs[2])==features.size()) {
                for (int i=0; i<features.size(); i++) {
                    weights[i] = mxGetPr(prhs[2])[i];
                }
            } else  {
                mexErrMsgTxt("weights vector must be of the same size as the number of features");
            }
        } 
    }
	
    // train model
    SVL_LOG(SVL_LOG_VERBOSE, "Training model on " << features.size() << " features...");
    if (!strcasecmp(classifierType, "LOGISTIC")) {
        svlMultiClassLogistic classifier(nFeatures, nClasses, lambda);
        classifier.train(features, labels, weights, 1.0e-6, maxIterations);
        
        // write model
        if (modelFilename != NULL) {
            SVL_LOG(SVL_LOG_VERBOSE, "...writing parameters to " << modelFilename);
            classifier.save(modelFilename);   // do we have to output a matrix, or is this fine?
        }
        vector<vector<double> > parameters = classifier.getTheta(); 
        
        int rows = classifier.numClasses() - 1; //10 (test)
        int cols = classifier.numFeatures();  //5 (test)
        mxArray *output = mxCreateDoubleMatrix(rows, cols, mxREAL);
        double *outputPtr = mxGetPr(output);
        for (int n=0; n<cols; n++) {
            for (int m=0; m<rows; m++) {
                //mexPrintf("para[%d][%d] = %f\n", n,m,parameters[n][m]);				
                outputPtr[n*rows + m] = parameters[m][n]; 
            }
        }
        plhs[0] = output;
        
    } /* else if (!strcasecmp(classifierType, "BOOSTING")) {

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

    } */ else {
        SVL_LOG(SVL_LOG_FATAL, "unrecognized classifier type " << classifierType);
    }

    SVL_LOG(SVL_LOG_VERBOSE, "...done");

    // print profile information
    svlCodeProfiler::print(cerr); 
}
