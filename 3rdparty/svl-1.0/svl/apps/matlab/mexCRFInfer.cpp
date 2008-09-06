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
** FILENAME:    mexCRFInfer.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**  CRF inference code for Matlab.
**
*****************************************************************************/

#include <cstdlib>
#include <cassert>
#include <iostream>
#include <fstream>
#include <iomanip>

// Matlab
#include "mex.h"
#include "matrix.h"

// xml
#include "xmlParser/xmlParser.h"

// STAIR Vision Library
#include "svlBase.h"
#include "svlPGM.h"

#include "matlabUtils.h"

using namespace std;

void usage()
{
    mexPrintf(SVL_USAGE_HEADER);
    mexPrintf("\n");
    mexPrintf("USAGE: output = mexCRFInfer(model, instance, [options]);\n");
    mexPrintf("OPTIONS:\n");
    mexPrintf("  options.maxIterations :: maximum number of messages\n");
    mexPrintf("  options.debug         :: turn on debugging\n");
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

    if ((!mxIsStruct(prhs[0])) || (!mxIsStruct(prhs[1]))) {
        mexErrMsgTxt("incorrect input argument types");
    }

    if (nlhs != 1) {
        usage();
        mexErrMsgTxt("incorrect number of output arguments");
    }

    // set svlLogger callbacks
    svlLogger::showErrorCallback = mexErrMsgTxt;
    svlLogger::showWarningCallback = mexWarnMsgTxt;
    svlLogger::showMessageCallback = mexMsgTxt;

    // set default options and parse
    map<string, string> options;
    options[string("maxIterations")] = string("100");
    options[string("debug")] = string("0");
    if (nrhs == 3) {
        parseOptions(prhs[2], options);
    }
    bool bDebug = (atoi(options[string("debug")].c_str()) != 0);
    if (bDebug) svlLogger::setLogLevel(SVL_LOG_DEBUG);

    // parse model and instance
    svlGeneralCRFModel model = parseCRFModel(prhs[0]);
    svlGeneralCRFInstance instance = parseCRFInstance(prhs[1]);
    int maxIterations = atoi(options[string("maxIterations")].c_str());

    // run inference
    vector<vector<double> > marginals;
    model.inference(instance, marginals, maxIterations);

    // copy marginals to output
    const char *outputFields[2] = {"marginal", "value"};
    int numVariables = instance.numVariables();
    plhs[0] = mxCreateStructArray(1, &numVariables, 2, outputFields);
    
    for (int i = 0; i < numVariables; i++) {
        mxArray *outputMarginal = mxCreateDoubleMatrix(1, marginals[i].size(), mxREAL);
        mxArray *outputValue = mxCreateDoubleMatrix(1, 1, mxREAL);
        double *outputMarginalPtr = mxGetPr(outputMarginal);
        int mapClass = 0;
        for (int j = 0; j < (int)marginals[i].size(); j++) {
            outputMarginalPtr[j] = marginals[i][j];
            if (marginals[i][j] > marginals[i][mapClass])
                mapClass = j;
        }        
        mxGetPr(outputValue)[0] = (double)mapClass;
        mxSetField(plhs[0], i, "marginal", outputMarginal);
        mxSetField(plhs[0], i, "value", outputValue);
    }
}

