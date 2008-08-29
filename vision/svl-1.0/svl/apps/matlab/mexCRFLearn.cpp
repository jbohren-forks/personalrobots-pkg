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
** FILENAME:    mexCRFLearn.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**  CRF learning code for Matlab.
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
    mexPrintf("USAGE: model = mexCRFLearn(model, instances, [options]);\n");
    mexPrintf("OPTIONS:\n");
    mexPrintf("  options.maxIterations :: maximum training iterations\n");
    mexPrintf("  options.lambda        :: regularization (single or one per weight)\n");
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
    options[string("maxIterations")] = string("1000");
    options[string("lambda")] = string("1.0e-9");
    options[string("debug")] = string("0");
    if (nrhs == 3) {
        parseOptions(prhs[2], options);
    }
    bool bDebug = (atoi(options[string("debug")].c_str()) != 0);
    if (bDebug) svlLogger::setLogLevel(SVL_LOG_DEBUG);

    // parse model and instances
    svlGeneralCRFModel model = parseCRFModel(prhs[0]);
    if (bDebug) {
        model.write(cout);
        cout.flush();
    }
    vector<svlGeneralCRFInstance> instances;
    instances.resize(mxGetNumberOfElements(prhs[1]));
    for (int i = 0; i < (int)instances.size(); i++) {
        instances[i] = parseCRFInstance(prhs[1], i);
#if 0
        if (bDebug) {
            instances[i].write(cout);
            cout.flush();
        }
#endif
    }
    int maxIterations = atoi(options[string("maxIterations")].c_str());
    vector<double> lambda;
    parseString<double>(options[string("lambda")], lambda);

    // run learning
    if (lambda.size() == 1) {
        lambda.resize(model.numWeights(), lambda[0]);
    }
    SVL_ASSERT(lambda.size() == (unsigned)model.numWeights());
    model.learn(instances, maxIterations, lambda);

    // copy model weights to output
    plhs[0] = mxDuplicateArray(prhs[0]);

    mxArray *weights = mxGetField(plhs[0], 0, "weights");
    for (int i = 0; i < model.numWeights(); i++) {
        mxGetPr(weights)[i] = model[i];
    }
}

