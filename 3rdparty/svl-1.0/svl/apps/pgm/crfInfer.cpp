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
** FILENAME:    crfInfer.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**  Application for inference in general or pairwise conditional Markov random
**  fields.
**
*****************************************************************************/

#include <cstdlib>
#include <cstring>
#include <cassert>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <algorithm>
#include <vector>

#include "svlBase.h"
#include "svlPGM.h"

using namespace std;

void usage()
{
    cerr << SVL_USAGE_HEADER << endl;
    cerr << "USAGE: ./crfInfer [OPTIONS] <model> <instance>\n";
    cerr << "OPTIONS:\n"
         << "  -o <filename>     :: output filename\n"
         << "  -t <type>         :: model type (GENERAL (default) or PAIRWISE)\n"
	 << "  -maxIters <n>     :: maximum number of messages\n"
         << SVL_STANDARD_OPTIONS_USAGE
	 << endl;
}

int main(int argc, char *argv[])
{
    svlLogger::setLogLevel(SVL_LOG_MESSAGE);

    // process commandline arguments
    const int NUM_REQUIRED_PARAMETERS = 2;

    const char *modelType = "GENERAL";
    int maxIterations = 100;
    const char *outputFilename = NULL;

    if (argc <= NUM_REQUIRED_PARAMETERS) {
        usage();
        return -1;
    }

    char **args = argv + 1;
    while (--argc > NUM_REQUIRED_PARAMETERS) {
        SVL_PROCESS_STANDARD_OPTIONS(args, argc)
        else if (!strcmp(*args, "-o")) {
            outputFilename = *(++args); argc--;
        } else if (!strcmp(*args, "-t")) {
            modelType = *(++args); argc--;
        } else if (!strcmp(*args, "-maxIters")) {
	    maxIterations = atoi(*(++args)); argc--;
        } else {
	    SVL_LOG(SVL_LOG_ERROR, "unrecognized option " << *args);
	    usage();
	    return -1;
	}
        args++;
    }

    const char *modelFilename = args[0];
    const char *instanceFilename = args[1];

    vector<vector<double> > marginals;

    if (!strcasecmp(modelType, "GENERAL")) {
        // load model
        SVL_LOG(SVL_LOG_VERBOSE, "Reading general model from " << modelFilename << "...");    
        svlGeneralCRFModel model;
        model.read(modelFilename);
        SVL_LOG(SVL_LOG_VERBOSE, "...done");
        
        // load instance
        SVL_LOG(SVL_LOG_VERBOSE, "Reading general instance from " << instanceFilename << "...");    
        svlGeneralCRFInstance instance;
        instance.read(instanceFilename);
        SVL_LOG(SVL_LOG_VERBOSE, "...done");

        // run inference
        SVL_LOG(SVL_LOG_VERBOSE, "Running inference...");
        model.inference(instance, marginals, maxIterations);
        SVL_LOG(SVL_LOG_VERBOSE, "...done");

    } else if (!strcasecmp(modelType, "PAIRWISE")) {
        // load model
        SVL_LOG(SVL_LOG_VERBOSE, "Reading pairwise model from " << modelFilename << "...");    
        svlPairwiseCRFModel model;
        model.read(modelFilename);
        SVL_LOG(SVL_LOG_VERBOSE, "...done");
        
        // load instance
        SVL_LOG(SVL_LOG_VERBOSE, "Reading pairwise instance from " << instanceFilename << "...");    
        svlPairwiseCRFInstance instance;
        instance.read(instanceFilename);
        SVL_LOG(SVL_LOG_VERBOSE, "...done");

        // run inference
        SVL_LOG(SVL_LOG_VERBOSE, "Running inference...");
        model.inference(instance, marginals, maxIterations);
        SVL_LOG(SVL_LOG_VERBOSE, "...done");

    } else {
        SVL_LOG(SVL_LOG_FATAL, "invalid model type \"" << modelType << "\"");
    }

    //mapAssignments = argmaxs(marginals);

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

    // print profile information
    svlCodeProfiler::print(cerr);
    return 0;
}


