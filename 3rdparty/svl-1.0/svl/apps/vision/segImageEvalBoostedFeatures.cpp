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
** FILENAME:    segImageEvalBoostedFeatures.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**  Evaluation code for multi-class image segmentation features.
**
*****************************************************************************/

#include <cstdlib>
#include <cassert>
#include <iostream>
#include <fstream>
#include <iomanip>

#include <cxcore.h>
#include <cv.h>
#include <ml.h>

#include "svlBase.h"
#include "svlML.h"
#include "svlPGM.h"
#include "svlVision.h"

using namespace std;

void usage()
{
    cerr << SVL_USAGE_HEADER << endl;
    cerr << "USAGE: ./segImageEvalBoostedFeatures [OPTIONS] <numClasses> <evalList>\n";
    cerr << "OPTIONS:\n"
         << "  -featuresExt <ext> :: features file extension (default: .features.txt)\n"
         << "  -featuresDir <dir> :: features file directory (default: .)\n"
         << "  -i <filestem>      :: input model files (default: ./segmentation)\n"
         << "  -modelExt <ext>    :: input model extension (default: .model)\n"
         << "  -outputDir <dir>   :: output directory (default: .)\n"
         << "  -outputExt <dir>   :: output features extension (default: .boosted.txt)\n"
         << "  -includeBias       :: include bias feature (default: false)\n"
         << "  -v                 :: verbose (-v -v for debug)\n"
         << endl;
}

int main(int argc, char *argv[])
{
    svlCodeProfiler::enabled = true;
    svlCodeProfiler::tic(svlCodeProfiler::getHandle("main"));

    // read commandline parameters
    const int NUM_REQUIRED_PARAMETERS = 2;

    const char *featuresExt = ".features.txt";
    const char *featuresDir = ".";
    const char *inputModelStem = NULL;
    const char *modelExt = ".model";
    bool bIncludeBias = false;

    const char *outputDir = ".";
    const char *outputExt = ".boosted.txt";

    char **args = argv + 1;
    while (--argc > NUM_REQUIRED_PARAMETERS) {
        if (!strcmp(*args, "-featuresExt")) {
            featuresExt = *(++args); argc--;
        } else if (!strcmp(*args, "-featuresDir")) {
            featuresDir = *(++args); argc--;
        } else if (!strcmp(*args, "-i")) {
            inputModelStem = *(++args); argc--;
        } else if (!strcmp(*args, "-modelExt")) {
            modelExt = *(++args); argc--;
        } else if (!strcmp(*args, "-outputDir")) {
            outputDir = *(++args); argc--;
        } else if (!strcmp(*args, "-outputExt")) {
            outputExt = *(++args); argc--;
        } else if (!strcmp(*args, "-includeBias")) {
            bIncludeBias = true;
        } else if (!strcmp(*args, "-v")) {
            if (svlLogger::getLogLevel() == SVL_LOG_VERBOSE)
                svlLogger::setLogLevel(SVL_LOG_VERBOSE);
            else svlLogger::setLogLevel(SVL_LOG_VERBOSE);
        } else {
            SVL_LOG(SVL_LOG_FATAL, "unrecognized option " << *args);
            return -1;
        }
        args++;
    }

    if (argc != NUM_REQUIRED_PARAMETERS) {
        usage();
        return -1;
    }

    int numLabels = atoi(args[0]);
    assert(numLabels > 0);
    const char *evalList = args[1];

    // load images
    SVL_LOG(SVL_LOG_MESSAGE, "Reading evaluation list from " << evalList << "...");
    vector<string> baseNames;
    ifstream ifs(evalList);
    assert(!ifs.fail());
    while (!ifs.eof()) {
        string name;
        ifs >> name;
        if (ifs.fail()) break;
        if (name.empty()) continue;
        baseNames.push_back(name);
    }
    ifs.close();
    SVL_LOG(SVL_LOG_MESSAGE, "...read " << baseNames.size() << " images");

    int nFeatures = -1;

    // load boosted detectors
    SVL_LOG(SVL_LOG_MESSAGE, "Reading boosted models...");

    vector<CvBoost *> models(numLabels);
    for (int i = 0; i < numLabels; i++) {
        string modelFilename = string(inputModelStem) + string(".class.") +
            toString(i) + string(modelExt);
        SVL_LOG(SVL_LOG_VERBOSE, "...loading from " << modelFilename);
        models[i] = new CvBoost();
        assert(models[i] != NULL);
        models[i]->load(modelFilename.c_str());
    }
    SVL_LOG(SVL_LOG_MESSAGE, "...done");

    SVL_LOG(SVL_LOG_MESSAGE, "Processing image features...");
    for (int i = 0; i < (int)baseNames.size(); i++) {
        string featuresName = string(featuresDir) + string("/") + baseNames[i] + string(featuresExt);
        SVL_LOG(SVL_LOG_VERBOSE, baseNames[i]);

        vector<vector<double> > x;

        ifstream ifsData(featuresName.c_str());
        assert(!ifsData.fail());
            
        while (1) {
            // determine number of features
            if (nFeatures < 0) {
                double fValue;
                nFeatures = 0;
                while (ifsData.peek() != '\n') {
                    ifsData >> fValue;
                    assert(!ifsData.fail());
                    nFeatures += 1;
                }
                ifsData.seekg(0, ios::beg);
            }
        
            // read feature vector
            CvMat *fv = cvCreateMat(1, nFeatures, CV_32FC1);
            for (int j = 0; j < nFeatures; j++) {
                ifsData >> CV_MAT_ELEM(*fv, float, 0, j);
            }
            if (ifsData.fail()) {
                cvReleaseMat(&fv);
                break;
            }
            
            x.push_back(vector<double>(numLabels));
            for (int k = 0; k < numLabels; k++) {
                CvMat *responses = cvCreateMat(models[k]->get_params().weak_count, 1, CV_32FC1);
                models[k]->predict(fv, NULL, responses);
                double score = cvSum(responses).val[0];
                assert(isfinite(score));		
                x.back()[k] = score;
                cvReleaseMat(&responses);
            }
            cvReleaseMat(&fv);
        }
        ifsData.close();
        
        // write boosted feature vector file
        string outputFilename = string(outputDir) + string("/") + baseNames[i] + string(outputExt);
        ofstream ofs(outputFilename.c_str());
        assert(!ofs.fail());
        for (unsigned j = 0; j < x.size(); j++) {
            if (bIncludeBias) ofs << "1 ";
            for (unsigned k = 0; k < x[j].size(); k++) {
                if (k != 0) ofs << " ";
                ofs << x[j][k];
            }
            ofs << "\n";
        }
        ofs.close();            
    }
    SVL_LOG(SVL_LOG_VERBOSE, "...done");
    
    // free memory
    for (int i = 0; i < (int)models.size(); i++) {
        delete models[i];
    }

    // print profile information
    svlCodeProfiler::toc(svlCodeProfiler::getHandle("main"));
    svlCodeProfiler::print(cerr);
    return 0;
}

