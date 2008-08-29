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
** FILENAME:    segImageFeatureExtract.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**  Code for extracting appearance features from image segments.
**
*****************************************************************************/

#include <cstdlib>
#include <cassert>
#include <iostream>
#include <fstream>
#include <iomanip>

#include "svlBase.h"
#include "svlVision.h"

using namespace std;

#define WINDOW_NAME "segImageExtractFeatures"

void usage()
{
    cerr << SVL_USAGE_HEADER << "\n";
    cerr << "USAGE: ./segImageExtractFeatures [OPTIONS] <imageDir>\n";
    cerr << "OPTIONS:\n"
         << "  -o <directory>         :: output directory (default: current)\n"
         << "  -imgExt <ext>          :: image file extension (default: .jpg)\n"
         << "  -segExt <ext>          :: over-segmentation file extension (default: .seg)\n"
         << "  -lblExt <ext>          :: pixel labels file extension (default: .txt)\n"
         << "  -segDir <dir>          :: over-segmentation file directory (default: <imageDir>)\n"
         << "  -lblDir <dir>          :: pixel labels file directory (default: <imageDir>)\n"
         << "  -featuresExt <ext>     :: output feature file extension (default: .features.txt)\n"
         << "  -segLblExt <ext>       :: segment labels file extension (default: .labels.txt)\n"
         << "  -noColor               :: exclude color features\n"
         << "  -noTexture             :: exclude texture features\n"
         << "  -noGeometry            :: exclude geometry features\n"
         << "  -noLocation            :: exclude image location features\n"
         << "  -includeBias           :: include bias feature (default: false)\n"
         << "  -skipLabels            :: skip extraction of labels\n"
         << "  -v                     :: verbose\n"
         << "  -x                     :: visualize feature extraction\n";
    cerr << "<imageDir> must contain images <base>.jpg and <regionsDir> must contain\n"
         << "corresponding over-segmentations <base>.seg.\n"
         << endl;
}

int main(int argc, char *argv[])
{
    svlCodeProfiler::enabled = true;
    svlCodeProfiler::tic(svlCodeProfiler::getHandle("main"));

    // read commandline parameters
    const int NUM_REQUIRED_PARAMETERS = 1;

    const char *imgDir = NULL;
    const char *segDir = NULL;
    const char *lblDir = NULL;
    const char *outputDir = ".";
    const char *imgExt = ".jpg";
    const char *segExt = ".seg";
    const char *lblExt = ".txt";
    const char *featuresExt = ".features.txt";
    const char *segLblExt = ".labels.txt";
    bool bIncludeBias = false;
    bool bSkipLabels = false;
    bool bVisualize = false;

    svlRegionFeatures featureCalculator;

    char **args = argv + 1;
    while (--argc > NUM_REQUIRED_PARAMETERS) {
        if (!strcmp(*args, "-o")) {
            outputDir = *(++args); argc -= 1;
        } else if (!strcmp(*args, "-imgExt")) {
            imgExt = *(++args); argc -= 1;
        } else if (!strcmp(*args, "-segExt")) {
            segExt = *(++args); argc -= 1;
        } else if (!strcmp(*args, "-lblExt")) {
            lblExt = *(++args); argc -= 1;
        } else if (!strcmp(*args, "-segDir")) {
            segDir = *(++args); argc -= 1;
        } else if (!strcmp(*args, "-lblDir")) {
            lblDir = *(++args); argc -= 1;
        } else if (!strcmp(*args, "-featuresExt")) {
            featuresExt = *(++args); argc -= 1;
        } else if (!strcmp(*args, "-segLblExt")) {
            segLblExt = *(++args); argc -= 1;
        } else if (!strcmp(*args, "-noColor")) {
            featureCalculator.setOption("noColor", true);
        } else if (!strcmp(*args, "-noTexture")) {
            featureCalculator.setOption("noTexture", true);
        } else if (!strcmp(*args, "-noGeometry")) {
            featureCalculator.setOption("noGeometry", true);
        } else if (!strcmp(*args, "-noLocation")) {
            featureCalculator.setOption("noLocation", true);
        } else if (!strcmp(*args, "-includeBias")) {
            bIncludeBias = true;
        } else if (!strcmp(*args, "-skipLabels")) {
            bSkipLabels = true;
        } else if (!strcmp(*args, "-v")) {
            svlLogger::setLogLevel(SVL_LOG_VERBOSE);
        } else if (!strcmp(*args, "-x")) {
            bVisualize = true;
        } else {
            SVL_LOG(SVL_LOG_FATAL, "unrecognized option " << *args);
        }
        args++;
    }

    if (argc != NUM_REQUIRED_PARAMETERS) {
        usage();
        return -1;
    }

    imgDir = args[0];
    if (segDir == NULL) segDir = imgDir;
    if (lblDir == NULL) lblDir = imgDir;

    if (bVisualize) {
        cvNamedWindow(WINDOW_NAME, 1);
    }

    // load images
    SVL_LOG(SVL_LOG_MESSAGE, "Reading images from " << imgDir << "...");
    vector<string> baseNames = svlDirectoryListing(imgDir, imgExt, false, false);
    SVL_LOG(SVL_LOG_MESSAGE, "...read " << baseNames.size() << " images");

    // extract features
    int hImageLoad = svlCodeProfiler::getHandle("imageLoad");
    int hComputeFeatures = svlCodeProfiler::getHandle("computeImageFeatures");
    int hFeatureSave = svlCodeProfiler::getHandle("featureSave");
    int hLabelSave = svlCodeProfiler::getHandle("labelSave");
    
    SVL_LOG(SVL_LOG_MESSAGE, "Processing images...");
    for (int i = 0; i < (int)baseNames.size(); i++) {
        // instantiate svlSegImage
        SVL_LOG(SVL_LOG_VERBOSE, "Loading image " << baseNames[i]);
        string imgName = string(imgDir) + string("/") + baseNames[i] + string(imgExt);
        string segName = string(segDir) + string("/") + baseNames[i] + string(segExt);
        string lblName = string(lblDir) + string("/") + baseNames[i] + string(lblExt);

        svlCodeProfiler::tic(hImageLoad);
        svlSegImage img(imgName.c_str(), segName.c_str(),
            bSkipLabels ? NULL : lblName.c_str());
        svlCodeProfiler::toc(hImageLoad);
        SVL_LOG(SVL_LOG_VERBOSE, "..." << toString(*img.image()));

        // show segmentation
        if (bVisualize) {
            IplImage *debugImage = img.visualize();
            img.colorBoundaries(debugImage, 0xff, 0xff, 0xff);
            cvShowImage(WINDOW_NAME, debugImage);
            cvWaitKey(50);
            cvReleaseImage(&debugImage);
        }
        
        // compute features
        SVL_LOG(SVL_LOG_VERBOSE, "...computing features");
        svlCodeProfiler::tic(hComputeFeatures);
        vector<vector<double> > x = featureCalculator.computeFeatures(img);
        svlCodeProfiler::toc(hComputeFeatures);
        SVL_LOG(SVL_LOG_VERBOSE, "..." << x.size() << " feature vectors computed");
        
        // save features
        svlCodeProfiler::tic(hFeatureSave);
        string outputName = string(outputDir) + string("/") + baseNames[i] + string(featuresExt);
        ofstream ofs(outputName.c_str());
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
        svlCodeProfiler::toc(hFeatureSave);

        // save labels
        if (!bSkipLabels) {
            svlCodeProfiler::tic(hLabelSave);
            string outputName = string(outputDir) + string("/") + baseNames[i] + string(segLblExt);
            ofstream ofs(outputName.c_str());
            assert(!ofs.fail());
            for (int i = 0; i < img.numSegments(); i++) {
                ofs << img.getLabel(i) << "\n";
            }
            ofs.close();
            svlCodeProfiler::toc(hLabelSave);
        }        
    }
    SVL_LOG(SVL_LOG_MESSAGE, "...done");
    
    // free memory
    cvDestroyAllWindows();
    svlCodeProfiler::toc(svlCodeProfiler::getHandle("main"));
    svlCodeProfiler::print(cerr);
    return 0;
}
