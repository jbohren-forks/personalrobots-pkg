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
** FILENAME:    segImageTrainModel.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**  Training code for multi-class image segmentation.
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

#define WINDOW_NAME "segImageTrainModel"

typedef struct _TRegionDef {
    int id;
    std::string name;
    unsigned char red;
    unsigned char green;
    unsigned char blue;
} TRegionDef;

void usage()
{
    cerr << SVL_USAGE_HEADER << endl;
    cerr << "USAGE: ./segImageTrainModel [OPTIONS] <trainingList>" << endl;
    cerr << "OPTIONS:" << endl
         << "  -imgExt <ext>      :: features file extension (default: .jpg)\n"
         << "  -imgDir <dir>      :: features file directory (default: .)\n"
         << "  -segExt <ext>      :: over-segmentation file extension (default: .seg)\n"
         << "  -segDir <dir>      :: over-segmentation file directory (default: <imgDir>)\n"
         << "  -lblExt <ext>      :: pixel labels file extension (default: .txt)\n"
         << "  -lblDir <dir>      :: pixel labels file directory (default: <imgDir>)\n"
         << "  -featuresExt <ext> :: features file extension (default: .boosted.txt)\n"
         << "  -featuresDir <dir> :: features file directory (default: <imgDir>)\n"
	 << "  -o <filename>      :: output model file (default: none)\n"
	 << "  -model <type>      :: LOGISTIC or CRF (default)\n"
	 << "  -regNodes <lambda> :: regularization on nodes (default: 1.0e-9)\n"
	 << "  -regEdges <lambda> :: regularization on edges (default: 1.0e-3)\n"
	 << "  -regions <file>    :: region definition file\n"
	 << "  -v                 :: verbose (-v -v for debug)\n"
	 << "  -x                 :: visualize training\n"
	 << endl;
}

int main(int argc, char *argv[])
{
    svlCodeProfiler::enabled = true;
    svlCodeProfiler::tic(svlCodeProfiler::getHandle("main"));

    // read commandline parameters
    const int NUM_REQUIRED_PARAMETERS = 1;

    const char *imgExt = ".jpg";
    const char *imgDir = ".";
    const char *segExt = ".seg";
    const char *segDir = NULL;
    const char *lblExt = ".txt";
    const char *lblDir = NULL;
    const char *featuresExt = ".boosted.txt";
    const char *featuresDir = NULL;

    const char *outputFilename = NULL;
    const char *modelType = "CRF";
    double lambdaNode = 1.0e-9;
    double lambdaEdge = 1.0e-3;
    const char *regionsFilename = NULL;
    bool bVisualize = false;

    char **args = argv + 1;
    while (--argc > NUM_REQUIRED_PARAMETERS) {
        if (!strcmp(*args, "-o")) {
            outputFilename = *(++args); argc -= 1;
        } else if (!strcmp(*args, "-imgExt")) {
            imgExt = *(++args); argc -= 1;
        } else if (!strcmp(*args, "-imgDir")) {
            imgDir = *(++args); argc -= 1;
        } else if (!strcmp(*args, "-segExt")) {
            segExt = *(++args); argc -= 1;
        } else if (!strcmp(*args, "-segDir")) {
            segDir = *(++args); argc -= 1;
        } else if (!strcmp(*args, "-lblExt")) {
            lblExt = *(++args); argc -= 1;
        } else if (!strcmp(*args, "-lblDir")) {
            lblDir = *(++args); argc -= 1;
        } else if (!strcmp(*args, "-featuresExt")) {
            featuresExt = *(++args); argc -= 1;
        } else if (!strcmp(*args, "-featuresDir")) {
            featuresDir = *(++args); argc -= 1;
        } else if (!strcmp(*args, "-model")) {
            modelType = *(++args); argc -= 1;
	} else if (!strcmp(*args, "-regNodes")) {
	    lambdaNode = atof(*(++args)); argc--;
	} else if (!strcmp(*args, "-regEdges")) {
	    lambdaEdge = atof(*(++args)); argc--;
	} else if (!strcmp(*args, "-regions")) {
	    regionsFilename = *(++args); argc--;
	} else if (!strcmp(*args, "-v")) {
            if (svlLogger::getLogLevel() == SVL_LOG_VERBOSE)
                svlLogger::setLogLevel(SVL_LOG_VERBOSE);
            else svlLogger::setLogLevel(SVL_LOG_VERBOSE);
	} else if (!strcmp(*args, "-x")) {
	    bVisualize = true;
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

    const char *trainingList = args[0];
    if (segDir == NULL) segDir = imgDir;
    if (lblDir == NULL) lblDir = imgDir;
    if (featuresDir == NULL) featuresDir = imgDir;

    if (bVisualize) {
	cvNamedWindow(WINDOW_NAME, 1);
    }

    // load region definitions
    vector<TRegionDef> regionDefinitions;
    TRegionDef def;
    if (regionsFilename == NULL) {
	def.id = 0; def.name = "background";
	def.red = def.green = def.blue = 0x00;
	regionDefinitions.push_back(def);
	def.id = 1; def.name = "foreground";
	def.red = def.green = def.blue = 0xff;
	regionDefinitions.push_back(def);
    } else {	
	XMLNode root = XMLNode::parseFile(regionsFilename, "regionDefinitions");
	assert(!root.isEmpty());
	for (int i = 0; i < root.nChildNode("region"); i++) {
	    XMLNode node = root.getChildNode("region", i);
	    def.id = atoi(node.getAttribute("id"));
	    def.name = string(node.getAttribute("name"));
	    if (sscanf(node.getAttribute("color"), "%hhd %hhd %hhd", 
		    &def.red, &def.green, &def.blue) != 3) {
		SVL_LOG(SVL_LOG_FATAL, "could not parse color for \"" << def.name << "\" \"");
		return -1;
	    }
	    regionDefinitions.push_back(def);
	}
	SVL_LOG(SVL_LOG_VERBOSE, "... " << regionDefinitions.size() << " regions defined");
    }

    // load training images
    SVL_LOG(SVL_LOG_MESSAGE, "Reading training list from " << trainingList << "...");
    vector<string> baseNames;
    ifstream ifs(trainingList);
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
    
    int numLabels = -1;
    int nFeatures = -1;

    int hImageLoad = svlCodeProfiler::getHandle("imageLoad");
    int hFeaturesLoad = svlCodeProfiler::getHandle("featuresLoad");
    int hTrainLogistic = svlCodeProfiler::getHandle("trainLogistic");
    int hTrainCRF = svlCodeProfiler::getHandle("trainCRF");

    vector<svlPairwiseCRFInstance> instances;

    // process training instances
    SVL_LOG(SVL_LOG_MESSAGE, "Loading training images...");
    for (int i = 0; i < (int)baseNames.size(); i++) {
        // instantiate svlSegImage
        SVL_LOG(SVL_LOG_VERBOSE, "Loading image " << baseNames[i]);
        string imgName = string(imgDir) + string("/") + baseNames[i] + string(imgExt);
        string segName = string(segDir) + string("/") + baseNames[i] + string(segExt);
        string lblName = string(lblDir) + string("/") + baseNames[i] + string(lblExt);

        svlCodeProfiler::tic(hImageLoad);
        svlSegImage trainingImage(imgName.c_str(), segName.c_str(), lblName.c_str());
        svlCodeProfiler::toc(hImageLoad);
        SVL_LOG(SVL_LOG_VERBOSE, "..." << toString(*trainingImage.image()));

	// show segmentation
	if (bVisualize) {
	    IplImage *debugImage = trainingImage.visualize();
	    cvShowImage(WINDOW_NAME, debugImage);
	    if (cvWaitKey(-1) == 27) {
		bVisualize = false;
		cvReleaseImage(&debugImage);
		break;
	    }
            
	    cvZero(debugImage);
	    for (int j = 0; j < trainingImage.numSegments(); j++) {
		int indx = trainingImage.getLabel(j);
		if (indx < 0) continue;
		for (int k = 0; k < (int)regionDefinitions.size(); k++) {
		    if (regionDefinitions[k].id == indx) {
			trainingImage.colorSegment(debugImage, j, regionDefinitions[k].red,
			    regionDefinitions[k].green, regionDefinitions[k].blue);
			break;
		    }
		}
	    }
	    trainingImage.colorBoundaries(debugImage);
	    cvShowImage(WINDOW_NAME, debugImage);
	    if (cvWaitKey(-1) == 27) {
		bVisualize = false;
	    }

	    cvReleaseImage(&debugImage);
	}

	// load features and add to training instances
	instances.push_back(svlPairwiseCRFInstance());
	SVL_LOG(SVL_LOG_VERBOSE, "...computing features");
	svlCodeProfiler::tic(hFeaturesLoad);

        string featuresName = string(featuresDir) + string("/") + baseNames[i] + string(featuresExt);
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
        
            // read feature vectors
            vector<double> v(nFeatures);
            for (int j = 0; j < nFeatures; j++) {
                ifsData >> v[j];
            }
            if (ifsData.fail()) break;
            
            instances.back().Xn.push_back(v);
        }
        ifs.close();
	assert((int)instances.back().Xn.size() == trainingImage.numSegments());
	svlCodeProfiler::toc(hFeaturesLoad);
       
	SVL_LOG(SVL_LOG_VERBOSE, "...assigning groundtruth labels");
	instances.back().Yn.resize(trainingImage.numSegments(), -1);
	for (int j = 0; j < trainingImage.numSegments(); j++) {
	    instances.back().Yn[j] = trainingImage.getLabel(j);
	    if (instances.back().Yn[j] >= numLabels)
		numLabels = instances.back().Yn[j] + 1;
	}
	
	SVL_LOG(SVL_LOG_VERBOSE, "...computing graph structure");
	instances.back().edges = trainingImage.getAdjacencyList();
	instances.back().Xnm.resize(instances.back().numEdges());
	for (int j = 0; j < (int)instances.back().Xnm.size(); j++) {
	    instances.back().Xnm[j].resize(1, 1.0);
	}

#if 1
        // add location to edge potentials
        for (int j = 0; j < (int)instances.back().edges.size(); j++) {
            pair<int, int> edge = instances.back().edges[j];
            double y = (double)(trainingImage.getCentroid(edge.first).y +
                trainingImage.getCentroid(edge.second).y) / (2.0 * trainingImage.height());
            double x = (double)(trainingImage.getCentroid(edge.first).x +
                trainingImage.getCentroid(edge.second).x) / (2.0 * trainingImage.width());
            instances.back().Xnm[j].push_back(y);
            instances.back().Xnm[j].push_back(x);
        }
#endif

        // assign weight to instance
        instances.back().weight = 1.0;
    }
    SVL_LOG(SVL_LOG_VERBOSE, "...done");

    SVL_LOG(SVL_LOG_MESSAGE, instances.size() << " training images; " 
        << nFeatures << " singleton features; " << numLabels << " labels");
        
#if 0
    // whiten features
    vector<double> mu(numSingletonFeatures, 0.0);
    vector<double> sigma(numSingletonFeatures, 0.0);
    double total = 0.0;
    for (int t = 0; t < (int)instances.size(); t++) {
        for (int n = 0; n < (int)instances[t].Xn.size(); n++) {
            for (int i = 0; i < numSingletonFeatures; i++) {
                mu[i] += instances[t].Xn[n][i];
                sigma[i] += instances[t].Xn[n][i] * instances[t].Xn[n][i];
            }
        }
        total += (double)instances[t].Xn.size();
    }

    for (int i = 0; i < numSingletonFeatures; i++) {
        mu[i] /= total;
        if (isnan(mu[i])) {
            cerr << "ERROR: feature " << i << " has NaN values" << endl;
        }
        assert(!isnan(mu[i]));
        sigma[i] = sigma[i] / total - mu[i] * mu[i];
        if (sigma[i] < 0.0)
            sigma[i] = 0.0;
        else sigma[i] = sqrt(sigma[i]);
    }

    if (modelFilestem != NULL) {
        string filename = string(modelFilestem) + string(".normalization.txt");
        ofstream ofs(filename.c_str());
        assert(!ofs.fail());
        for (int i = 0; i < numSingletonFeatures; i++) {
            ofs << mu[i] << " " << sigma[i] << "\n";
        }
        ofs.close();
    }

    for (int t = 0; t < (int)instances.size(); t++) {
        for (int n = 0; n < (int)instances[t].Xn.size(); n++) {
            for (int i = 0; i < numSingletonFeatures; i++) {
                if (sigma[i] == 0.0) continue;
                instances[t].Xn[n][i] = (instances[t].Xn[n][i] - mu[i]) / sigma[i];
            }
        }
    }    
#endif    

    // train model
    if (!strcasecmp(modelType, "LOGISTIC")) {
	// populate training data
        vector<int> actualLabels;
	vector<vector<double> > singletonFeatures;
	for (int t = 0; t < (int)instances.size(); t++) {
	    singletonFeatures.reserve(singletonFeatures.size() + instances[t].Xn.size());
	    actualLabels.reserve(actualLabels.size() + instances[t].Xn.size());
	    for (int i = 0; i < (int)instances[t].Xn.size(); i++) {
		if (instances[t].Yn[i] < 0)
		    continue;
		singletonFeatures.push_back(instances[t].Xn[i]);
		actualLabels.push_back(instances[t].Yn[i]);
	    }
	    assert(actualLabels.size() == singletonFeatures.size());
	}

	svlMultiClassLogistic logisticModel(nFeatures, numLabels, lambdaNode);
	svlCodeProfiler::tic(hTrainLogistic);
	logisticModel.train(singletonFeatures, actualLabels, 1.0e-3, 1000);
	svlCodeProfiler::toc(hTrainLogistic);

	if (outputFilename != NULL) {
	    SVL_LOG(SVL_LOG_VERBOSE, "...saving to " << outputFilename);
	    logisticModel.save(outputFilename);
	}

    } else if (!strcasecmp(modelType, "CRF")) {
	svlPairwiseCRFModel crfModel(numLabels, nFeatures, instances[0].Xnm[0].size());
        
	svlCodeProfiler::tic(hTrainCRF);
	crfModel.learn(instances, 1000, lambdaNode, lambdaEdge);
	svlCodeProfiler::toc(hTrainCRF);

	if (outputFilename != NULL) {
	    SVL_LOG(SVL_LOG_VERBOSE, "...saving to " << outputFilename);
	    crfModel.write(outputFilename);
	}

    } else {
        SVL_LOG(SVL_LOG_FATAL, "unrecognized model type");
	return -1;
    }

    // free memory
    cvDestroyAllWindows();
    svlCodeProfiler::toc(svlCodeProfiler::getHandle("main"));
    svlCodeProfiler::print(cerr);
    return 0;
}

