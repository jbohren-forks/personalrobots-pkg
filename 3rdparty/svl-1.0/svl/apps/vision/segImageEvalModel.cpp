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
** FILENAME:    segImageEvalModel.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**  Evaluation code for multi-class image segmentation.
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

#define WINDOW_NAME "segImageEvalModel"

typedef struct _TRegionDef {
    int id;
    std::string name;
    unsigned char red;
    unsigned char green;
    unsigned char blue;
} TRegionDef;

// function prototypes -----------------------------------------------------

double scoreImagePixelwise(svlSegImage* testImage, 
    const vector<int>& predictedLabels,
    svlConfusionMatrix& confusion);

double scoreImageSegmentwise(const vector<int>& actualLabels,
    const vector<int>& predictedLabels,
    svlConfusionMatrix& confusion);

// main --------------------------------------------------------------------

void usage()
{
    cerr << SVL_USAGE_HEADER << endl;
    cerr << "USAGE: ./segImageEvalModel [OPTIONS] <modelFile> <evaluationList>" << endl;
    cerr << "OPTIONS:" << endl
         << "  -imgExt <ext>      :: features file extension (default: .jpg)\n"
         << "  -imgDir <dir>      :: features file directory (default: .)\n"
         << "  -segExt <ext>      :: over-segmentation file extension (default: .seg)\n"
         << "  -segDir <dir>      :: over-segmentation file directory (default: <imgDir>)\n"
         << "  -lblExt <ext>      :: pixel labels file extension (default: .txt)\n"
         << "  -lblDir <dir>      :: pixel labels file directory (default: <imgDir>)\n"
         << "  -featuresExt <ext> :: features file extension (default: .boosted.txt)\n"
         << "  -featuresDir <dir> :: features file directory (default: <imgDir>)\n"
	 << "  -model <type>      :: LOGISTIC or CRF (default)\n"
	 << "  -regions <file>    :: region definition file\n"
	 << "  -outputDir <dir>   :: output directory for images/marginals\n"
	 << "  -outputImages      :: output images\n"
         << "  -outputScores      :: output image scores\n"
	 << "  -outputMarginals   :: output marginals\n"
	 << "  -outputConfusion   :: output confusion matrix (<directory>/confusion.txt)\n"
         << "  -segwiseScore      :: produces scores on segments rather than pixels\n"
	 << "  -v                 :: verbose (-v -v for debug)\n"
	 << "  -x                 :: visualize evaluation\n"
	 << endl;
}

int main(int argc, char *argv[])
{
    svlCodeProfiler::enabled = true;
    svlCodeProfiler::tic(svlCodeProfiler::getHandle("main"));

    // read commandline parameters
    const int NUM_REQUIRED_PARAMETERS = 2;

    const char *imgExt = ".jpg";
    const char *imgDir = ".";
    const char *segExt = ".seg";
    const char *segDir = NULL;
    const char *lblExt = ".txt";
    const char *lblDir = NULL;
    const char *featuresExt = ".boosted.txt";
    const char *featuresDir = NULL;

    const char *modelType = "CRF";
    const char *regionsFilename = NULL;

    const char *outputDir = NULL;
    bool bOutputImages = false;
    bool bOutputScores = false;
    bool bOutputMarginals = false;
    bool bOutputConfusion = false;
    bool bPixelwiseScore = true;
    bool bVisualize = false;

    char **args = argv + 1;
    while (--argc > NUM_REQUIRED_PARAMETERS) {
        if (!strcmp(*args, "-imgExt")) {
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
	} else if (!strcmp(*args, "-regions")) {
	    regionsFilename = *(++args); argc--;
	} else if (!strcmp(*args, "-outputDir")) {
	    outputDir = *(++args); argc--;
	} else if (!strcmp(*args, "-outputImages")) {
            bOutputImages = true;
	} else if (!strcmp(*args, "-outputScores")) {
            bOutputScores = true;
	} else if (!strcmp(*args, "-outputMarginals")) {
            bOutputMarginals = true;
	} else if (!strcmp(*args, "-outputConfusion")) {
            bOutputConfusion = true;
        } else if (!strcmp(*args, "-segwiseScore")) {
            bPixelwiseScore = false;
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

    const char *modelFilename = args[0];
    const char *evaluationList = args[1];
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

    // initialize models
    svlMultiClassLogistic logisticModel;
    svlPairwiseCRFModel crfModel;
    int numFeatures = 0;
    int numClasses = 0;
    if (!strcasecmp(modelType, "LOGISTIC")) {
	logisticModel.load(modelFilename);
	numFeatures = logisticModel.numFeatures();
	numClasses = logisticModel.numClasses();
    } else if (!strcasecmp(modelType, "CRF")) {
	crfModel.read(modelFilename);
	numFeatures = crfModel.dimSingleton();
	numClasses = crfModel.numClasses();
    } else {
	usage();
	return -1;
    }

    // load evaluation images
    SVL_LOG(SVL_LOG_MESSAGE, "Reading evaluation list from " << evaluationList << "...");
    vector<string> baseNames;
    ifstream ifs(evaluationList);
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
    
    int hImageLoad = svlCodeProfiler::getHandle("imageLoad");
    int hFeaturesLoad = svlCodeProfiler::getHandle("featuresLoad");
    int hLogisticInference = svlCodeProfiler::getHandle("inference (logistic)");
    int hCRFInference = svlCodeProfiler::getHandle("inference (crf)");

    svlConfusionMatrix confusion(numClasses);

    // process instances
    for (int n = 0; n < (int)baseNames.size(); n++) {
        // instantiate svlSegImage
        SVL_LOG(SVL_LOG_MESSAGE, "Processing image " << baseNames[n]);
        string imgName = string(imgDir) + string("/") + baseNames[n] + string(imgExt);
        string segName = string(segDir) + string("/") + baseNames[n] + string(segExt);
        string lblName = string(lblDir) + string("/") + baseNames[n] + string(lblExt);

        svlCodeProfiler::tic(hImageLoad);
        svlSegImage evalImage(imgName.c_str(), segName.c_str(), lblName.c_str());
        svlCodeProfiler::toc(hImageLoad);
        SVL_LOG(SVL_LOG_VERBOSE, "..." << toString(*evalImage.image()));

	// show segmentation
	if (bVisualize) {
	    IplImage *debugImage = evalImage.visualize();
	    cvShowImage(WINDOW_NAME, debugImage);
	    if (cvWaitKey(-1) == 27) {
		bVisualize = false;
		cvReleaseImage(&debugImage);
		break;
	    }
            
	    cvZero(debugImage);
	    for (int j = 0; j < evalImage.numSegments(); j++) {
		int indx = evalImage.getLabel(j);
		if (indx < 0) continue;
		for (int k = 0; k < (int)regionDefinitions.size(); k++) {
		    if (regionDefinitions[k].id == indx) {
			evalImage.colorSegment(debugImage, j, regionDefinitions[k].red,
			    regionDefinitions[k].green, regionDefinitions[k].blue);
			break;
		    }
		}
	    }
	    evalImage.colorBoundaries(debugImage);
	    cvShowImage(WINDOW_NAME, debugImage);
	    if (cvWaitKey(-1) == 27) {
		bVisualize = false;
	    }

	    cvReleaseImage(&debugImage);
	}

	// load features and add to training instances
	svlPairwiseCRFInstance instance;
	SVL_LOG(SVL_LOG_VERBOSE, "...computing features");
	svlCodeProfiler::tic(hFeaturesLoad);

        string featuresName = string(featuresDir) + string("/") + baseNames[n] + string(featuresExt);
        ifstream ifsData(featuresName.c_str());
        assert(!ifsData.fail());            
        while (1) {
            // read feature vectors
            vector<double> v(numFeatures);
            for (int j = 0; j < numFeatures; j++) {
                ifsData >> v[j];
            }
            if (ifsData.fail()) break;
            
            instance.Xn.push_back(v);
        }
        ifs.close();
	assert((int)instance.Xn.size() == evalImage.numSegments());
	svlCodeProfiler::toc(hFeaturesLoad);
       
	SVL_LOG(SVL_LOG_VERBOSE, "...assigning groundtruth labels");
	instance.Yn.resize(evalImage.numSegments(), -1);
	for (int j = 0; j < evalImage.numSegments(); j++) {
	    instance.Yn[j] = evalImage.getLabel(j);
	}
	
	SVL_LOG(SVL_LOG_VERBOSE, "...computing graph structure");
	instance.edges = evalImage.getAdjacencyList();
	instance.Xnm.resize(instance.numEdges());
	for (int j = 0; j < (int)instance.Xnm.size(); j++) {
	    instance.Xnm[j].resize(1, 1.0);
	}

#if 1
        // add location to edge potentials
        for (int j = 0; j < (int)instance.edges.size(); j++) {
            pair<int, int> edge = instance.edges[j];
            double y = (double)(evalImage.getCentroid(edge.first).y +
                evalImage.getCentroid(edge.second).y) / (2.0 * evalImage.height());
            double x = (double)(evalImage.getCentroid(edge.first).x +
                evalImage.getCentroid(edge.second).x) / (2.0 * evalImage.width());
            instance.Xnm[j].push_back(y);
            instance.Xnm[j].push_back(x);
        }
#endif

#if 0
        // TODO: whiten features
#endif

        // evaluate model on instance
	vector<vector<double> > marginals;
        
	// run logistic model
	if (!strcasecmp(modelType, "LOGISTIC")) {
            svlCodeProfiler::tic(hLogisticInference);	    
            vector<double> m;
            for (int j = 0; j < (int)instance.Xn.size(); j++) {
                logisticModel.evaluateMarginal(instance.Xn[j], m);
                marginals.push_back(m);
            }
            svlCodeProfiler::toc(hLogisticInference);
        }
    
	// run crf model
	if (!strcasecmp(modelType, "CRF")) {
	    svlCodeProfiler::tic(hCRFInference);
#if 0
            // run sum-product loopy
	    crfModel.inference(instance, marginals, 500);
#else
            // run max-product loopy
	    vector<int> maxMarginals;
	    crfModel.inference(instance, maxMarginals, 500);
            marginals.resize(maxMarginals.size());
            for (int i = 0; i < (int)maxMarginals.size(); i++) {
                marginals[i].resize(numClasses, 0.0);
                marginals[i][maxMarginals[i]] = 1.0;
            }
#endif
	    svlCodeProfiler::toc(hCRFInference);
	}

        // compute max marginal
        vector<int> predictedLabels = argmaxs(marginals);

        // score image
        double imageScore = 0.0;
        if (bPixelwiseScore) {
            imageScore = scoreImagePixelwise(&evalImage, predictedLabels, confusion);
        } else {
            imageScore = scoreImageSegmentwise(instance.Yn, predictedLabels, confusion);
        }
        SVL_LOG(SVL_LOG_MESSAGE, "..." << imageScore);
        
        // show results
        if ((bVisualize) && (regionsFilename != NULL)) {
            IplImage *debugImage = evalImage.visualize();
            for (int i = 0; i < evalImage.numSegments(); i++) {
                for (int k = 0; k < (int)regionDefinitions.size(); k++) {
                    if (predictedLabels[i] == regionDefinitions[k].id) {
                        evalImage.colorSegment(debugImage, i, regionDefinitions[k].red,
                            regionDefinitions[k].green, regionDefinitions[k].blue);
                        break;
                    }
                }
            }
            evalImage.colorBoundaries(debugImage);
            cvShowImage(WINDOW_NAME, debugImage);
            cvWaitKey(50);
            cvReleaseImage(&debugImage);
        }
        
        // output results
        if (bOutputImages) {
            IplImage *outputImage = evalImage.visualize();
            // color segments
            for (int i = 0; i < evalImage.numSegments(); i++) {
                for (int k = 0; k < (int)regionDefinitions.size(); k++) {
                    if (predictedLabels[i] == regionDefinitions[k].id) {
                        evalImage.colorSegment(outputImage, i, regionDefinitions[k].red,
                            regionDefinitions[k].green, regionDefinitions[k].blue);
                        break;
                    }
                }
            }

            string outputFilename = string(outputDir) + string("/") + string(baseNames[n]);
            outputFilename += string(".results.png");
            cvSaveImage(outputFilename.c_str(), outputImage);
            cvReleaseImage(&outputImage);
        }

        // output scores
        if (bOutputScores) {
            string outputFilename = string(outputDir) + string("/") + string(baseNames[n]);
            outputFilename += string(".score.txt");
            ofstream ofs(outputFilename.c_str());
            assert(!ofs.fail());
            ofs << imageScore << endl;
            ofs.close();
        }
        
        // output marginals
        if (bOutputMarginals) {
            string outputFilename = string(outputDir) + string("/") + string(baseNames[n]);
            outputFilename += string(".marginal.txt");
            ofstream ofs(outputFilename.c_str());
            assert(!ofs.fail());
            for (int m = 0; m < (int)marginals.size(); m++) {
                for (int k = 0; k < numClasses; k++) {
                    ofs << " " << marginals[m][k];
                }
                ofs << "\n";
            }
            ofs.close();
        }                
    }
    SVL_LOG(SVL_LOG_VERBOSE, "...done");

    // show confusion matrix
    confusion.printCounts();
    confusion.printRowNormalized();

    cout << "Class counts:" << endl;
    for (int i = 0; i < numClasses; i++)
	cout << "\t" << confusion.rowSum(i);
    cout << endl << endl << "Accuracy: "  << confusion.accuracy() << endl << endl;

    if (bOutputConfusion) {
	string outputFilename = string(outputDir) + string("/confusion.txt");
	ofstream ofs(outputFilename.c_str());
	assert(!ofs.fail());
	confusion.printCounts(ofs);
	ofs.close();
    }

    // free memory
    cvDestroyAllWindows();
    svlCodeProfiler::toc(svlCodeProfiler::getHandle("main"));
    svlCodeProfiler::print(cerr);
    return 0;
}

// helper functions ---------------------------------------------------------

double scoreImagePixelwise(svlSegImage* testImage, 
    const vector<int>& predictedLabels,
    svlConfusionMatrix& confusion)
{
        // score image
        double imageScore = 0.0;
        double totalPixels = 0.0;

        // pixel-wise accuracy
        assert(testImage != NULL);
        for (int i = 0; i < testImage->numSegments(); i++) {
            for (int y = 0; y < testImage->height(); y++) {
                for (int x = 0; x < testImage->width(); x++) {
                    int actual = testImage->getLabel(x, y);
                    if (actual < 0) continue;
                    int predicted = predictedLabels[testImage->getSegment(x, y)];
                    // accumulate confusion matrix
                    confusion.accumulate(actual, predicted);
                    imageScore += (actual == predicted ? 1.0 : 0.0);
                    totalPixels += 1.0;
                }
            }
        }

        if (totalPixels > 0.0)
            imageScore /= totalPixels;
        else imageScore = 1.0;

        return imageScore;
}

double scoreImageSegmentwise(const vector<int>& actualLabels,
    const vector<int>& predictedLabels,
    svlConfusionMatrix& confusion)
{
        // score image
        double imageScore = 0.0;
        double totalPixels = 0.0;

        // segment-wise accuracy
        confusion.accumulate(actualLabels, predictedLabels);
        for (int i = 0; i < (int)actualLabels.size(); i++) {
            int actual = actualLabels[i];
            if (actual < 0) continue;
            imageScore += (actual == predictedLabels[i] ? 1.0 : 0.0);
            totalPixels += 1.0;
        }

        if (totalPixels > 0.0)
            imageScore /= totalPixels;
        else imageScore = 1.0;
        
        return imageScore;
}
