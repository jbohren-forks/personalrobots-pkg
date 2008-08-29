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
** FILENAME:    segImageTrainBoostedFeatures.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**  Training code for multi-class image segmentation features.
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
    cerr << "USAGE: ./segImageTrainBoostedFeatures [OPTIONS] <trainingList>\n";
    cerr << "OPTIONS:\n"
         << "  -featuresExt <ext> :: features file extension (default: .features.txt)\n"
         << "  -segLblExt <ext>   :: segment labels file extension (default: .labels.txt)\n"
         << "  -featuresDir <dir> :: features file directory (default: .)\n"
         << "  -segLblDir <dir>   :: segment labels file directory (default: <featuresDir>)\n"
         << "  -o <filestem>      :: output model files (no training if missing)\n"
         << "  -outputExt <ext>   :: output model extension (default: .model)\n"
         << "  -rounds <num>      :: number of rounds of boosting (default: 100)\n"
         << "  -splits <num>      :: number of splits in weak learners (default: 2)\n"
         << "  -subSample <n>     :: subsample data by 1/<n>\n"
         << "  -balanced <alpha>  :: balance +ve and -ve training sets (dirichlet <alpha>)\n"
         << "  -v                 :: verbose (-v -v for debug)\n"
         << endl;
}

int main(int argc, char *argv[])
{
    svlCodeProfiler::enabled = true;
    svlCodeProfiler::tic(svlCodeProfiler::getHandle("main"));

    // read commandline parameters
    const int NUM_REQUIRED_PARAMETERS = 1;

    const char *featuresExt = ".features.txt";
    const char *segLblExt = ".labels.txt";
    const char *featuresDir = ".";
    const char *segLblDir = NULL;

    const char *outputModelStem = NULL;
    const char *outputExt = ".model";
    int numRounds = 100;
    int numSplits = 2;    
    int subSample = 0;
    int pseudoCounts = 0;

    int numLabels = -1;

    char **args = argv + 1;
    while (--argc > NUM_REQUIRED_PARAMETERS) {
        if (!strcmp(*args, "-featuresExt")) {
            featuresExt = *(++args); argc--;
        } else if (!strcmp(*args, "-segLblExt")) {
            segLblExt = *(++args); argc--;
        } else if (!strcmp(*args, "-featuresDir")) {
            featuresDir = *(++args); argc--;
        } else if (!strcmp(*args, "-segLblDir")) {
            segLblDir = *(++args); argc--;
        } else if (!strcmp(*args, "-o")) {
            outputModelStem = *(++args); argc--;
        } else if (!strcmp(*args, "-outputExt")) {
            outputExt = *(++args); argc--;
        } else if (!strcmp(*args, "-rounds")) {
            numRounds = atoi(*(++args)); argc--;
        } else if (!strcmp(*args, "-splits")) {
            numSplits = atoi(*(++args)); argc--;
        } else if (!strcmp(*args, "-subSample")) {
            subSample = atoi(*(++args)); argc--;
        } else if (!strcmp(*args, "-balanced")) {
            pseudoCounts = atoi(*(++args)); argc--;
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

    const char *trainingList = args[0];
    if (segLblDir == NULL) segLblDir = featuresDir;

    // load images
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

    int nFeatures = -1;

    // train boosted detectors
    if (outputModelStem != NULL) {
        
        vector<vector<double> > featureVectors;
        vector<int> featureLabels;
        
        SVL_LOG(SVL_LOG_MESSAGE, "Processing image features...");
        for (int i = 0; i < (int)baseNames.size(); i++) {
            string featuresName = string(featuresDir) + string("/") + baseNames[i] + string(featuresExt);
            string labelsName = string(segLblDir) + string("/") + baseNames[i] + string(segLblExt);
            
            ifstream ifsLabels(labelsName.c_str());
            if (ifsLabels.fail()) {
                SVL_LOG(SVL_LOG_ERROR, "labels file missing for " << baseNames[i]);
                continue;
            }

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
        
                // read feature vector and label
                vector<double> v(nFeatures);
                for (int j = 0; j < nFeatures; j++) {
                    ifsData >> v[j];
                }
                if (ifsData.fail()) break;
                
                int k;
                ifsLabels >> k;
                assert(!ifsLabels.fail());

                // accumulate non-void labels
                if (k < 0) continue;

                // random skip
                if ((subSample > 1) && (rand() % subSample != 0))
                    continue;

                featureVectors.push_back(v);
                featureLabels.push_back(k);
            }

            ifsData.close();
            ifsLabels.close();

            SVL_LOG(SVL_LOG_VERBOSE, "..." << featureVectors.size() << " feature vectors accumulated");
        }

        // train boosted superpixel classifiers
        numLabels = *max_element(featureLabels.begin(), featureLabels.end()) + 1;
        SVL_LOG(SVL_LOG_MESSAGE, featureVectors.size() << " feature vectors; " 
                << featureVectors[0].size() << " features; "
                << numLabels << " labels");
            
        CvMat *data = cvCreateMat((int)featureVectors.size(),
            (int)featureVectors[0].size(), CV_32FC1);
        CvMat *labels = cvCreateMat((int)featureVectors.size(), 1, CV_32SC1);
        assert((data != NULL) && (labels != NULL));
        double maxValue = -numeric_limits<double>::max();
        double minValue = numeric_limits<double>::max();
        for (int j = 0; j < (int)featureVectors.size(); j++) {
            for (int k = 0; k < data->width; k++) {
                cvmSet(data, j, k, featureVectors[j][k]);
                assert(!isnan(featureVectors[j][k]));
                if (featureVectors[j][k] > maxValue)
                    maxValue = featureVectors[j][k];
                if (featureVectors[j][k] < minValue)
                    minValue = featureVectors[j][k];
            }
        }
        SVL_LOG(SVL_LOG_MESSAGE, "Feature range: " << minValue << " to " << maxValue);
        
        for (int i = 0; i < numLabels; i++) {
            SVL_LOG(SVL_LOG_MESSAGE, "Training boosted classifier for class " << i << "...");
            // set up data vectors
            int posCount = 0;
            for (int j = 0; j < (int)featureVectors.size(); j++) {
                CV_MAT_ELEM(*labels, int, j, 0) = (featureLabels[j] == i) ? 1 : -1;
                posCount += (featureLabels[j] == i);
            }
            int negCount = featureVectors.size() - posCount;
            SVL_LOG(SVL_LOG_MESSAGE, "..." << posCount << " positive samples, " 
                << negCount << " negative samples");
            
            if (posCount == 0) continue;
            
            // train the classifier
            float *classPriors = NULL;
            if (pseudoCounts > 0) {
                classPriors = new float[2];
                classPriors[0] = (float)(posCount + pseudoCounts) /
                    (float)(negCount + posCount + 2 * pseudoCounts);
                classPriors[1] = (float)(negCount + pseudoCounts) / 
                    (float)(negCount + posCount + 2 * pseudoCounts);
            }
            CvBoostParams parameters(CvBoost::GENTLE, numRounds, 0.90, numSplits,
                false, classPriors);
            parameters.split_criteria = CvBoost::DEFAULT;
            CvMat *varType = cvCreateMat(data->width + 1, 1, CV_8UC1);
            for (int j = 0; j < data->width; j++) {
                CV_MAT_ELEM(*varType, unsigned char, j, 0) = CV_VAR_NUMERICAL;
            }
            CV_MAT_ELEM(*varType, unsigned char, data->width, 0) = CV_VAR_CATEGORICAL;
            svlCodeProfiler::tic(svlCodeProfiler::getHandle("boosting"));
            CvBoost *model = new CvBoost(data, CV_ROW_SAMPLE, labels,
                NULL, NULL, varType, NULL, parameters);
            svlCodeProfiler::toc(svlCodeProfiler::getHandle("boosting"));
            cvReleaseMat(&varType);
            if (classPriors != NULL) {
                delete[] classPriors;
            }
            
            // save the model
            string modelFilename = string(outputModelStem) + string(".class.") +
                toString(i) + string(outputExt);
            SVL_LOG(SVL_LOG_VERBOSE, "...saving to " << modelFilename);
            model->save(modelFilename.c_str());
            
            // evaluate model (on training data)
            int tp = 0; int tn = 0; int fp = 0; int fn = 0;
            
            CvMat *fv = cvCreateMat(1, (int)featureVectors[0].size(), CV_32FC1);
            for (int j = 0; j < (int)featureVectors.size(); j++) {
                for (int k = 0; k < data->width; k++) {
                    cvmSet(fv, 0, k, featureVectors[j][k]);
                }
                float score = model->predict(fv);
                if (CV_MAT_ELEM(*labels, int, j, 0) > 0) {
                    if (score > 0.0) {
                        tp += 1;
                    } else {
                        fn += 1;
                    }
                } else if (score > 0.0) {
                    fp += 1;
                } else {
                    tn += 1;
                }
            }
            cvReleaseMat(&fv);
            
            SVL_LOG(SVL_LOG_MESSAGE, "...done (TP, FN, TN, FP): " << tp << ", " << fn << ", " << tn << ", " << fp);
            
            delete model;            
        }

        // free memory
        cvReleaseMat(&labels);
        cvReleaseMat(&data);
    }

#if 0    
    // evaluate boosted detectors
    if (outputDir != NULL) {

        // load boosted detectors
        SVL_LOG(SVL_LOG_MESSAGE, "Reading boosted models...");

        vector<CvBoost *> models(numLabels);
        for (int i = 0; i < numLabels; i++) {
            string modelFilename = string(inputModelStem) + string(".class.") +
                toString(i) + string(".model");
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
                ofs << "1";
                for (unsigned k = 0; k < x[j].size(); k++) {
                    ofs << " " << x[j][k];
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
    }
#endif

    // print profile information
    svlCodeProfiler::toc(svlCodeProfiler::getHandle("main"));
    svlCodeProfiler::print(cerr);
    return 0;
}

