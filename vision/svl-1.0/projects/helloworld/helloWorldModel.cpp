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
** FILENAME:    helloWorldModel.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**
*****************************************************************************/

// C++ Standard Headers
#include <cstdlib>
#include <cassert>
#include <string>

// Open CV Headers
#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

// SVL Headers
#include "svlBase.h"
#include "svlML.h"
#include "svlVision.h"

// Project Headers
#include "helloWorldModel.h"

using namespace std;

// statics and constants -----------------------------------------------------

const int HelloWorldModel::FEATURE_LENGTH = 7;
const int HelloWorldModel::MAX_SAMPLES_PER_IMAGE = 50;

// constructors/destructors --------------------------------------------------

HelloWorldModel::HelloWorldModel() :
    logistic(FEATURE_LENGTH, 2)
{
    // do nothing
}

HelloWorldModel::~HelloWorldModel()
{
    // do nothing
}

// model i/o -----------------------------------------------------------------

bool HelloWorldModel::loadDetector(const char *filestem)
{
    string dictionaryFilename = string(filestem) + string(".dictionary");
    string modelFilename = string(filestem) + string(".model");

    bool success = detector.readDictionary(dictionaryFilename.c_str());
    success = success && detector.readModel(modelFilename.c_str());

    return success;
}

bool HelloWorldModel::loadLogistic(const char *filename)
{
    return logistic.save(filename);
}

bool HelloWorldModel::saveLogistic(const char *filename)
{
    return logistic.load(filename);
}
    
// training ------------------------------------------------------------------

void HelloWorldModel::learnModel(svlImageSequence& images,
    svlObject2dSequence& groundtruth)
{
    vector<vector<double> > data;
    vector<int> labels;

    // iterate through images building up training set
    for (unsigned i = 0; i < images.size(); i++) {
        string imageId = strBaseName(images[i]);
        SVL_LOG(SVL_LOG_MESSAGE, "Processing image " << imageId << "...");
        svlObject2dFrame samples = runDetectorOnImage(images.image(i), 0.01);

        // find samples which overlap with groundtruth by more than 50%
        if (groundtruth.find(imageId) != groundtruth.end()) {
            svlObject2dFrame truePositives = samples;
            removeNonGroundTruthObjects(truePositives, 
                groundtruth[imageId], 0.5);
            
            for (unsigned j = 0; j < truePositives.size(); j++) {
                data.push_back(extractFeatures(images.image(i),
                    truePositives[j]));
                labels.push_back(1);
            }

            removeGroundTruthObjects(samples, groundtruth[imageId], 0.5);
        }

        // add up to 50 remaining detections are negatives
        samples = sortObjects(samples);
        for (unsigned j = 0; j < samples.size(); j++) {
            data.push_back(extractFeatures(images.image(i), samples[j]));
            labels.push_back(0);
            if (j >= (unsigned)MAX_SAMPLES_PER_IMAGE) break;
        }        
    }
    
    // train logistic classifier
    logistic.train(data, labels);
}
    
// testing -------------------------------------------------------------------

svlObject2dFrame HelloWorldModel::evaluateModel(const IplImage *image)
{
    // set a low threshold so that we get a lot of candidate detections
    svlObject2dFrame detections = runDetectorOnImage(image, 0.01);

    // run logistic over each detection
    vector<double> marginal(2);
    for (unsigned i = 0; i < detections.size(); i++) {
        // extract feature vector for current detection
        vector<double> v = extractFeatures(image, detections[i]);
        // compute marginal probabilities for object/no object
        logistic.evaluateMarginal(v, marginal);
        // assign probability of object
        detections[i].pr = marginal[1];
    }
    
    return detections;
}

svlObject2dSequence HelloWorldModel::evaluateModel(svlImageSequence& images)
{
    svlObject2dSequence sequence;

    // iterate through image sequence evaluating each image
    for (unsigned i = 0; i < images.size(); i++) {
        IplImage *img = cvCloneImage(images.image(i));
        sequence[strBaseName(images[i])] = evaluateModel(img);
        cvReleaseImage(&img);
    }

    return sequence;
}

// feature extraction --------------------------------------------------------

svlObject2dFrame HelloWorldModel::runDetectorOnImage(const IplImage* image,
    double threshold)
{
    assert(detector.isInitialized());

    // construct vector containing intensity and gradient images
    svlSoftEdgeMap edgeMap;
    IplImage *intensityImage = greyImage(image);
    IplImage *gradientImage = cvCloneImage(edgeMap.processImage(intensityImage));
    vector<IplImage *> imageChannels(2);
    imageChannels[0] = intensityImage;
    imageChannels[1] = gradientImage;

    // run object detector
    detector.setThreshold(threshold);
    svlObject2dFrame detections = detector.classifyImage(imageChannels);

    // free intensity and gradient images
    cvReleaseImage(&gradientImage);
    cvReleaseImage(&intensityImage);

    return detections;
}

vector<double> HelloWorldModel::extractFeatures(const IplImage *image,
    const svlObject2d& detection) const
{
    vector<double> fv(FEATURE_LENGTH);

    IplImage *intensityImage = greyImage(image);

    // add image feature for all four compass directions and center
    fv[0] = intensityVariation(intensityImage, cvRect((int)detection.x,
        (int)detection.y, (int)detection.w, (int)detection.h));
    fv[1] = intensityVariation(intensityImage, cvRect((int)(detection.x - detection.w),
        (int)detection.y, (int)detection.w, (int)detection.h));
    fv[1] = intensityVariation(intensityImage, cvRect((int)(detection.x + detection.w),
        (int)detection.y, (int)detection.w, (int)detection.h));
    fv[3] = intensityVariation(intensityImage, cvRect((int)detection.x,
        (int)(detection.y - detection.h), (int)detection.w, (int)detection.h));
    fv[4] = intensityVariation(intensityImage, cvRect((int)detection.x,
        (int)(detection.y + detection.h), (int)detection.w, (int)detection.h));

    // add score from object detector
    fv[5] = log(detection.pr + SVL_EPSILON) - log(1.0 - detection.pr + SVL_EPSILON);

    // add constant (bias) term
    fv[6] = 1.0;

    // free intensity image
    cvReleaseImage(&intensityImage);

    return fv;
}

double HelloWorldModel::intensityVariation(IplImage *image, CvRect region) const
{
    // clip region to be within the image
    svlClipRect(region, image);
    if ((region.width == 0) || (region.height == 0))
        return 0.0;

    // compute variance of intensity within region
    cvSetImageROI(image, region);
    CvScalar v;
    cvAvgSdv(image, NULL, &v);
    cvResetImageROI(image);

    return v.val[0];
}
