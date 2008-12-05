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
** FILENAME:    svlSlidingWindowDetector.h
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**              Sid Batra <sidbatra@stanford.edu>
** DESCRIPTION:
**  Defines a multi-channel interface for sliding-window boosted-classifier 
**  object detectors. Derived classes must implement feature extraction 
**  functions and may override other functions for more efficient implementation.
**  Uses svlOptions interface for handling training options and basic feature
**  additions. The derived class svlSingleChannelSlidingWindowDetector implements
**  a single channel sliding-window detector.
**
*****************************************************************************/

#pragma once

#include <cassert>
#include <iostream>
#include <vector>
#include <map>

#include "opencv/cv.h"
#include "opencv/cxcore.h"
#include "opencv/ml.h"

#include "svlBase.h"
#include "svlVision.h"

using namespace std;

// svlSlidingWindowDetector ---------------------------------------------------

class svlSlidingWindowDetector : public svlOptions {
 public:
    static bool DEBUG_OUTPUT;
    static int DELTA_X;
    static int DELTA_Y;
    static double DELTA_SCALE;

 protected:
    svlSmartPointer<CvBoost> _model;
    int _windowWidth, _windowHeight;
    string _objectName;
    double _threshold;
    
    CvMat *_featureVector;
    CvMat *_weakResponses;

 public:
    svlSlidingWindowDetector(const char *name = NULL, int w = 32, int h = 32);
    svlSlidingWindowDetector(const svlSlidingWindowDetector& c);
    virtual ~svlSlidingWindowDetector();

    // configuration and access functions
    bool readModel(const char *filename);
    bool writeModel(const char *filename);

    int width() const { return _windowWidth; }
    int height() const { return _windowHeight; }
    virtual bool isInitialized() const { return (_model != NULL); }

    inline string getObjectName() const { return _objectName; }
    inline void setObjectName(string name) { _objectName = name; }
    inline void setObjectName(const char *name) { _objectName = string(name); }

    inline double getThreshold() const { return _threshold; }
    inline void setThreshold(double t) { _threshold = t; }

    // evaluate classifer over all shifts and scales
    virtual svlObject2dFrame classifyImage(const vector<IplImage *>& images);
    // evaluate classifier at base scale and given shifts
    virtual svlObject2dFrame classifyImage(const vector<IplImage *>& images,
        const vector<CvPoint>& locations);
    // evaluate classifier on given region (and subwindows)
    virtual svlObject2dFrame classifySubImage(const vector<IplImage *>& images,
	CvRect region);

    // create sliding window locations for given scale
    virtual void createWindowLocations(int width, int height, vector<CvPoint>& locations) const;
    virtual void createAllWindowLocations(int width, int height,
        map<double, vector<CvPoint> >& locations) const;

    // learning functions
    virtual void learnModel(const vector<vector<double> >& posSamples,
        const vector<vector<double> >& negSamples);	
    virtual void learnModel(const char *posDirectory, const char *negDirectory,
        const char *extesnions[], int numChannels = 1);

    // evaluate the model on a given feature vector/patch
    virtual float evaluateModel(const vector<double>& fv);
    virtual float evaluateModel(const vector<IplImage *>& images,
        CvRect region = cvRect(0, 0, 0, 0));
	
 protected:
    // Computes one feature vector for each location. The window (rectangle) over
    // which the i-th feature vector is defined is:
    //      <location[i].x, location[i].y, _windowWidth, _windowHeight>
    // The derived class must override this function.
    virtual void computeImageFeatureVectors(vector<vector<double> >& fv,
        const vector<CvPoint>& locations, const vector<IplImage*>& images) = 0;

    // Computes a single feature vector for the entire image patch.
    virtual void computePatchFeatureVector(vector<double> &fv, 
        const vector<IplImage*>& images) = 0;

    // Convert the score returned by the evaluateModel() function to a probability
    virtual float scoreToProbability(float score) const;
    virtual float probabilityToScore(float pr) const;
};

// svlSingleChannelSlidingWindowDetector ------------------------------------

class svlSingleChannelSlidingWindowDetector : public svlSlidingWindowDetector {
 public:
    svlSingleChannelSlidingWindowDetector();
    ~svlSingleChannelSlidingWindowDetector();

    // evaluate classifer over all shifts and scales
    virtual svlObject2dFrame classifyImage(IplImage *image);
    // evaluate classifier at base scale and given shifts
    virtual svlObject2dFrame classifyImage(IplImage *image,
        const vector<CvPoint>& locations);
    // evaluate classifier on given region (and subwindows)
    virtual svlObject2dFrame classifySubImage(IplImage *image, 
	CvRect region);

    // evaluate the model on a given feature vector/patch
    virtual float evaluateModel(IplImage * image,
        CvRect region = cvRect(0, 0, 0, 0));	
};
