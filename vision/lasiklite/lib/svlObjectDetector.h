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
** DISCLAIMED. IN NO EVENT SHALL <copyright holder> BE LIABLE FOR ANY
** DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
** (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
** LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
** ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
** SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
*****************************************************************************
**
** FILENAME:    svlObjectDetector.h
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**  Defines interface for sliding-window boosted-classifier object detectors.
**  Derived classes must implement feature extraction functions and may override
**  other functions for more efficient implementation. Uses svlOptions interface
**  for handling training options.
**
*****************************************************************************/

#pragma once

#include <cassert>
#include <iostream>
#include <vector>
#include <map>

#include <cv.h>
#include <cxcore.h>

#include "svlOptions.h"
#include "svlObjectList.h"
#include "legacyml.h"

// svlObjectDetector ---------------------------------------------------------

class svlObjectDetector : public svlOptions {
 public:
    static bool DEBUG_OUTPUT;

 protected:
    legacy::CvStatModel *_model;
    int _windowWidth, _windowHeight;
    double _threshold;
    std::string _objectName;
    CvMat *_featureVector;

 public:
    svlObjectDetector(const char *name = NULL, int w = 32, int h = 32);
    svlObjectDetector(const svlObjectDetector& c);
    virtual ~svlObjectDetector();

    // configuration and access functions
    bool readModel(const char *filename);
    bool writeModel(const char *filename) const;

    int width() const { return _windowWidth; }
    int height() const { return _windowHeight; }
    virtual bool isInitialized() const { return (_model != NULL); }

    inline std::string getObjectName() const { return _objectName; }
    inline void setObjectName(std::string name) { _objectName = name; }
    inline void setObjectName(const char *name) { _objectName = std::string(name); }

    inline double getThreshold() const { return _threshold; }
    inline void setThreshold(double t) { _threshold = t; }

    inline legacy::CvStatModel* getModel() const { return _model; }

    // evaluate classifer over all shifts and scales
    virtual svlObject2dFrame classifyImage(const IplImage *image);
    // evaluate classifier at base scale and given shifts
    virtual svlObject2dFrame classifyImage(IplImage *image, 
	const std::vector<CvPoint>& locations);
    // evaluate classifier at given scales and locations
    virtual svlObject2dFrame classifyImage(IplImage *image,
	const std::map<double, std::vector<CvPoint> >& locations);
    // evaluate classifier on given region (and subwindows)
    virtual svlObject2dFrame classifyRegion(const IplImage *image, 
	CvRect region);
    // evaluate all image locations
    virtual void evaluateWindowLocations(IplImage *image,
	const std::map<double, std::vector<CvPoint> >& locations,
	std::map<double, std::vector<float> >& scores);
    // compute a response map for rectangle centroids (caller must free memory)
    virtual CvMat *responseMap(const IplImage *image);

    // create sliding window locations for given scale
    virtual void createWindowLocations(int width, int height,
	std::vector<CvPoint>& locations) const;
    virtual void createAllWindowLocations(int width, int height,
	std::map<double, std::vector<CvPoint> >& locations) const;

    // learning functions
    virtual void learnModel(const std::vector<std::vector<double> >& posSamples,
	const std::vector<std::vector<double> >& negSamples);
    virtual void learnModel(const std::vector<IplImage *>& posSamples,
	const std::vector<IplImage *>& negSamples);
    virtual void learnModel(const char *posDirectory, const char *negDirectory,
	const char *ext = ".jpg");
    float evaluateModel(const std::vector<double>& fv);
    float evaluateModel(IplImage *image, CvRect region = cvRect(0, 0, 0, 0));
	float evaluateMaskedModel(IplImage *image, CvRect region);

public:
    // Computes one feature vector for each location. The window (rectangle) over
    // which the i-th feature vector is defined is:
    //      <location[i].x, location[i].y, _windowWidth, _windowHeight>
    // The derived class must override this function.
    virtual void computeImageFeatureVectors(std::vector<std::vector<double> >& fv,
        const std::vector<CvPoint>& locations, IplImage *image) = 0;

	 protected:
    // Computes a single feature vector for the entire image patch.
    virtual void computePatchFeatureVector(std::vector<double> &fv, IplImage *image) = 0;
};
