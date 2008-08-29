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
** FILENAME:    helloWorldModel.h
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**  Helloworld example application.
**
*****************************************************************************/

#pragma once

// C++ Standard Headers
#include <cstdlib>
#include <cassert>

// Open CV Headers
#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

// SVL Headers
#include "svlBase.h"
#include "svlML.h"
#include "svlVision.h"

using namespace std;

class HelloWorldModel {
 protected:
    // number of features in logistic classifier
    static const int FEATURE_LENGTH;
    // maximum number of training samples per image
    static const int MAX_SAMPLES_PER_IMAGE;

    // pre-trained detector
    svlPatchBasedObjectDetector detector;

    // binary logistic model that incorporates context
    svlMultiClassLogistic logistic;

 public:
    // constructors/destructors
    HelloWorldModel();
    ~HelloWorldModel();

    // model i/o
    bool loadDetector(const char *filestem);
    bool loadLogistic(const char *filename);
    bool saveLogistic(const char *filename);
    
    // training
    void learnModel(svlImageSequence& images,
        svlObject2dSequence& groundtruth);
    
    // testing
    svlObject2dFrame evaluateModel(const IplImage *image);    
    svlObject2dSequence evaluateModel(svlImageSequence& images);

 protected:
    // feature extraction
    svlObject2dFrame runDetectorOnImage(const IplImage* image, 
        double threshold);
    vector<double> extractFeatures(const IplImage *image,
        const svlObject2d& detection) const;
    double intensityVariation(IplImage *image, CvRect region) const;
};
