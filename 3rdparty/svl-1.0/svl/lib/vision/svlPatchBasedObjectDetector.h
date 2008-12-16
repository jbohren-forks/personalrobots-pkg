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
** FILENAME:    svlPatchBasedObjectDetector.h
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**              Sid Batra<sidbatra@stanford.edu>
** DESCRIPTION:
**
*****************************************************************************/

#pragma once

#include <cassert>
#include <iostream>
#include <vector>

#include "opencv/cv.h"
#include "opencv/cxcore.h"

#include "svlBase.h"
#include "svlVision.h"
#include "svlSlidingWindowDetector.h"

using namespace std;

// svlPatchBasedObjectDetector ----------------------------------------------

class svlPatchBasedObjectDetector : public svlSlidingWindowDetector
{
 protected:
    svlPatchDictionary _dictionary;

 public:
    svlPatchBasedObjectDetector(const char *name = NULL);
    svlPatchBasedObjectDetector(const svlPatchBasedObjectDetector& c);
    virtual ~svlPatchBasedObjectDetector();

    // configuration and access functions
    bool readDictionary(const char *filename);
    bool writeDictionary(const char *filename);

    inline bool isInitialized() const { 
    	return (_model != NULL) && (_dictionary.numEntries() > 0); 
    }
    
    // dictionary learning functions
    void initializePatchDictionary(int patchWidth = 32, int patchHeight = 32);
    void buildPatchDictionary(const vector<vector<IplImage *> >& images, 
        vector<svlPatchDefinitionType>& imageTypes,        
        int samplesPerImage = 10);
    vector<double> evaluatePatchDictionary(const vector<IplImage *>& images);

 protected:
    void computeImageFeatureVectors(vector<vector<double> >& fv,
	const vector<CvPoint>& locations, const vector<IplImage *>& images);
    void computePatchFeatureVector(vector<double> &fv, 
        const vector<IplImage *>& images);
};
