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
** FILENAME:    svlPatchBasedObjectDetector.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**              Sid Batra <sidbatra@stanford.edu>
**
*****************************************************************************/

#include <cassert>
#include <iostream>
#include <fstream>
#include <vector>
#include <limits>

#include "opencv/highgui.h"

#include "svlBase.h"
#include "svlVision.h"

using namespace std;


// svlPatchBasedObjectDetector ----------------------------------------------

svlPatchBasedObjectDetector::svlPatchBasedObjectDetector(const char *name) :
    svlSlidingWindowDetector(name), _dictionary(_windowWidth, _windowHeight)
{
    // do nothing
}

svlPatchBasedObjectDetector::svlPatchBasedObjectDetector(const svlPatchBasedObjectDetector& c) :
    svlSlidingWindowDetector(c), _dictionary(c._dictionary)
{
    // do nothing
}

svlPatchBasedObjectDetector::~svlPatchBasedObjectDetector()
{
    // do nothing
}

// configuration and access functions
bool svlPatchBasedObjectDetector::readDictionary(const char *filename)
{
    if (_dictionary.read(filename)) {
        _windowWidth = _dictionary.windowWidth();
        _windowHeight = _dictionary.windowHeight();
        return true;
    }

    return false;
}
 
bool svlPatchBasedObjectDetector::writeDictionary(const char *filename)
{
    return _dictionary.write(filename);
}

// dictionary learning functions
void svlPatchBasedObjectDetector::initializePatchDictionary(int patchWidth, int patchHeight)
{
    _windowWidth = patchWidth;
    _windowHeight = patchHeight;
    _dictionary = svlPatchDictionary(_windowWidth, _windowHeight);
}

void svlPatchBasedObjectDetector::buildPatchDictionary(const vector<vector<IplImage *> >& images,
    vector<svlPatchDefinitionType>& imageTypes,        
    int samplesPerImage)
{
    _dictionary.buildDictionary(images, imageTypes, samplesPerImage);
}

vector<double> svlPatchBasedObjectDetector::evaluatePatchDictionary(const vector<IplImage *>& images)
{
    return _dictionary.patchResponse(images);
}

void svlPatchBasedObjectDetector::computeImageFeatureVectors(vector<vector<double> >& fv,
    const vector<CvPoint>& locations, const vector<IplImage *>& images)
{
    fv = _dictionary.imageResponse(images, locations);
}

void svlPatchBasedObjectDetector::computePatchFeatureVector(vector<double> &fv,
    const vector<IplImage *>& images)
{
    fv = _dictionary.patchResponse(images);	
}

