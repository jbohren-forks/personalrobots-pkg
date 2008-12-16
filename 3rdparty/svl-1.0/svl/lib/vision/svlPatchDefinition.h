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
** FILENAME:    svlPatchDefinition.h
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**              Sid Batra <sidbatra@cs.stanford.edu>
** DESCRIPTION:
**   Stores a generic definition of a patch. Which is inherited by patch
**   definitions to represent seperate channels. The patches are written
**   and read in XML format.
**
*****************************************************************************/

#pragma once

#include <cassert>
#include <iostream>
#include <vector>

#include "opencv/cv.h"
#include "opencv/cxcore.h"

#include "xmlParser/xmlParser.h"

#include "svlBase.h"
#include "svlVision.h"

using namespace std;

// svlPatchDefinitionType ----------------------------------------------------

typedef enum _svlPatchDefinitionType {
    SVL_INTENSITY_PATCH, SVL_DEPTH_PATCH
} svlPatchDefinitionType;

// svlPatchDefinition --------------------------------------------------------

class svlPatchDefinition 
{
    friend class svlPatchDictionary;

 protected:
    IplImage *_template;    // (single channel) patch template
    CvRect _validRect;      // valid response region
    int _validChannel;      // channel information

 public:
    svlPatchDefinition();
    svlPatchDefinition(XMLNode& node);
    svlPatchDefinition(const IplImage *t, const CvRect& rect, int channel = 0);
    virtual ~svlPatchDefinition();

    // constuction helper functions
    static svlSmartPointer<svlPatchDefinition>
        createPatchDefinition(svlPatchDefinitionType t);
    static svlSmartPointer<svlPatchDefinition>
        createPatchDefinition(XMLNode& node);

    // i/o functions
    virtual svlPatchDefinitionType patchType() const = 0;
    virtual bool read(XMLNode& node);
    virtual bool write(ostream& os);

    //Returns the valid channel index
    int validChannel() { return _validChannel; }
    void setValidChannel( int newChannel ) { _validChannel = newChannel ; }

    // Compute response image for input image. The caller is responsible for 
    // freeing memory. Input images should have the correct number of channels
    // for the patch feature. Non-vectorized inputs assume that the correct
    // channel is being passed in (usually channel 0).
    virtual IplImage *responseImage(const IplImage *image) const = 0;
    virtual IplImage *responseImage(const vector<IplImage *>& images) const;
       
    // Computes the response value for the patch at any given location of the
    // sliding window. responseImage should have been computed by responseImage()
    // function.
    virtual double patchValue(const IplImage *responseImage, const CvPoint& location);
    virtual vector<double> patchValues(IplImage *image, 
        const vector<CvPoint>& locations);
    virtual vector<double> patchValues(const vector<IplImage *>& images,
        const vector<CvPoint>& locations);
};

// svlIntensityPatchDefinition -----------------------------------------------

class svlIntensityPatchDefinition  : public svlPatchDefinition
{
 public:
    svlIntensityPatchDefinition();
    svlIntensityPatchDefinition(XMLNode& node);
    svlIntensityPatchDefinition(const IplImage *t, const CvRect& rect, int channel = 0);
    ~svlIntensityPatchDefinition();
    
    svlPatchDefinitionType patchType() const { return SVL_INTENSITY_PATCH; }
	bool read(XMLNode &node);

    // response images
    IplImage *responseImage(const IplImage *image) const;
    IplImage *responseImage(const IplImage *image, const IplImage *imageSum,
        const IplImage *imageSumSq) const;
    IplImage *responseImage(vector<const IplImage *>& images,
        vector<const IplImage *> &imageSums,
        vector<const IplImage *> &imageSumSqs) const;
    
    vector<double> patchValues(IplImage *image, 
        const vector<CvPoint>& locations);
	
};

// svlDepthPatchDefinition ---------------------------------------------------

class svlDepthPatchDefinition  : public svlPatchDefinition
{
 protected:
    static int _maxDistance;
    static int _totalBins;
    int *_histogram;

 public:
    static CvSize _windowSize;

 public:
    svlDepthPatchDefinition();
    svlDepthPatchDefinition(XMLNode& node);
    svlDepthPatchDefinition(const IplImage *t, const CvRect& rect, int channel = 1);
    ~svlDepthPatchDefinition();

    // i/o functions
    svlPatchDefinitionType patchType() const { return SVL_DEPTH_PATCH; }
    bool read(XMLNode& node);

    // response images
    IplImage *responseImage(const IplImage *image) const;

    vector<double> patchValues(IplImage *image, const vector<CvPoint>& windows);

 protected:
    // takes the depth values of the template patch and computes a depth histogram for the selected region
    int** computeDepthHistogram(const IplImage* patchTemplate , CvRect region);
    void smoothHistogram(int *histogram, int totalLength);

    // simple euclidean distance between 2 vectors
    int computeEuclideanDistance(int *a , int *b , int totalLength);
    int computeEMD(int *a , int *b , int totalLength);

    // double patchValue(const IplImage *responseImage,  const CvPoint& location);
};
