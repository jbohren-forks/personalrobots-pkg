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
** FILENAME:    svlPatchDictionary.h
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**   Stores a patch dictionary. Each entry is defined by an image patch of
**   size W-by-H, and a window defined relative to some image/region. Also
**   implements a simple patch-based object detector.
**
*****************************************************************************/

#pragma once

#include <cassert>
#include <iostream>
#include <vector>

#include "cv.h"
#include "cxcore.h"

#include "svlObjectList.h"
#include "svlObjectDetector.h"

class svlPatchDictionary;

// svlPatchDefinition --------------------------------------------------------

class svlPatchDefinition {
 friend class svlPatchDictionary;
 protected:
    IplImage *_template;    // (single channel) patch template
    CvRect _validRect;      // valid response region
    int _validChannel;      // image channel (0 = greyscale, 1 = edge)

 public:
    svlPatchDefinition();
    svlPatchDefinition(IplImage *t, const CvRect& rect, int channel = 0);
    svlPatchDefinition(const svlPatchDefinition& def);
    virtual ~svlPatchDefinition();

    // i/o functions
    bool read(std::istream& is);
    bool write(std::ostream& os) const;

    // Compute response image for input image. The caller is responsible for 
    // freeing memory. Input images should have the correct number of channels
    // for the patch feature. Non-vectorized inputs assume that the correct
    // channel is being passed in (usually channel 0).
    IplImage *responseImage(const IplImage *image) const;
    IplImage *responseImage(std::vector<const IplImage *>& images) const;

    IplImage *responseImage(const IplImage *image, const IplImage *imageSum, const IplImage *imageSumSq) const;
    IplImage *responseImage(std::vector<const IplImage *>& images,
	std::vector<const IplImage *> &imageSums, std::vector<const IplImage *> &imageSumSqs) const;

    // Computes the response value for the patch at any given location of the
    // sliding window. responseImage should have been computed by responseImage()
    // function.
    double patchValue(const IplImage *responseImage, const CvPoint& location) const;

    // Computes the response value for the patch at all locations of the
    // sliding window given the image. Computes the response image on the
    // fly. The input image must have the correct number of channels.
    std::vector<double> patchValues(const IplImage *image, const std::vector<CvPoint>& locations) const;
    std::vector<double> patchValues(std::vector<const IplImage *>& images, const std::vector<CvPoint>& locations) const;
	
    // standard operators
    svlPatchDefinition& operator=(const svlPatchDefinition& patch);
};

// svlPatchDictionary -----------------------------------------------------------

class svlPatchDictionary {
 public:
    static bool INCLUDE_FILTER_FEATURES;

 protected:
    CvSize _windowSize;
    std::vector<svlPatchDefinition> _entries;

 public:
    svlPatchDictionary(unsigned width = 0, unsigned height = 0);
    virtual ~svlPatchDictionary();

    inline unsigned numEntries() const { return (unsigned)_entries.size(); }
    inline unsigned windowWidth() const { return _windowSize.width; }
    inline unsigned windowHeight() const { return _windowSize.height; }

    void clear();
    void truncate(unsigned n);
    bool read(const char *filename);
    bool write(const char *filename) const;

    // Builds dictionary by selecting n patches from each given samples. All samples
    // should be of size _windowSize.
    void buildDictionary(std::vector<IplImage *>& samples, unsigned n, bool bAppend = true);
    
    // Constructs a huge image with all the patches in it. Good for debugging.
    IplImage *visualizeDictionary() const;
    IplImage *visualizePatch(unsigned index, IplImage *image = NULL) const;

    // Compute response/feature vector for patch/image. The window size is assumed
    // to be the same size as the dictionary _windowSize.
    std::vector<double> patchResponse(IplImage *image) const;
    virtual std::vector<std::vector<double> > imageResponse(IplImage *image,
	    const std::vector<CvPoint>& windows) const;
};

// svlCachedPatchDictionary ----------------------------------------------------

class svlCachedPatchDictionary : public svlPatchDictionary {
 public:
    static unsigned NUM_RESPONSES_PER_ITERATION;
    static unsigned NUM_PIXEL_LIMIT;

 protected:
    mutable std::map<std::pair<int, int>, IplImage **> _responseCache;
    mutable std::map<std::pair<int, int>, int> _updateIndices;

 public:
    svlCachedPatchDictionary(unsigned width = 0, unsigned height = 0);
    svlCachedPatchDictionary(const svlCachedPatchDictionary &d);
    ~svlCachedPatchDictionary();

    void resetCache();

    virtual std::vector<std::vector<double> > imageResponse(IplImage *image,
	    const std::vector<CvPoint>& windows) const;
};

// svlPatchBasedClassifier -----------------------------------------------------

class svlPatchBasedClassifier : public svlObjectDetector {
 protected:
    svlCachedPatchDictionary _dictionary;

 public:
    svlPatchBasedClassifier(const char *name = NULL);
    svlPatchBasedClassifier(const svlPatchBasedClassifier& c);
    virtual ~svlPatchBasedClassifier();

    // configuration and access functions
    bool readDictionary(const char *filename);
    bool writeDictionary(const char *filename) const;

    inline bool isInitialized() const { 
    	return (_model != NULL) && (_dictionary.numEntries() > 0); 
    }
    
    // dictionary learning functions
    void initializePatchDictionary(int patchWidth = 32, int patchHeight = 32);
    void buildPatchDictionary(std::vector<IplImage *>& images, int samplesPerImage = 10);
    std::vector<double> evaluatePatchDictionary(IplImage * image) const;

 protected:
    void computeImageFeatureVectors(std::vector<std::vector<double> >& fv,
        const std::vector<CvPoint>& locations, IplImage *image);
    void computePatchFeatureVector(std::vector<double> &fv, IplImage *image);
};
