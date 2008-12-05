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
** FILENAME:    svlPatchDictionary.h
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**              Sid Batra <sidbatra@cs.stanford.edu>
** DESCRIPTION:
**   Stores a patch dictionary. Each entry is defined by a multichannel image
**   patch of size W-by-H, and a window defined relative to some image/region.
**   Also implements a simple multi-channel patch-based object detector.
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
#include "svlDevel.h"

// precompute median depth images
#define ENABLE_MEDIAN_IMAGES
#define USE_MEDIANHEAP
//#define USE_NORM_THREADS
//#define USE_BATCH_NORM_THREADS

using namespace std;

// svlPatchDictionary ---------------------------------------------------------

class svlPatchDictionary
{
 protected:
    CvSize _windowSize;
    vector<svlSmartPointer<svlPatchDefinition> > _entries;

 public:
    svlPatchDictionary(unsigned width = 0, unsigned height = 0);
    svlPatchDictionary(const svlPatchDictionary& d);
    virtual ~svlPatchDictionary();

    inline unsigned numEntries() const { return _entries.size(); }
    inline unsigned windowWidth() const { return _windowSize.width; }
    inline unsigned windowHeight() const { return _windowSize.height; }

    void clear();
    void truncate(unsigned n);
    bool read(const char *filename);
    bool write(const char *filename);

    // Builds dictionary by selecting n patches from each given samples.
    // All samples should be of size _windowSize.
    void buildDictionary(const vector<vector<IplImage *> >& samples,
        const vector<svlPatchDefinitionType>& imageTypes, unsigned n);

    //Alters the valid channel for each of the dictionary entries
    void alterValidChannel(int newChannel);

    //Merge all entries of given dictionary with the current entries
    void mergeDictionary( svlPatchDictionary input );

    // Constructs a huge image with all the patches in it. Good for debugging.
    IplImage *visualizeDictionary() const;
    IplImage *visualizePatch(unsigned index, IplImage *image = NULL) const;

    // Compute response/feature vector for patch/image. The window size is
    // assumed to be the same size as the dictionary _windowSize.
    vector<double> patchResponse(const vector<IplImage *> images) ;
    vector<vector<double> > imageResponse(const vector<IplImage *> images,
        const vector<CvPoint>& windows);

    static vector<IplImage*> _normalizedDepthPatches;

#if 0
 protected:
    void intensityChannelResponse(IplImage *image, const vector<CvPoint>& windows , vector<vector<double> > &v);
    void depthChannelResponse(IplImage *image, const vector<CvPoint>& windows , vector<vector<double> > &v);
#endif

#ifdef ENABLE_MEDIAN_IMAGES
    // TODO: make private when done testing
 public:
    // Compute and return the median image of the given image
    IplImage *medianImage(const IplImage* const image,
			  const int deltaX, const int deltaY,
			  const bool bInsertionSort = true) const;
    void normalizeImagePatches(IplImage* image,
			       const int deltaX, const int deltaY);
#ifndef USE_MEDIANHEAP
    // Insert a point into the given sorted vector
    void insertPointSorted(vector<pair<float, CvPoint> > &points,
			   pair<float, CvPoint> const key);
#endif

 protected:
    // Returns the patch at the given window
    inline IplImage *getPatch(IplImage* image, const int x, const int y);

#ifdef USE_NORM_THREADS
#ifdef USE_BATCH_NORM_THREADS
    // Run subtract patch with a list of patches & values
    static void *subtractPatches(void *argP);
#else
    // Subtract a value from each pixel of an image and synchronously push it
    // on the normalizedDepthPatches vector
    static void *subtractPatch(void *argP);
#endif
#else
    // same thing, not threaded
    inline IplImage *subtractPatch(IplImage *patch, const float value);
#endif

    // args for subtractPatch
    struct SubtractPatchArgs {
      SubtractPatchArgs(IplImage *p, const double v, const unsigned i) :
	patch(p), value(v), index(i) {}

      IplImage *patch;
      const double value;
      const unsigned index;
    };

#endif
};

