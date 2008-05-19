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
** FILENAME:    svlSoftEdgeMap.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**
*****************************************************************************/

#include <cstdio>
#include <cassert>
#include <limits>

#include "svlOpenCVUtils.h"
#include "svlSoftEdgeMap.h"

using namespace std;

svlSoftEdgeMap::svlSoftEdgeMap() :
_imageBuffer(NULL), _hEdgeBuffer(NULL), _vEdgeBuffer(NULL)
{
    // do nothing
}

svlSoftEdgeMap::svlSoftEdgeMap(const svlSoftEdgeMap& m)  :
_imageBuffer(NULL), _hEdgeBuffer(NULL), _vEdgeBuffer(NULL)
{
    if (m._imageBuffer != NULL) {
        _imageBuffer = cvCloneImage(m._imageBuffer);
    }
    
    if (m._hEdgeBuffer != NULL) {
        _hEdgeBuffer = cvCloneImage(m._hEdgeBuffer);
    }

    if (m._vEdgeBuffer != NULL) {
        _vEdgeBuffer = cvCloneImage(m._vEdgeBuffer);
    }
}

svlSoftEdgeMap::~svlSoftEdgeMap()
{
    if (_imageBuffer != NULL) {
        cvReleaseImage(&_imageBuffer);
        cvReleaseImage(&_hEdgeBuffer);
        cvReleaseImage(&_vEdgeBuffer);
    }
}

const IplImage *svlSoftEdgeMap::processImage(const unsigned char *data, int w, int h, bool bNormalize)
{
    allocateMemory(w, h);

    // copy data into image
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            _imageBuffer->imageData[y * _imageBuffer->widthStep + x] = 
                (unsigned char)(0.299 * data[3 * (y * _imageBuffer->width + x) + 0] +
                0.587 * data[3 * (y * _imageBuffer->width + x) + 1] +
                0.114 * data[3 * (y * _imageBuffer->width + x) + 2]);
        }
    }

    // horizontal and vertical edge filters
    cvSobel(_imageBuffer, _hEdgeBuffer, 1, 0, 3);
    cvSobel(_imageBuffer, _vEdgeBuffer, 0, 1, 3);

    // combine and rescale
    for (int y = 0; y < h; y++) {
	const short *pGx = (const short *)&_hEdgeBuffer->imageData[y * _hEdgeBuffer->widthStep];
	const short *pGy = (const short *)&_vEdgeBuffer->imageData[y * _vEdgeBuffer->widthStep];
	unsigned char *p = (unsigned char *)&_imageBuffer->imageData[y * _imageBuffer->widthStep];
        for (int x = 0; x < w; x++) {
	    p[x] = MIN(255, (abs(pGx[x]) + abs(pGy[x])) / 2);
	}
    }
    if (bNormalize) {
	scaleToRange(_imageBuffer, 0.0, 255.0);
    }

    return _imageBuffer;
}

const IplImage *svlSoftEdgeMap::processImage(const IplImage *image, bool bNormalize)
{
    assert(image != NULL);
    allocateMemory(image->width, image->height);

    if (image->nChannels == 3) {
	cvCvtColor(image, _imageBuffer, CV_BGR2GRAY);
    } else {
	cvCopy(image, _imageBuffer);
    }

    // horizontal and vertical edge filters
    cvSobel(_imageBuffer, _hEdgeBuffer, 1, 0, 3);
    cvSobel(_imageBuffer, _vEdgeBuffer, 0, 1, 3);

    // combine and rescale
    for (int y = 0; y < image->height; y++) {
	const short *pGx = (const short *)&_hEdgeBuffer->imageData[y * _hEdgeBuffer->widthStep];
	const short *pGy = (const short *)&_vEdgeBuffer->imageData[y * _vEdgeBuffer->widthStep];
	unsigned char *p = (unsigned char *)&_imageBuffer->imageData[y * _imageBuffer->widthStep];
        for (int x = 0; x < image->width; x++) {
	    p[x] = MIN(255, (abs(pGx[x]) + abs(pGy[x])) / 2);
	}
    }
    if (bNormalize) {
	scaleToRange(_imageBuffer, 0.0, 255.0);
    }

    return _imageBuffer;
}

void svlSoftEdgeMap::allocateMemory(int w, int h)
{
    if (_imageBuffer == NULL) {
        _imageBuffer = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 1);
        _hEdgeBuffer = cvCreateImage(cvSize(w, h), IPL_DEPTH_16S, 1);
        _vEdgeBuffer = cvCreateImage(cvSize(w, h), IPL_DEPTH_16S, 1);
    } else if ((_imageBuffer->width != w) || (_imageBuffer->height != h)) {
        cvReleaseImage(&_imageBuffer);
        cvReleaseImage(&_hEdgeBuffer);
        cvReleaseImage(&_vEdgeBuffer);
        _imageBuffer = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 1);
        _hEdgeBuffer = cvCreateImage(cvSize(w, h), IPL_DEPTH_16S, 1);
        _vEdgeBuffer = cvCreateImage(cvSize(w, h), IPL_DEPTH_16S, 1);
    }
}
