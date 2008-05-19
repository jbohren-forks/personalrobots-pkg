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
** FILENAME:    svlPatchDictionary.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**
*****************************************************************************/

#include <cassert>
#include <iostream>
#include <fstream>
#include <vector>
#include <limits>

#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

#include "svlOpenCVUtils.h"
#include "svlPatchDictionary.h"
#include "svlSoftEdgeMap.h"

#define USE_THREADS
#define USE_INTEGRAL_IMAGES

#ifdef USE_THREADS
#include <pthread.h>
#endif

using namespace std;

// Threading functions -------------------------------------------------------

#ifdef USE_THREADS
typedef struct _patch_response_args_t {
    const IplImage *image;
#ifdef USE_INTEGRAL_IMAGES
    const IplImage *sumImage;
    const IplImage *sumImage2;
#endif
    IplImage **response;    
    const svlPatchDefinition *patch;
} patch_response_args_t;

void *patch_response_fcn(void *args)
{
    const IplImage *image = ((patch_response_args_t *)args)->image; 
#ifdef USE_INTEGRAL_IMAGES
    const IplImage *sumImage = ((patch_response_args_t *)args)->sumImage; 
    const IplImage *sumImage2 = ((patch_response_args_t *)args)->sumImage2;    
#endif
    IplImage **response = ((patch_response_args_t *)args)->response;
    const svlPatchDefinition *patch = ((patch_response_args_t *)args)->patch;

#ifdef USE_INTEGRAL_IMAGES
    *response = patch->responseImage(image, sumImage, sumImage2);
#else
    *response = patch->responseImage(image);
#endif

    return NULL;
}
#endif

// svlPatchDefinition class --------------------------------------------------

svlPatchDefinition:: svlPatchDefinition() :   _template(NULL), _validChannel(0)
{
    _validRect = cvRect(0, 0, 0, 0);
}

svlPatchDefinition::svlPatchDefinition(IplImage *t, const CvRect& rect, int channel)
{
    assert((t->nChannels == 1) && (t->depth == 8));

    _template = cvCloneImage(t);
    _validRect.x = rect.x;
    _validRect.y = rect.y;
    _validRect.width = rect.width;
    _validRect.height = rect.height;
    _validChannel = channel;
}

svlPatchDefinition::svlPatchDefinition(const svlPatchDefinition& def)
{
    if (def._template != NULL) {
	_template = cvCloneImage(def._template);
    } else {
	_template = NULL;
    }
    _validRect = def._validRect;
    _validChannel = def._validChannel;
}

svlPatchDefinition::~svlPatchDefinition()
{
    if (_template != NULL) {
	cvReleaseImage(&_template);
    }
}

bool svlPatchDefinition::read(istream& is)
{
    if (_template != NULL) {
	cvReleaseImage(&_template);
    }

    is >> _validRect.x >> _validRect.y
       >> _validRect.width >> _validRect.height;

    is >> _validChannel;

    int w, h;
    is >> w >> h;

    _template = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 1);

    for (unsigned y = 0; y < (unsigned)_template->height; y++) 
	{
	char *p = &_template->imageData[y * _template->widthStep];
	
	for (unsigned x = 0; x < (unsigned)_template->width; x++) 
	{
	    unsigned n;
	    is >> n;
	    p[x] = (char)n;
	}
    }

    return true;
}

bool svlPatchDefinition::write(ostream& os) const
{
    assert(_template != NULL);

    os << _validRect.x << " " << _validRect.y << " "
       << _validRect.width << " " << _validRect.height << " "
       << _validChannel << endl;

    os << _template->width << " " << _template->height << endl;

    for (unsigned y = 0; y < (unsigned)_template->height; y++) {
	for (unsigned x = 0; x < (unsigned)_template->width; x++) {
	    if (x > 0) os << " ";
	    os << (int)CV_IMAGE_ELEM(_template, unsigned char, y, x);
	}
	os << endl;
    }

    return true;
}

IplImage *svlPatchDefinition::responseImage(const IplImage *image) const
{
    assert((_template != NULL) && (image != NULL));

    IplImage *response = cvCreateImage(cvSize(image->width - _template->width + 1,
	    image->height - _template->height + 1), IPL_DEPTH_32F, 1);
    cvMatchTemplate(image, _template, response, CV_TM_CCOEFF_NORMED);
    
    return response;
}

IplImage *svlPatchDefinition::responseImage(vector<const IplImage *>& images) const
{
    assert(images.size() > (unsigned)_validChannel);
    return responseImage(images[_validChannel]);
}

IplImage *svlPatchDefinition::responseImage(const IplImage *image, 
    const IplImage *imageSum, const IplImage *imageSumSq) const
{
    assert((_template != NULL) && (image != NULL) &&
        (imageSum != NULL) && (imageSumSq != NULL));

    IplImage *response = cvCreateImage(cvSize(image->width - _template->width + 1,
	    image->height - _template->height + 1), IPL_DEPTH_32F, 1);

    //assert(image->depth == IPL_DEPTH_8U);
    //assert(imageSum->depth == IPL_DEPTH_32S);
    //assert(imageSumSq->depth == IPL_DEPTH_64F);

    float patchSum = 0.0;
    float patchNorm = 0.0;
    unsigned char *pp = (unsigned char *)_template->imageData;
    for (int y = 0; y < _template->height; y++) {
        for (int x = 0; x < _template->width; x++) {
            patchSum += (float)pp[x];
            patchNorm += (float)(pp[x] * pp[x]);
        }
        pp += _template->widthStep;
    }
    float N = (float)(_template->width * _template->height);
    float patchMean = patchSum / N;
    patchNorm -= patchSum * patchMean;

    //int numMultiplications = 0;
    for (int y = 0; y < image->height - _template->height + 1; y++) {
        const unsigned char *p = &CV_IMAGE_ELEM(image, unsigned char, y, 0);
        const int *ps = &CV_IMAGE_ELEM(imageSum, int, y, 0);
        const int *ps2 = &CV_IMAGE_ELEM(imageSum, int, y + _template->height, 0);
        const double *psSq = &CV_IMAGE_ELEM(imageSumSq, double, y, 0);
        const double *psSq2 = &CV_IMAGE_ELEM(imageSumSq, double, y + _template->height, 0);
        float *q = &CV_IMAGE_ELEM(response, float, y, 0);
        for (int x = 0; x < image->width - _template->width + 1; x++, p++) {
            float windowSum = (float)(ps[x] + ps2[x + _template->width] - 
                ps[x + _template->width] - ps2[x]);
            float windowNorm = (float)(psSq[x] + psSq2[x + _template->width] - 
                psSq[x + _template->width] - psSq2[x]);
            const unsigned char *pp = (const unsigned char *)_template->imageData;
            const unsigned char *pw = p;
	    unsigned int convolutionSum = 0;
            for (int v = 0; v < _template->height; v++) {
                for (int u = 0; u < _template->width; u++) {
                    convolutionSum += pp[u] * pw[u];
                    //numMultiplications += 1;
                }
                pp += _template->widthStep;
                pw += image->widthStep;
            }

            windowNorm -= windowSum * windowSum / N;
            *q = (float)convolutionSum - patchMean * windowSum;
            *q /= sqrt(patchNorm * windowNorm);
            q++;
        }
    }

    //cerr << numMultiplications << " multiplications" << endl;
    return response;
}

IplImage *svlPatchDefinition::responseImage(vector<const IplImage *>& images, 
    vector<const IplImage *>& imageSums, vector<const IplImage *>& imageSumSqs) const
{
    assert((images.size() > (unsigned)_validChannel) &&
	(imageSums.size() > (unsigned)_validChannel) &&
	(imageSumSqs.size() > (unsigned)_validChannel));

    return responseImage(images[_validChannel], imageSums[_validChannel],
	imageSumSqs[_validChannel]);
}

double svlPatchDefinition::patchValue(const IplImage *responseImage,  const CvPoint& location) const
{
#if 0
    assert((location.x + _validRect.x + _validRect.width <= responseImage->width) &&
	   (location.y + _validRect.y + _validRect.height <= responseImage->height));
#endif
    float maxScore = -numeric_limits<float>::max();

    float *p = &CV_IMAGE_ELEM(responseImage, float, location.y + _validRect.y,
        location.x + _validRect.x);

    size_t inc = responseImage->widthStep / sizeof(float);

    for (int dy = 0; dy < _validRect.height; dy++) 
	{
	for (int dx = 0; dx < _validRect.width; dx++) 
	{
	    if (p[dx] > maxScore) 
		{
		maxScore = p[dx];
	    }
	}
	p += inc;
    }

    return (double)maxScore;
}

vector<double> svlPatchDefinition::patchValues(const IplImage *image,
    const vector<CvPoint>& locations) const
{
    assert(image->depth == IPL_DEPTH_8U);

    // compute patch statistics
    float patchSum = 0.0;
    float patchNorm = 0.0;
    unsigned char *pp = (unsigned char *)_template->imageData;
    for (int y = 0; y < _template->height; y++) {
        for (int x = 0; x < _template->width; x++) {
            patchSum += (float)pp[x];
            patchNorm += (float)pp[x] * (float)pp[x];
        }
        pp += _template->widthStep;
    }
    float N = (float)(_template->width * _template->height);
    float patchMean = patchSum / N;
    patchNorm -= patchSum * patchMean;

    // precompute these in patch dictionary
    vector<double> values;
    values.reserve(locations.size());

    for (vector<CvPoint>::const_iterator it = locations.begin(); it != locations.end(); ++it) {
        float maxVal = -numeric_limits<float>::max();
        for (int dy = _validRect.y; dy < _validRect.y + _validRect.height; dy++) {
            for (int dx = _validRect.x; dx < _validRect.x + _validRect.width; dx++) {
                const unsigned char *pw = &CV_IMAGE_ELEM(image, unsigned char, it->y + dy, it->x + dx);
                const unsigned char *pp = (const unsigned char *)_template->imageData;
                unsigned int windowSum = 0;
                unsigned int windowNorm = 0;
                unsigned int val = 0;
                for (int v = 0; v < _template->height; v++) {
                    for (int u = 0; u < _template->width; u++) {
                        windowSum += pw[u];
                        windowNorm += pw[u] * pw[u];
                        val += pp[u] * pw[u];
                    }
                    pp += _template->widthStep;
                    pw += image->widthStep;
                }

                float fWindowNorm = (float)windowNorm - (float)(windowSum * windowSum) / N;
                float fVal = (float)val - patchMean * (float)windowSum;
                fVal /= sqrt(patchNorm * fWindowNorm);
                if (fVal > maxVal) {
                    maxVal = fVal;
                }
            }
        }
        values.push_back((double)maxVal);
    }

    return values;
}

vector<double> svlPatchDefinition::patchValues(vector<const IplImage *>& images,
    const vector<CvPoint>& locations) const
{
    assert(images.size() > (unsigned)_validChannel);
    return patchValues(images[_validChannel], locations);
}

svlPatchDefinition& svlPatchDefinition::operator=(const svlPatchDefinition& patch)
{
    if (patch._template != NULL) 
	{
	_template = cvCloneImage(patch._template);
    } else 
	{
	_template = NULL;
    }
    _validRect = patch._validRect;
    _validChannel = patch._validChannel;

    return *this;
}

// svlPatchDictionary class --------------------------------------------------

bool svlPatchDictionary::INCLUDE_FILTER_FEATURES = true;

svlPatchDictionary::svlPatchDictionary(unsigned width, unsigned height)
{
    _windowSize = cvSize(width, height);
}

svlPatchDictionary::~svlPatchDictionary()
{
    // do nothing
}

void svlPatchDictionary::clear()
{
    _entries.clear();
}

void svlPatchDictionary::truncate(unsigned n)
{
    if (n < _entries.size()) {
	_entries.resize(n);
    }
}

bool svlPatchDictionary::read(const char *filename)
{
    _entries.clear();
    ifstream ifs(filename);
    if (ifs.fail()) {
	cerr << "WARNING: could not read patch dictionary " << filename << endl;
	return false; 
    }

    int n;
    ifs >> _windowSize.width >> _windowSize.height >> n;
    _entries.resize(n);

    for (unsigned i = 0; i < _entries.size(); i++) {
	_entries[i].read(ifs);
    }

    ifs.close();
    return true;
}

bool svlPatchDictionary::write(const char *filename) const
{
    ofstream ofs(filename);
    if (ofs.fail()) { return false; }

    ofs << _windowSize.width << " " << _windowSize.height << endl;
    ofs << _entries.size() << endl << endl;

    for (unsigned i = 0; i < _entries.size(); i++) {
	_entries[i].write(ofs);
	ofs << endl;
    }

    ofs.close();
    return true;
}

void svlPatchDictionary::buildDictionary(vector<IplImage *>& samples, unsigned n, bool bAppend)
{
    if (!bAppend) 
	{
		_entries.clear();
    }

    CvRNG rngState(0xffffffff);

    svlSoftEdgeMap edgeMap;
    IplImage *edgeFilterResponse = cvCreateImage(_windowSize, IPL_DEPTH_8U, 1);
    vector<IplImage *> imageChannels(2);

    for (unsigned i = 0; i < samples.size(); i++) 
	{
	assert((samples[i]) && (samples[i]->width == _windowSize.width) && (samples[i]->height == _windowSize.height));

	// compute edge map for image
	cvCopy(edgeMap.processImage(samples[i]), edgeFilterResponse);

	imageChannels[0] = samples[i];

	if (INCLUDE_FILTER_FEATURES)
	    imageChannels[1] = edgeFilterResponse;
	else imageChannels[1] = NULL;

	for (unsigned c = 0; c < imageChannels.size(); c++) 
	{
	    if (imageChannels[c] == NULL)
		continue;

	    for (unsigned j = 0; j < n; j++) 
		{
		CvRect r;

		// randomly sample rectangle in the window
		// minimum patch size is MIN(4, _windowSize / 8)
		// maximum patch size is _windowSize / 2
		const int minWidth = MIN(4, _windowSize.width / 8);
		const int minHeight = MIN(4, _windowSize.height / 8);
		r.width = (cvRandInt(&rngState) % (_windowSize.width / 2 - minWidth)) + minWidth;
		r.height = (cvRandInt(&rngState) % (_windowSize.height / 2 - minHeight)) + minHeight;
		r.x = cvRandInt(&rngState) % (_windowSize.width - r.width);
		r.y = cvRandInt(&rngState) % (_windowSize.height - r.height);

		//cerr << r.x << ", " << r.y << ", " << r.width << ", " << r.height << endl;

		IplImage *t = cvCreateImage(cvSize(r.width, r.height), IPL_DEPTH_8U, 1);
		cvSetImageROI(imageChannels[c], r);
		cvCopyImage(imageChannels[c], t);
		cvResetImageROI(imageChannels[c]);
		
		// patch is valid over 7x7 pixel region around source location
		r.x -= 3; r.y -=3; r.width = 7; r.height = 7;

		svlClipRect(r, _windowSize.width - t->width + 1, _windowSize.height - t->height + 1);

		_entries.push_back(svlPatchDefinition(t, r, c));

		cvReleaseImage(&t);
	    }
	}
    }

    cvReleaseImage(&edgeFilterResponse);
}

IplImage *svlPatchDictionary::visualizeDictionary() const
{
    int n = (int)ceil(sqrt((double)_entries.size()));
    IplImage *allPatches = cvCreateImage(cvSize(n * _windowSize.width, n * _windowSize.height),
					 IPL_DEPTH_8U, 3);
    cvZero(allPatches);
    for (unsigned i = 0; i < _entries.size(); i++) {
	CvRect r = cvRect((i % n) * _windowSize.width, ((int)(i / n)) * _windowSize.height,
			  _windowSize.width, _windowSize.height);
	cvRectangle(allPatches, cvPoint(r.x, r.y), cvPoint(r.x + r.width, r.y + r.height),
		    CV_RGB(0, 0, 255), 1);

	cvSetImageROI(allPatches, cvRect(r.x + _entries[i]._validRect.x, r.y + _entries[i]._validRect.y,
		_entries[i]._template->width, _entries[i]._template->height));
	IplImage *templateCopy = cvCloneImage(_entries[i]._template);
	scaleToRange(templateCopy, 0.0, 255.0);
	for (unsigned k = 0; k < 3; k++) {
	    cvSetImageCOI(allPatches, k + 1);
	    //cvCopyImage(_entries[i]._template, allPatches);
	    cvCopyImage(templateCopy, allPatches);
	}
	cvReleaseImage(&templateCopy);
	cvSetImageCOI(allPatches, 0);
	cvResetImageROI(allPatches);

	cvRectangle(allPatches, cvPoint(r.x + _entries[i]._validRect.x, r.y + _entries[i]._validRect.y),
		    cvPoint(r.x + _entries[i]._validRect.x + _entries[i]._validRect.width + _entries[i]._template->width,
			    r.y + _entries[i]._validRect.y + _entries[i]._validRect.height + _entries[i]._template->height),
		    CV_RGB(255, 0, 0), 1);
    }

    return allPatches;
}

IplImage *svlPatchDictionary::visualizePatch(unsigned index, IplImage *image) const
{
    // create image of the right size (unless one has been passed in)
    if ((image == NULL) || (image->width != _windowSize.width) ||
	(image->height != _windowSize.height) || (image->depth != IPL_DEPTH_8U) ||
	(image->nChannels != 3)) {
	if (image != NULL) {
	    cvReleaseImage(&image);
	}
	image = cvCreateImage(cvSize(_windowSize.width, _windowSize.height),
	    IPL_DEPTH_8U, 3);
    }
    cvZero(image);
    
    // copy template into image
    cvSetImageROI(image, cvRect(_entries[index]._validRect.x, _entries[index]._validRect.y,
	    _entries[index]._template->width, _entries[index]._template->height));
    for (unsigned k = 0; k < 3; k++) {
	cvSetImageCOI(image, k + 1);
	cvCopyImage(_entries[index]._template, image);
    }
    cvSetImageCOI(image, 0);
    cvResetImageROI(image);

    // draw box around valid region
    cvRectangle(image, cvPoint(_entries[index]._validRect.x, _entries[index]._validRect.y),
	cvPoint(_entries[index]._validRect.x + _entries[index]._validRect.width + _entries[index]._template->width,
	    _entries[index]._validRect.y + _entries[index]._validRect.height + _entries[index]._template->height),
	CV_RGB(255, 0, 0), 1);

    // indicate channel index
    for (int j = 0; j <= _entries[index]._validChannel; j++) {
	CvPoint pt =  cvPoint(_entries[index]._validRect.x + _entries[index]._validRect.width + _entries[index]._template->width,
	    _entries[index]._validRect.y + _entries[index]._validRect.height + _entries[index]._template->height);
	cvLine(image, cvPoint(pt.x - 2 * (j + 1), pt.y - 2), cvPoint(pt.x - 2 * (j + 1), pt.y - 3), CV_RGB(255, 0, 0)); 
    }
    
    return image;
}

vector<double> svlPatchDictionary::patchResponse(IplImage *image) const
{
	
    assert((image != NULL) && (image->depth == IPL_DEPTH_8U) && (image->nChannels == 1));
    vector<double> fv(_entries.size());

    if ((image->width != _windowSize.width) && (image->height != _windowSize.height)) {
	// TO DO
	assert(false);
    }

    svlSoftEdgeMap edgeMap;
    edgeMap.processImage(image);

    for (unsigned i = 0; i < _entries.size(); i++) 
	{
	IplImage *response;
	if (_entries[i]._validChannel == 1) 
	{
	    response = _entries[i].responseImage(edgeMap.getImage());
	}
	else 
	{
	    response = _entries[i].responseImage(image);
	}

		fv[i] = _entries[i].patchValue(response, cvPoint(0, 0));
		cvReleaseImage(&response);
    }

    return fv;
}

vector<vector<double> > svlPatchDictionary::imageResponse(IplImage *image,
    const vector<CvPoint>& windows) const
{
	
    // allocate memory for return vectors
    vector<vector<double> > v(windows.size());
    for (unsigned i = 0; i < windows.size(); i++) {
        v[i].resize(_entries.size());
    }

    // compute soft edge map for filter features
    svlSoftEdgeMap edgeMap;
    edgeMap.processImage(image);

    unsigned maxLocations = (image->width - _windowSize.width + 4) *
        (image->height - _windowSize.height + 4) / 16;

    if ((windows.size() > maxLocations / 4) && (_entries.size() < 200)) {

#ifdef USE_THREADS
	IplImage **responses = new IplImage * [_entries.size()];
	patch_response_args_t *args = new patch_response_args_t[_entries.size()];
	pthread_t *threads = new pthread_t[_entries.size()];

#ifdef USE_INTEGRAL_IMAGES
        // pre-compute integral images for use by patch responses
        IplImage *imageSum = cvCreateImage(cvSize(image->width + 1, image->height + 1),
            IPL_DEPTH_32S, 1);
        IplImage *imageSumSq = cvCreateImage(cvSize(image->width + 1, image->height + 1),
            IPL_DEPTH_64F, 1);
        cvIntegral(image, imageSum, imageSumSq);

        IplImage *imageSum2 = cvCreateImage(cvSize(image->width + 1, image->height + 1),
            IPL_DEPTH_32S, 1);
        IplImage *imageSumSq2 = cvCreateImage(cvSize(image->width + 1, image->height + 1),
            IPL_DEPTH_64F, 1);
        cvIntegral(edgeMap.getImage(), imageSum2, imageSumSq2);
#endif

	for (unsigned i = 0; i < _entries.size(); i++) {
	    if (_entries[i]._validChannel == 1) {
		args[i].image = edgeMap.getImage();
#ifdef USE_INTEGRAL_IMAGES
		args[i].sumImage = imageSum2;
		args[i].sumImage2 = imageSumSq2;
#endif
	    } else {
		args[i].image = image;
#ifdef USE_INTEGRAL_IMAGES
		args[i].sumImage = imageSum;
		args[i].sumImage2 = imageSumSq;
#endif
	    }
	    args[i].response = &responses[i];
	    args[i].patch = &_entries[i];
	    pthread_create(&threads[i], NULL, patch_response_fcn, (void *)&args[i]);
	}

	for (unsigned i = 0; i < _entries.size(); i++) {
	    pthread_join(threads[i], NULL);
	    for (unsigned j = 0; j < windows.size(); j++) {
		v[j][i] = _entries[i].patchValue(responses[i], windows[j]);
	    }
	    cvReleaseImage(&responses[i]);
	}

#ifdef USE_INTEGRAL_IMAGES
        // release integral images
        cvReleaseImage(&imageSumSq);
        cvReleaseImage(&imageSum);
        cvReleaseImage(&imageSumSq2);
        cvReleaseImage(&imageSum2);
#endif
	
	delete[] threads;
	delete[] args;
	delete[] responses;
	
#else

        // pre-compute integral images for use by patch responses
        IplImage *imageSum = cvCreateImage(cvSize(image->width + 1, image->height + 1),
            IPL_DEPTH_32S, 1);
        IplImage *imageSumSq = cvCreateImage(cvSize(image->width + 1, image->height + 1),
            IPL_DEPTH_64F, 1);
        cvIntegral(image, imageSum, imageSumSq);

        IplImage *imageSum2 = cvCreateImage(cvSize(image->width + 1, image->height + 1),
            IPL_DEPTH_32S, 1);
        IplImage *imageSumSq2 = cvCreateImage(cvSize(image->width + 1, image->height + 1),
            IPL_DEPTH_64F, 1);
        cvIntegral(edgeMap.getImage(), imageSum2, imageSumSq2);

        // iterate through patches
        for (unsigned i = 0; i < _entries.size(); i++) {
            IplImage *response;
	    if (_entries[i]._validChannel == 1) {
		response = _entries[i].responseImage(edgeMap.getImage(), imageSum2, imageSumSq2);
	    } else {
		response = _entries[i].responseImage(image, imageSum, imageSumSq);
	    }
#if 0
            cvNamedWindow("debug", 1);
            cvShowImage("debug", response);
            cvWaitKey(0);
            cvDestroyWindow("debug");
#endif
	    for (unsigned j = 0; j < windows.size(); j++) {
                v[j][i] = _entries[i].patchValue(response, windows[j]);
            }
            
            cvReleaseImage(&response);
        }

        // release integral images
        cvReleaseImage(&imageSumSq);
        cvReleaseImage(&imageSum);
        cvReleaseImage(&imageSumSq2);
        cvReleaseImage(&imageSum2);
#endif

	} else {

	// create array of images
	vector<const IplImage *> imageChannels(2);
	imageChannels[0] = image;
	imageChannels[1] = edgeMap.getImage();

        // iterate through patches
        for (unsigned i = 0; i < _entries.size(); i++) {
            vector<double> p = _entries[i].patchValues(imageChannels, windows);
            for (unsigned j = 0; j < windows.size(); j++) {
                v[j][i] = p[j];
            }
        }
    }

	
    return v;
}

// svlCachedPatchDictionary ----------------------------------------------------

unsigned svlCachedPatchDictionary::NUM_RESPONSES_PER_ITERATION = numeric_limits<unsigned>::max();
unsigned svlCachedPatchDictionary::NUM_PIXEL_LIMIT = 80 * 60;

svlCachedPatchDictionary::svlCachedPatchDictionary(unsigned width, unsigned height) :
    svlPatchDictionary(width, height)
{
    // do nothing
}

svlCachedPatchDictionary::svlCachedPatchDictionary(const svlCachedPatchDictionary &d) :
    svlPatchDictionary(d)
{
    // don't copy cache
}

svlCachedPatchDictionary::~svlCachedPatchDictionary()
{
    resetCache();
}

void svlCachedPatchDictionary::resetCache()
{
    for (map<pair<int, int>, IplImage **>::const_iterator it = _responseCache.begin();
        it != _responseCache.end(); it++) {
            for (unsigned p = 0; p < _entries.size(); p++) {
                if (it->second[p] != NULL)
                    cvReleaseImage(&(it->second[p]));
            }
    }
    _responseCache.clear();
    _updateIndices.clear();
}

vector<vector<double> > svlCachedPatchDictionary::imageResponse(IplImage *image,
    const vector<CvPoint>& windows) const
{
    if (_entries.size() <= NUM_RESPONSES_PER_ITERATION) 
	{
#ifdef VERBOSE
	cerr << "Computing responses for all patches..." << endl;
#endif
        return svlPatchDictionary::imageResponse(image, windows);
    }

    if (image->width * image->height < (int)NUM_PIXEL_LIMIT) {
		#ifdef VERBOSE
	cerr << "Computing responses for all patches..." << endl;
	#endif
        return svlPatchDictionary::imageResponse(image, windows);
    }

    pair<int, int> cacheIndex(image->width, image->height);
    IplImage **responses;
    if (_responseCache.find(cacheIndex) == _responseCache.end()) {
        responses = new IplImage *[_entries.size()];
        for (unsigned p = 0; p < _entries.size(); p++) {
            responses[p] = NULL;
        }
        _responseCache[cacheIndex] = responses;
    } else {
        responses = _responseCache[cacheIndex];
    }

    int updateIndex = 0;
    if (_updateIndices.find(cacheIndex) != _updateIndices.end()) {
        updateIndex = _updateIndices[cacheIndex];
    }

    // create array of images channels
    svlSoftEdgeMap edgeMap;
    edgeMap.processImage(image);
    vector<const IplImage *> imageChannels(2);
    imageChannels[0] = image;
    imageChannels[1] = edgeMap.getImage();

    // update response cache
    cerr << "Computing responses for " << NUM_RESPONSES_PER_ITERATION << " patches..." << endl;
    for (unsigned i = 0; i < NUM_RESPONSES_PER_ITERATION; i++) {
        if (responses[updateIndex] != NULL) {
            cvReleaseImage(&responses[updateIndex]);
        }
	responses[updateIndex] = _entries[updateIndex].responseImage(imageChannels);
        updateIndex = (updateIndex + 1) % (int)_entries.size();
    }
    _updateIndices[cacheIndex] = updateIndex;

    // allocate memory for return vectors
    vector<vector<double> > v(windows.size());
    for (unsigned i = 0; i < windows.size(); i++) {
        v[i].resize(_entries.size());
    }

    // compute features
    for (unsigned i = 0; i < _entries.size(); i++) {
        if (responses[i] == NULL) continue;
	for (unsigned j = 0; j < windows.size(); j++) {
	    v[j][i] = _entries[i].patchValue(responses[i], windows[j]);
	}
    }

	

    return v;
}

// svlPatchBasedClassifier class -------------------------------------------

svlPatchBasedClassifier::svlPatchBasedClassifier(const char *name) : svlObjectDetector(name), _dictionary(_windowWidth, _windowHeight)
{
    // do nothing
}

svlPatchBasedClassifier::svlPatchBasedClassifier(const svlPatchBasedClassifier& c) :
    svlObjectDetector(c), _dictionary(c._dictionary)
{
    // do nothing
}

svlPatchBasedClassifier::~svlPatchBasedClassifier()
{
    // do nothing
}

// configuration and access functions
bool svlPatchBasedClassifier::readDictionary(const char *filename)
{
    if (_dictionary.read(filename)) {
        _windowWidth = _dictionary.windowWidth();
        _windowHeight = _dictionary.windowHeight();
	    return true;
    }

    return false;
}
 
bool svlPatchBasedClassifier::writeDictionary(const char *filename) const
{
    return _dictionary.write(filename);
}

// dictionary learning functions
void svlPatchBasedClassifier::initializePatchDictionary(int patchWidth, int patchHeight)
{
    _windowWidth = patchWidth;
    _windowHeight = patchHeight;
    _dictionary = svlCachedPatchDictionary(_windowWidth, _windowHeight);
}

void svlPatchBasedClassifier::buildPatchDictionary(vector<IplImage *>& images,int samplesPerImage)
{
    _dictionary.buildDictionary(images, samplesPerImage, true);
}

vector<double> svlPatchBasedClassifier::evaluatePatchDictionary(IplImage * image) const
{
    return _dictionary.patchResponse(image);
}

void svlPatchBasedClassifier::computeImageFeatureVectors(std::vector<std::vector<double> >& fv,
    const std::vector<CvPoint>& locations, IplImage *image)
{
	
    fv = _dictionary.imageResponse(image, locations);
}

void svlPatchBasedClassifier::computePatchFeatureVector(std::vector<double> &fv, IplImage *image)
{
    fv = _dictionary.patchResponse(image);
}
