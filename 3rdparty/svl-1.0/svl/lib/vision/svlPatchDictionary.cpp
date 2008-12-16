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
** FILENAME:    svlPatchDictionary.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**              Sid Batra <sidbatra@cs.stanford.edu>
**              David Breeden <breeden@cs.stanford.edu>
**
*****************************************************************************/

#include <cassert>
#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <limits>

#include "opencv/highgui.h"

#include "xmlParser/xmlParser.h"

#include "svlBase.h"
#include "svlVision.h"
#include "svlDevel.h"

// Toggle image-only and multi-modal code
#define ENABLE_DEPTH

// speed up by using threads on a linux machine
#define USE_THREADS

// speed up intensity channels by using integral images
#define USE_INTEGRAL_IMAGES

#define TOTAL_THREADS 8

#if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
#undef max
#undef USE_THREADS
#endif

#ifdef USE_THREADS
#include <pthread.h>
#endif

using namespace std;

// Threading functions -------------------------------------------------------

// TODO: Change flags
#ifdef USE_THREADS

#ifdef ENABLE_DEPTH
struct NormalizePatchArgs {
  NormalizePatchArgs(IplImage* const p, const int i) : patch(p), index(i) {}

  IplImage* const patch;
  const int index;
};

void *normalizePatch(void *argsP) {
  NormalizePatchArgs *args = (NormalizePatchArgs*) argsP;
  IplImage* const patch = args->patch;

  vector<double> depths;

  for(int y = 0; y < patch->height; y++) {
    for(int x = 0; x < patch->width; x++) {
      depths.push_back(CV_IMAGE_ELEM(patch, float, y, x));
    }
  }

  sort(depths.begin(),depths.end());

  const double median = depths[depths.size()/2];

  for( int y=0; y<patch->height; y++) {
    for( int x=0; x<patch->width; x++) {
      CV_IMAGE_ELEM(patch, float, y, x) =
	CV_IMAGE_ELEM(patch, float, y, x) - median;
    }
  }

  svlPatchDictionary::_normalizedDepthPatches[args->index] = patch;

  delete args;

  return NULL;
}

struct CalcFVArgs {
  CalcFVArgs(vector<svlSmartPointer<svlPatchDefinition> > *e,
		  const int idx, vector<IplImage*> *img,
		  const vector<CvPoint> *w, vector<vector<double> > *v) :
    entries(e), index(idx), images(img), windows(w), fv(v) {}

  vector<svlSmartPointer<svlPatchDefinition> > *entries;
  const int index;
  vector<IplImage*> *images;
  const vector<CvPoint> *windows;
  vector<vector<double> > *fv;
};

void *calcFV(void *argP) {

  CalcFVArgs *args = (CalcFVArgs*) argP;

  // TODO: can we speed up patchValues?  This is hugely parallelizable
  vector<double> p =
    args->entries->at(args->index)->patchValues(*(args->images),
						*(args->windows));
        
  for (unsigned j = 0; j < args->windows->size(); j++) {
    args->fv->at(j)[args->index] = p[j];
  }

  delete args;

  return NULL;
}

struct IntegralImageResponseArgs {
  IntegralImageResponseArgs(const IplImage *i, const IplImage *sI,
			    const IplImage *sI2,
			    vector<svlSmartPointer<svlPatchDefinition> > *e,
			    const int eI, const vector<CvPoint> *w,
			    vector<vector<double> > *fv) :
    image(i), sumImage(sI), sumImage2(sI2), entries(e), entryIndex(eI),
    windows(w), v(fv) {}
  const IplImage *image;
  const IplImage *sumImage;
  const IplImage *sumImage2;
  vector<svlSmartPointer<svlPatchDefinition> > *entries;
  const int entryIndex;
  const vector<CvPoint> *windows;
  vector<vector<double> > *v;
};

void *calcIntegralImageResponse(void *argP) {
  IntegralImageResponseArgs *args = (IntegralImageResponseArgs*) argP;

  const int entryIndex = args->entryIndex;
  svlPatchDefinition *entry = args->entries->at(entryIndex);

  IplImage *response =
    ((svlIntensityPatchDefinition*)
     entry)->responseImage(args->image, args->sumImage, args->sumImage2);

  const vector<CvPoint> *windows = args->windows;
  for (unsigned j = 0; j < windows->size(); j++) {
    args->v->at(j)[entryIndex] = entry->patchValue(response, windows->at(j));
  }

  cvReleaseImage(&response);

  delete args;

  return NULL;
}

#else // !ENABLE_DEPTH
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
    const svlPatchDefinition *patch = ((patch_response_args_t *)args)->patch;
    IplImage **response = ((patch_response_args_t *)args)->response;

#ifdef USE_INTEGRAL_IMAGES
    if (patch->patchType() == SVL_INTENSITY_PATCH) {
      *response = ((svlIntensityPatchDefinition *)
		   patch)->responseImage(image, sumImage, sumImage2);
    } else {
        *response = patch->responseImage(image);
    }
#else
    *response = patch->responseImage(image);
#endif

    return NULL;
}
#endif // ENABLE_DEPTH
#endif // USE_THREADS


// svlPatchDictionary class --------------------------------------------------

vector<IplImage*> svlPatchDictionary::_normalizedDepthPatches;

svlPatchDictionary::svlPatchDictionary(unsigned width, unsigned height)
{
    _windowSize = cvSize(width, height);
}

svlPatchDictionary::svlPatchDictionary(const svlPatchDictionary& d) :
    _windowSize(d._windowSize), _entries(d._entries)
{
    // do nothing
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
    _entries.resize(n);
}

void svlPatchDictionary::alterValidChannel(int newChannel)
{
    for (unsigned i=0 ; i<_entries.size() ; i++)
    {
        _entries[i]->setValidChannel(newChannel);
    }
}

void svlPatchDictionary::mergeDictionary( svlPatchDictionary input )
{
    for (unsigned i=0 ; i<input.numEntries() ; i++)
    {
        _entries.push_back( input._entries[i] );
    }
}

bool svlPatchDictionary::read(const char *filename)
{
    _entries.clear();

    XMLNode root = XMLNode::parseFile(filename, "PatchDictionary");
    if (root.isEmpty()) {
        SVL_LOG(SVL_LOG_WARNING, "could not read patch dictionary " << filename);
	return false;
    }

    _windowSize.width = atoi(root.getAttribute("width"));
    _windowSize.height = atoi(root.getAttribute("height"));
    int n = atoi(root.getAttribute("numEntries"));

    assert(root.nChildNode("PatchDefinition") == n);
    _entries.resize(n, NULL);

    for (unsigned i = 0; i < _entries.size(); i++) 
	{		
        XMLNode node = root.getChildNode("PatchDefinition", i);
        _entries[i] = svlPatchDefinition::createPatchDefinition(node);
        assert(_entries[i] != NULL);
    }

    return true;
}

bool svlPatchDictionary::write(const char *filename)
{
    ofstream ofs(filename);
    if (ofs.fail()) { return false; }

    ofs << "<PatchDictionary version=\"1\"\n"
        << "  width=\"" <<  _windowSize.width << "\"\n"
        << "  height=\"" << _windowSize.height << "\"\n"
        << "  numEntries=\"" << _entries.size() << "\">\n";

    for (unsigned i = 0; i < _entries.size(); i++) {
        _entries[i]->write(ofs);
    }

    ofs << "</PatchDictionary>\n";

    ofs.close();
    return true;
}

void svlPatchDictionary::buildDictionary(const vector<vector<IplImage *> >& samples,
    const vector<svlPatchDefinitionType>& imageTypes, unsigned n)
{
    assert(imageTypes.size() == samples[0].size());

    CvRNG rngState(0xffffffff);

    int numChannels = samples[0].size();
    for (int c = 0; c < numChannels; c++) {
        for (unsigned i = 0; i < samples.size(); i++) {
            assert(samples[i].size() == (unsigned)numChannels);
            if (samples[i][c] == NULL)
                continue;
            assert((samples[i][c]->width == _windowSize.width) && 
                (samples[i][c]->height == _windowSize.height));
            
            for (unsigned j = 0; j < n; j++) {
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
                
                IplImage *t = cvCreateImage(cvSize(r.width, r.height), samples[i][c]->depth, 1);
                cvSetImageROI(samples[i][c], r);
                cvCopyImage(samples[i][c], t);
                cvResetImageROI(samples[i][c]);
		
                // patch is valid over 7x7 pixel region around source location
                r.x -= 3; r.y -=3; r.width = 7; r.height = 7;
                
                svlClipRect(r, _windowSize.width - t->width + 1, 
                    _windowSize.height - t->height + 1);

                switch (imageTypes[c]) {
                case SVL_INTENSITY_PATCH:
                    _entries.push_back(new svlIntensityPatchDefinition(t, r, c));
                    break;
                case SVL_DEPTH_PATCH:
                    _entries.push_back(new svlDepthPatchDefinition(t, r, c));
                    break;
                default:
                    SVL_LOG(SVL_LOG_FATAL, "unrecognized channel type");
                }
                
                cvReleaseImage(&t);
            } // patches
        } // images
    } // channels
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

	cvSetImageROI(allPatches, cvRect(r.x + _entries[i]->_validRect.x, r.y + _entries[i]->_validRect.y,
		_entries[i]->_template->width, _entries[i]->_template->height));
	IplImage *templateCopy = cvCloneImage(_entries[i]->_template);
	scaleToRange(templateCopy, 0.0, 255.0);
        if (templateCopy->depth != IPL_DEPTH_8U) {
            IplImage *tmpImg = cvCreateImage(cvGetSize(templateCopy), IPL_DEPTH_8U, 1);
            cvConvertScale(templateCopy, tmpImg);
            cvReleaseImage(&templateCopy);
            templateCopy = tmpImg;
        }
	for (unsigned k = 0; k < 3; k++) 
	{
	    cvSetImageCOI(allPatches, k + 1);
	    //cvCopyImage(_entries[i]->_template, allPatches);
	    cvCopyImage(templateCopy, allPatches);
	}
	cvReleaseImage(&templateCopy);
	cvSetImageCOI(allPatches, 0);
	cvResetImageROI(allPatches);

	cvRectangle(allPatches, cvPoint(r.x + _entries[i]->_validRect.x, r.y + _entries[i]->_validRect.y),
            cvPoint(r.x + _entries[i]->_validRect.x + _entries[i]->_validRect.width + _entries[i]->_template->width,
                r.y + _entries[i]->_validRect.y + _entries[i]->_validRect.height + _entries[i]->_template->height),
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
    cvSetImageROI(image, cvRect(_entries[index]->_validRect.x, _entries[index]->_validRect.y,
	    _entries[index]->_template->width, _entries[index]->_template->height));

    IplImage *templateCopy = cvCloneImage(_entries[index]->_template);
    scaleToRange(templateCopy, 0.0, 255.0);
    if (templateCopy->depth != IPL_DEPTH_8U) {
        IplImage *tmpImg = cvCreateImage(cvGetSize(templateCopy), IPL_DEPTH_8U, 1);
        cvConvertScale(templateCopy, tmpImg);
        cvReleaseImage(&templateCopy);
        templateCopy = tmpImg;
    }

    for (unsigned k = 0; k < 3; k++) {
	cvSetImageCOI(image, k + 1);
	cvCopyImage(templateCopy, image);
    }
    
    cvReleaseImage(&templateCopy);
    cvSetImageCOI(image, 0);
    cvResetImageROI(image);
    
    // draw box around valid region
    cvRectangle(image, cvPoint(_entries[index]->_validRect.x, _entries[index]->_validRect.y),
	cvPoint(_entries[index]->_validRect.x + _entries[index]->_validRect.width + _entries[index]->_template->width,
	    _entries[index]->_validRect.y + _entries[index]->_validRect.height + _entries[index]->_template->height),
	CV_RGB(255, 0, 0), 1);
    
    // indicate channel index
    for (int j = 0; j <= _entries[index]->_validChannel; j++) {
	CvPoint pt =  cvPoint(_entries[index]->_validRect.x + _entries[index]->_validRect.width + _entries[index]->_template->width,
	    _entries[index]->_validRect.y + _entries[index]->_validRect.height + _entries[index]->_template->height);
	cvLine(image, cvPoint(pt.x - 2 * (j + 1), pt.y - 2), cvPoint(pt.x - 2 * (j + 1), pt.y - 3), CV_RGB(255, 0, 0)); 
    }
    
    return image;
}

vector<double> svlPatchDictionary::patchResponse(vector<IplImage *> images)
{
    assert(!images.empty());
    for (unsigned c = 0; c < images.size(); c++) {
        assert(images[c] != NULL);
        if ((images[c]->width != _windowSize.width) ||
            (images[c]->height != _windowSize.height)) {
            // TO DO: resize
            SVL_LOG(SVL_LOG_FATAL, "invalid image size in patchResponse");
        }
    }

    vector<double> fv(_entries.size());

    for (unsigned i = 0; i < _entries.size(); i++) 
	{		
        IplImage *response = _entries[i]->responseImage(
            images[_entries[i]->_validChannel]);

		fv[i] = _entries[i]->patchValue(response, cvPoint(0, 0));
		cvReleaseImage(&response);
    }

    return fv;
}


vector<vector<double> > svlPatchDictionary::imageResponse(vector<IplImage*> images, const vector<CvPoint> &windows) {

  // allocate memory for return vectors
  vector<vector<double> > v(windows.size());

  for (unsigned i = 0; i < windows.size(); i++) 
  {
    v[i].resize(_entries.size());
  }

  int numChannels = (int)images.size();

#ifdef ENABLE_DEPTH

  svlDepthPatchDefinition::_windowSize = _windowSize;

  // Check if depth channel exists and retrieve channel index

  int depthChannel = -1;

  for (unsigned i=0; i < _entries.size(); i++) 
  {
    if( _entries[i]->patchType() == SVL_DEPTH_PATCH ) 
    {
      depthChannel = _entries[i]->validChannel();
      break;
    }
  }


  // If depth channel exists...
  if (depthChannel != -1) 
  {
    svlPatchDictionary::_normalizedDepthPatches.clear();

    //Load depth map image
    IplImage *image = images[ depthChannel ];

    // Populate normalized depth patches
#ifdef USE_THREADS
    svlPatchDictionary::_normalizedDepthPatches.resize(windows.size());
    svlThreadPool threadPool(TOTAL_THREADS);
    const int hThreadNormalization = svlCodeProfiler::getHandle("svlPatchDictionary::Normalize depth patches synchronously");
    svlCodeProfiler::tic(hThreadNormalization);
    threadPool.start();
#endif

    for (unsigned j = 0; j < windows.size(); j++) 
    {
      IplImage *patch = cvCreateImage(_windowSize, image->depth, 1);
      cvSetImageROI(image, cvRect(windows[j].x, windows[j].y, _windowSize.width, _windowSize.height));
      cvCopyImage(image, patch);
      cvResetImageROI(image);
            
      // Normalize the entire patch by subtracting the median
#ifdef USE_THREADS
      threadPool.addJob(normalizePatch, new NormalizePatchArgs(patch, j));
#else
      vector<double> depths;
            
      for(int y = 0; y < patch->height; y++) 
      {
	        for(int x = 0; x < patch->width; x++) 
            {
	            depths.push_back(CV_IMAGE_ELEM(patch, float, y, x));
	        }
      }
            
      sort(depths.begin(),depths.end());
            
      double median = depths[depths.size()/2];
            
      for(int y = 0; y < patch->height; y++) 
      {
	        for(int x = 0; x < patch->width; x++) 
            {
	            CV_IMAGE_ELEM(patch, float, y, x) = CV_IMAGE_ELEM(patch, float, y, x) - median;
	        }
      }

      depths.clear();

      svlPatchDictionary::_normalizedDepthPatches.push_back(patch);
#endif // USE_THREADS

    } // windows loop

#ifdef USE_THREADS
    // wait for finish
    threadPool.finish();
    svlCodeProfiler::toc(hThreadNormalization);
#endif
            
  } // if depth channel

#ifdef USE_THREADS

#ifdef USE_INTEGRAL_IMAGES
  // pre-compute integral images for use by patch responses
  IplImage **imageSum = new IplImage* [numChannels];
  IplImage **imageSumSq = new IplImage* [numChannels];

  const int hIntegralImages = svlCodeProfiler::getHandle("svlPatchDictionary::Precompute integral images");
  svlCodeProfiler::tic(hIntegralImages);

  for (int c = 0; c < numChannels; c++) 
  {
    if (c == depthChannel) 
    {
	imageSum[c] = NULL;
	imageSumSq[c] = NULL;
        continue; 
    }

    imageSum[c] = NULL;
    imageSumSq[c] = NULL;
    
    if (images[c]->depth == IPL_DEPTH_8U) 
    {
      imageSum[c] =	cvCreateImage(cvSize(images[c]->width + 1, images[c]->height + 1),IPL_DEPTH_32S, 1);
      imageSumSq[c] = cvCreateImage(cvSize(images[c]->width + 1, images[c]->height + 1),IPL_DEPTH_64F, 1);
      cvIntegral(images[c], imageSum[c], imageSumSq[c]);
    }
  }

  svlCodeProfiler::toc(hIntegralImages);
#endif // USE_INTEGRAL_IMAGES

  //svlThreadPool vThreadPool(_entries.size());
  svlThreadPool vThreadPool(TOTAL_THREADS);
  vThreadPool.start();

  for (unsigned i = 0; i < _entries.size(); i++) 
  {
     #ifdef USE_INTEGRAL_IMAGES
        
      if (_entries[i]->patchType() == SVL_INTENSITY_PATCH) 
      {
        IntegralImageResponseArgs *args =
	        new IntegralImageResponseArgs(images[_entries[i]->_validChannel],
				      imageSum[_entries[i]->_validChannel],
				      imageSumSq[_entries[i]->_validChannel],
				      &_entries, i, &windows, &v);

	    vThreadPool.addJob(calcIntegralImageResponse, args);
    } 
      else 
      {
      vThreadPool.addJob(calcFV, new CalcFVArgs(&_entries, i, &images,
						&windows, &v));
    }

#else
    
      vThreadPool.addJob(calcFV, new CalcFVArgs(&_entries, i, &images, &windows,
					      &v));

#endif //   USE_INTEGRAL_IMAGES

  }
  vThreadPool.finish();

#ifdef USE_INTEGRAL_IMAGES
    // release integral images
    for (int i = 0; i < numChannels; i++) 
    {
        if (imageSumSq[i] != NULL)
            cvReleaseImage(&imageSumSq[i]);
        if (imageSum[i] != NULL)
            cvReleaseImage(&imageSum[i]);
    }
    
    delete[] imageSumSq;
    delete[] imageSum;
#endif //   USE_INTEGRAL_IMAGES
//    delete[] threads;
 //   delete[] args;
  //  delete[] responses;	


#else // !USE_THREADS

  // iterate through patches
  for (unsigned i = 0; i < _entries.size(); i++) 
  {
    int handle = svlCodeProfiler::getHandle("svlPatchDefinition::patchValue");
    svlCodeProfiler::tic(handle);

    vector<double> p = _entries[i]->patchValues(images, windows);
        
    for (unsigned j = 0; j < windows.size(); j++) 
    {
      v[j][i] = p[j];
    }

    svlCodeProfiler::toc(handle);
  }

#endif // USE_THREADS

  // Free all memory from the normalized patches

  for (unsigned j = 0; j < svlPatchDictionary::_normalizedDepthPatches.size();j++) 
  {
    cvReleaseImage(&svlPatchDictionary::_normalizedDepthPatches[j]);
  }
  
  svlPatchDictionary::_normalizedDepthPatches.clear();


  //*************************************************************************
#else // !ENABLE_DEPTH

  IplImage **responses = new IplImage * [_entries.size()];
  patch_response_args_t *args = new patch_response_args_t[_entries.size()];
  pthread_t *threads = new pthread_t[_entries.size()];

#ifdef USE_INTEGRAL_IMAGES

  // pre-compute integral images for use by patch responses
  IplImage **imageSum = new IplImage* [numChannels];
  IplImage **imageSumSq = new IplImage* [numChannels];

  for (int i = 0; i < numChannels; i++) 
  {
    imageSum[i] = NULL;
    imageSumSq[i] = NULL;
    if (images[i]->depth == IPL_DEPTH_8U) {
      imageSum[i] =
	cvCreateImage(cvSize(images[i]->width + 1, images[i]->height + 1),
		      IPL_DEPTH_32S, 1);
      imageSumSq[i] =
	cvCreateImage(cvSize(images[i]->width + 1, images[i]->height + 1),
		      IPL_DEPTH_64F, 1);
      cvIntegral(images[i], imageSum[i], imageSumSq[i]);
    }
  }
#endif //   USE_INTEGRAL_IMAGES

  for (unsigned i = 0; i < _entries.size(); i++) 
  {
    args[i].image = images[_entries[i]->_validChannel];
#ifdef USE_INTEGRAL_IMAGES
    args[i].sumImage = imageSum[_entries[i]->_validChannel];
    args[i].sumImage2 = imageSumSq[_entries[i]->_validChannel];
#endif //   USE_INTEGRAL_IMAGES
    args[i].response = &responses[i];
    args[i].patch = _entries[i];

    pthread_create(&threads[i], NULL, patch_response_fcn, (void *)&args[i]);
  }

  for (unsigned i = 0; i < _entries.size(); i++) {
        pthread_join(threads[i], NULL);
        int handle = svlCodeProfiler::getHandle("svlPatchDefinition::patchValue");
        svlCodeProfiler::tic(handle);
        for (unsigned j = 0; j < windows.size(); j++) {
            v[j][i] = _entries[i]->patchValue(responses[i], windows[j]);
        }
        svlCodeProfiler::toc(handle);
        cvReleaseImage(&responses[i]);
    }

#ifdef USE_INTEGRAL_IMAGES
    // release integral images
    for (int i = 0; i < numChannels; i++) 
    {
        if (imageSumSq[i] != NULL)
            cvReleaseImage(&imageSumSq[i]);
        if (imageSum[i] != NULL)
            cvReleaseImage(&imageSum[i]);
    }
    
    delete[] imageSumSq;
    delete[] imageSum;
#endif //   USE_INTEGRAL_IMAGES
    delete[] threads;
    delete[] args;
    delete[] responses;	

#endif // ENABLE_DEPTH
        
    return v;
}
