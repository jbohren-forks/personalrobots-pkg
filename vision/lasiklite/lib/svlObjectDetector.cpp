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
** FILENAME:    svlObjectDetector.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**
*****************************************************************************/

#include <cassert>
#include <iostream>
#include <fstream>
#include <vector>
#include <sys/types.h>
#include <limits>
#include <dirent.h>

#include "svlObjectDetector.h"
#include "svlOpenCVUtils.h"

#define VERBOSE
#undef DEBUG

using namespace std;

// Statics and Globals -------------------------------------------------------

#ifndef DEBUG
bool svlObjectDetector::DEBUG_OUTPUT = false;
#else
bool svlObjectDetector::DEBUG_OUTPUT = true;
#endif

// Constructors/Destructors --------------------------------------------------

svlObjectDetector::svlObjectDetector(const char *name, int w, int h) : svlOptions(),
    _model(NULL), _windowWidth(w), _windowHeight(h), _threshold(0.5),
    _featureVector(NULL)
{
    if (name == NULL) {
	_objectName = "[unknown]";
    } else {
	_objectName = string(name);
    }

    // default training options
    declareOption("boostingRounds", "100");
    declareOption("trimRate", "0.9");
    declareOption("numSplits", "2");
    declareOption("maxTrainingImages", numeric_limits<int>::max());
    declareOption("trainOnNegSubRects", false);
    declareOption("trainOnPosSubRects", false);
}

svlObjectDetector::svlObjectDetector(const svlObjectDetector& c) :
    _model(NULL), _windowWidth(c._windowWidth), _windowHeight(c._windowHeight),
    _threshold(c._threshold), _objectName(c._objectName), _featureVector(NULL)
{
    if (c._model != NULL) {
        _model = (legacy::CvStatModel *)cvClone(c._model);
    }
}

svlObjectDetector::~svlObjectDetector()
{
    if (_featureVector != NULL) {
        cvReleaseMat(&_featureVector);
    }
    if (_model != NULL) {
	cvRelease((void **)&_model);
    }
}

// Public Member Functions -----------------------------------------------------

bool svlObjectDetector::readModel(const char *filename)
{
    if (_featureVector != NULL) {
        cvReleaseMat(&_featureVector);
        _featureVector = NULL;
    }
    if (_model != NULL) {
	cvRelease((void **)&_model);
    }
    _model = (legacy::CvStatModel *)cvLoad(filename);

    return (_model != NULL);
}

bool svlObjectDetector::writeModel(const char *filename) const
{
    assert(_model != NULL);
    cvSave(filename, _model);
    return true;
}

// evaluation functions
svlObject2dFrame svlObjectDetector::classifyImage(const IplImage *image)
{
    assert((_model != NULL) && (image != NULL));

    svlObject2dFrame objects;
    double scale = 1.0;
    float scoreThreshold = (float)(log(_threshold) - log(1.0 - _threshold));

    cerr << "Running classifier on image..." << endl;

    IplImage *imgCopy;

    if (image->nChannels == 1) {
	imgCopy = cvCloneImage(image);
    } else {
	imgCopy = cvCreateImage(cvSize(image->width, image->height), IPL_DEPTH_8U, 1);
	cvCvtColor(image, imgCopy, CV_RGB2GRAY);
    }

    IplImage *scaledImage = cvCloneImage(imgCopy);

    while ((scaledImage->width >= _windowWidth) &&
	(scaledImage->height >= _windowHeight)) {

	// create sliding window locations
	vector<CvPoint> locations;
	createWindowLocations(scaledImage->width, scaledImage->height, locations);
	
#if 1	
	for (int i = (int)locations.size() - 1; i >= 0; i--) {
	    double minVal, maxVal;
	    cvSetImageROI(scaledImage, cvRect(locations[i].x, locations[i].y, _windowWidth, _windowHeight));
	    cvMinMaxLoc(scaledImage, &minVal, &maxVal);
	    cvResetImageROI(scaledImage);
	    if (maxVal - minVal < 32.0) {
		locations.erase(locations.begin() + i);
	    }
	}
#endif
	
	// run classifier
#ifdef VERBOSE
	cerr << "Computing features over " << locations.size() << " windows for image size "
	     << scaledImage->width << " x " << scaledImage->height << "..." << endl;
#endif

	vector<vector<double> > features;
	features.reserve(locations.size());
	computeImageFeatureVectors(features, locations, scaledImage);
	
#ifdef VERBOSE
	cerr << "...done" << endl;
	cerr << "Running classifier on " << locations.size() << " windows..." << endl;
#endif
	if (_featureVector == NULL) {
	    _featureVector = cvCreateMat(1, ((legacy::CvBtClassifier *)_model)->total_features, CV_32F);
	}
	
	assert(_featureVector->cols == ((legacy::CvBtClassifier *)_model)->total_features);
	
	for (unsigned i = 0; i < locations.size(); i++) {
	    for (unsigned j = 0; j < (unsigned)_featureVector->cols; j++) {
		cvmSet(_featureVector, 0, j, features[i][j]);
	    }

	    float score = legacy::cvEvalWeakClassifiers((legacy::CvBtClassifier *)_model,
		_featureVector, NULL);
	    if (score > scoreThreshold) {
		objects.push_back(svlObject2d(scale * locations[i].x, scale * locations[i].y,
			scale * _windowWidth, scale * _windowHeight));
		objects.back().name = _objectName;
		objects.back().pr = 1.0 / (1.0 + exp(-score));
#ifdef VERBOSE
		//cerr << "...found " << objects.back() << " with probability " << objects.back().pr << endl;
#endif
	    }
	}
#ifdef VERBOSE
	cerr << "...done" << endl;
#endif

    	// scale down image
	scale *= 1.2;
	
	cvReleaseImage(&scaledImage);
	scaledImage = cvCreateImage(cvSize((int)(imgCopy->width / scale), (int)(imgCopy->height / scale)),
	    IPL_DEPTH_8U, 1);
	cvResize(imgCopy, scaledImage);
    }
    
    cvReleaseImage(&scaledImage);
    cvReleaseImage(&imgCopy);
    
    return objects;
}

svlObject2dFrame svlObjectDetector::classifyImage(IplImage *image,
    const vector<CvPoint>& locations)
{
    assert((_model != NULL) && (image != NULL));

    svlObject2dFrame objects;
    float scoreThreshold = (float)(log(_threshold) - log(1.0 - _threshold));

    IplImage *imgCopy;
    if (image->nChannels == 1) {
	imgCopy = image;
    } else {
	imgCopy = cvCreateImage(cvSize(image->width, image->height), IPL_DEPTH_8U, 1);
	cvCvtColor(image, imgCopy, CV_RGB2GRAY);
    }

    // run classifier
#ifdef VERBOSE
    cerr << "Computing features over " << locations.size() << " windows for image size "
	 << imgCopy->width << " x " << imgCopy->height << "..." << endl;
#endif
    vector<vector<double> > features;
    features.reserve(locations.size());
    computeImageFeatureVectors(features, locations, imgCopy);
#ifdef VERBOSE
    cerr << "...done" << endl;
    cerr << "Running classifier on " << locations.size() << " windows..." << endl;
#endif
    if (_featureVector == NULL) {
	_featureVector = cvCreateMat(1, ((legacy::CvBtClassifier *)_model)->total_features, CV_32F);
    }
    assert(_featureVector->cols == ((legacy::CvBtClassifier *)_model)->total_features);
    
    for (unsigned i = 0; i < locations.size(); i++) {
	for (unsigned j = 0; j < (unsigned)_featureVector->cols; j++) {
	    cvmSet(_featureVector, 0, j, features[i][j]);
	}
	float score = legacy::cvEvalWeakClassifiers((legacy::CvBtClassifier *)_model,
	    _featureVector, NULL);
	if (score > scoreThreshold) {
	    objects.push_back(svlObject2d(locations[i].x, locations[i].y,
		    _windowWidth,  _windowHeight));
	    objects.back().name = _objectName;
	    objects.back().pr = 1.0 / (1.0 + exp(-score));
#ifdef VERBOSE
	    //cerr << "...found " << objects.back() << " with probability " << objects.back().pr << endl;
#endif
	}
    }
#ifdef VERBOSE
    cerr << "...done" << endl;
#endif

    // free memory if we allocated any
    if (image->nChannels != 1) {	
	cvReleaseImage(&imgCopy);
    }

    return objects;
}

svlObject2dFrame svlObjectDetector::classifyImage(IplImage *image,
    const map<double, vector<CvPoint> >& locations)
{
    assert((_model != NULL) && (image != NULL));

    svlObject2dFrame objects;

    cerr << "Running classifier on image..." << endl;

    IplImage *imgCopy;
    if (image->nChannels == 1) {
	imgCopy = cvCloneImage(image);
    } else {
	imgCopy = cvCreateImage(cvSize(image->width, image->height), IPL_DEPTH_8U, 1);
	cvCvtColor(image, imgCopy, CV_RGB2GRAY);
    }

    IplImage *scaledImage;
    for (map<double, vector<CvPoint> >::const_iterator it = locations.begin();
	 it != locations.end(); it++) {

	double scale = it->first;
	scaledImage = cvCreateImage(cvSize((int)(imgCopy->width / scale), (int)(imgCopy->height / scale)),
	    IPL_DEPTH_8U, 1);
	cvResize(imgCopy, scaledImage);

	if ((scaledImage->width < _windowWidth) || (scaledImage->height < _windowHeight)) {
	    cvReleaseImage(&scaledImage);
	    cerr << "WARNING: scaled makes image smaller than window size" << endl;
	    continue;
	}

	svlObject2dFrame o = classifyImage(scaledImage, it->second);
	for (svlObject2dFrame::const_iterator jt = o.begin(); jt != o.end(); jt++) {	    
	    objects.push_back(*jt);
	    objects.back().scale(scale);
	}
	cvReleaseImage(&scaledImage);
    }

    // free memory if we allocated any
    if (image->nChannels != 1) {	
	cvReleaseImage(&imgCopy);
    }
    
    return objects;
}

svlObject2dFrame svlObjectDetector::classifyRegion(const IplImage *image, CvRect region)
{
    assert(image != NULL);
    svlObject2dFrame objects;

    if ((region.x == 0) && (region.width == image->width) &&
	(region.y == 0) && (region.height == image->height)) {
	return classifyImage(image);
    }

    IplImage *imgCopy = cvCloneImage(image);
    assert(imgCopy != NULL);

    svlClipRect(region, image);
    IplImage *subImage = cvCreateImage(cvSize(region.width, region.height),
	image->depth, image->nChannels);
    assert(subImage != NULL);
    cvSetImageROI(imgCopy, region);
    cvCopyImage(imgCopy, subImage);
    cvReleaseImage(&imgCopy);

    objects = classifyImage(subImage);

    for (unsigned i = 0; i < objects.size(); i++) {
	objects[i].x += region.x;
	objects[i].y += region.y;
    }

    cvReleaseImage(&subImage);

    return objects;
}

// evaluate all image locations
void svlObjectDetector::evaluateWindowLocations(IplImage *image,
    const map<double, vector<CvPoint> >& locations, map<double, vector<float> >& scores)
{
    assert((_model != NULL) && (image != NULL));

    cerr << "Evaluating classifier on image..." << endl;

    IplImage *imgCopy;
    if (image->nChannels == 1) {
	imgCopy = cvCloneImage(image);
    } else {
	imgCopy = cvCreateImage(cvSize(image->width, image->height), IPL_DEPTH_8U, 1);
	cvCvtColor(image, imgCopy, CV_RGB2GRAY);
    }

    IplImage *scaledImage;
    for (map<double, vector<CvPoint> >::const_iterator it = locations.begin();
	 it != locations.end(); it++) {

	double scale = it->first;
	scores[scale] = vector<float>(it->second.size(), 0.0);

	scaledImage = cvCreateImage(cvSize((int)(imgCopy->width / scale), (int)(imgCopy->height / scale)),
	    IPL_DEPTH_8U, 1);
	cvResize(imgCopy, scaledImage);

	if ((scaledImage->width < _windowWidth) || (scaledImage->height < _windowHeight)) {
	    cvReleaseImage(&scaledImage);
	    cerr << "WARNING: scaled makes image smaller than window size" << endl;
	    continue;
	}

	// run classifier
#ifdef VERBOSE
	cerr << "Computing features over " << it->second.size() << " windows for image size "
	     << scaledImage->width << " x " << scaledImage->height << "..." << endl;
#endif
	vector<vector<double> > features;
	features.reserve(it->second.size());
	computeImageFeatureVectors(features, it->second, scaledImage);
#ifdef VERBOSE
	cerr << "...done" << endl;
	cerr << "Running classifier on " << it->second.size() << " windows..." << endl;
#endif
	if (_featureVector == NULL) {
	    _featureVector = cvCreateMat(1, ((legacy::CvBtClassifier *)_model)->total_features, CV_32F);
	}
	assert(_featureVector->cols == ((legacy::CvBtClassifier *)_model)->total_features);
    
	for (unsigned i = 0; i < it->second.size(); i++) {
	    for (unsigned j = 0; j < (unsigned)_featureVector->cols; j++) {
		cvmSet(_featureVector, 0, j, features[i][j]);
	    }
	    
	    scores[scale][i] = legacy::cvEvalWeakClassifiers((legacy::CvBtClassifier *)_model,
		_featureVector, NULL);
	}
#ifdef VERBOSE
	cerr << "...done" << endl;
#endif
	cvReleaseImage(&scaledImage);
    }

    // free memory if we allocated any
    if (image->nChannels != 1) {	
	cvReleaseImage(&imgCopy);
    }
}

// produce response map for detector
CvMat *svlObjectDetector::responseMap(const IplImage *image)
{
    assert(image != NULL);
    CvMat *response = cvCreateMat(image->height, image->width, CV_32FC1);
    cvZero(response);

    if ((image->width < _windowWidth) || (image->height < _windowHeight))
	return response;

    IplImage *imgCopy;
    if (image->nChannels == 1) {
	imgCopy = cvCloneImage(image);
    } else {
	imgCopy = cvCreateImage(cvSize(image->width, image->height), IPL_DEPTH_8U, 1);
	cvCvtColor(image, imgCopy, CV_RGB2GRAY);
    }

    // create sliding window locations
    vector<CvPoint> locations;
    createWindowLocations(imgCopy->width, imgCopy->height, locations);
	
    // run classifier
#ifdef VERBOSE
    cerr << "Computing features over " << locations.size() << " windows for image size "
	 << imgCopy->width << " x " << imgCopy->height << "..." << endl;
#endif
    vector<vector<double> > features;
    features.reserve(locations.size());
    computeImageFeatureVectors(features, locations, imgCopy);
	
#ifdef VERBOSE
    cerr << "...done" << endl;
    cerr << "Running classifier on " << locations.size() << " windows..." << endl;
#endif
    if (_featureVector == NULL) {
	_featureVector = cvCreateMat(1, ((legacy::CvBtClassifier *)_model)->total_features, CV_32F);
    }
    assert(_featureVector->cols == ((legacy::CvBtClassifier *)_model)->total_features);
	
    for (unsigned i = 0; i < locations.size(); i++) {
	for (unsigned j = 0; j < (unsigned)_featureVector->cols; j++) {
	    cvmSet(_featureVector, 0, j, features[i][j]);
	}
	float score = legacy::cvEvalWeakClassifiers((legacy::CvBtClassifier *)_model,
	    _featureVector, NULL);
	CV_MAT_ELEM(*response, float, locations[i].y + _windowHeight / 2, 
	    locations[i].x + _windowWidth / 2) = score;
    }
#ifdef VERBOSE
    cerr << "...done" << endl;
#endif

    cvReleaseImage(&imgCopy);

    return response;
}

// create sliding window locations
void svlObjectDetector::createWindowLocations(int width, int height,
    vector<CvPoint>& locations) const
{
    const int STEP_SIZE = 4;

    int maxX = width - _windowWidth;
    int maxY = height - _windowHeight;

    if ((maxX <= 0) || (maxY <= 0))
	return;

    locations.reserve((maxX + STEP_SIZE) * (maxY + STEP_SIZE) / (STEP_SIZE * STEP_SIZE));

    for (int x = 0; x <= maxX; x += STEP_SIZE) 
	{
	for (int y = 0; y <= maxY; y += STEP_SIZE) 
	{
	    locations.push_back(cvPoint(x, y));
	}
    }

}

void svlObjectDetector::createAllWindowLocations(int width, int height,
    map<double, vector<CvPoint> >& locations) const
{
    double scale = 1.0;
    int w = width;
    int h = height;
    locations.clear();
    while ((w >= _windowWidth) && (h >= _windowHeight)) {
	createWindowLocations(w, h, locations[scale]);
	scale *= 1.2;
	w = (int)(width / scale);
	h = (int)(height / scale);
    }    
}

// learning functions
void svlObjectDetector::learnModel(const vector<vector<double> >& posSamples, const vector<vector<double> >& negSamples)
{
    if (_featureVector != NULL) 
	{
        cvReleaseMat(&_featureVector);
        _featureVector = NULL;
    }

    if (_model != NULL)
	{
	cvRelease((void **)&_model);	
    }

    cerr << "Training with " << posSamples.size() << " positive and "
	    << negSamples.size() << " negative samples" << endl;
    cerr << "Feature vector size is " << posSamples[0].size() << endl;

    // assemble design matrix for training
    CvMat *data = cvCreateMat((int)(posSamples.size() + negSamples.size()),
        (int)posSamples[0].size(), CV_32FC1);
    CvMat *labels = cvCreateMat((int)(posSamples.size() + negSamples.size()), 1, CV_32SC1);
    assert((data != NULL) && (labels != NULL));

    for (unsigned i = 0; i < posSamples.size(); i++) {
	    assert(posSamples[i].size() == (unsigned)data->width);
	    for (unsigned j = 0; j < (unsigned)data->width; j++) {
	        cvmSet(data, i, j, posSamples[i][j]);
	    }
	    CV_MAT_ELEM(*labels, int, i, 0) =  1;
    }

    for (unsigned i = 0; i < negSamples.size(); i++) {
	    assert(negSamples[i].size() == (unsigned)data->width);
	    for (unsigned j = 0; j < (unsigned)data->width; j++) {
	        cvmSet(data, i + posSamples.size(), j, negSamples[i][j]);
	    }
	    CV_MAT_ELEM(*labels, int, i + posSamples.size(), 0) = -1;
    }

    // train the classifier
    legacy::CvBtClassifierTrainParams parameters;
    memset(&parameters, 0, sizeof(legacy::CvBtClassifierTrainParams));
    parameters.boost_type = CV_BT_GENTLE;
    parameters.num_iter = getOptionAsInt("boostingRounds");
    parameters.infl_trim_rate = getOptionAsDouble("trimRate");
    parameters.num_splits = getOptionAsInt("numSplits");

    _model = legacy::cvCreateBtClassifier(data, CV_ROW_SAMPLE, labels, (legacy::CvStatModelParams*)&parameters);
    icvReleaseBoostTrainStateMembers((legacy::CvBoostTrainState *)((legacy::CvBtClassifier*)_model)->ts);

    // evaluate model (on training data)
    int tp = 0;
    int tn = 0;
    int fp = 0;
    int fn = 0;
    for (unsigned i = 0; i < posSamples.size(); i++) {
        if (evaluateModel(posSamples[i]) > 0.0) {
            tp += 1;
        } else {
            fn += 1;
        }
    }
    for (unsigned i = 0; i < negSamples.size(); i++) {
        if (evaluateModel(negSamples[i]) > 0.0) {
            fp += 1;
        } else {
            tn += 1;
        }
    }
    cerr << "Evaluation (TP, FN, TN, FP): " << tp << ", " << fn << ", " << tn << ", " << fp << endl;

    // release memory
    cvReleaseMat(&labels);
    cvReleaseMat(&data);
}

void svlObjectDetector::learnModel(const vector<IplImage *>& posSamples,
    const vector<IplImage *>& negSamples)
{
    cerr << "NOT IMPLEMENTED YET" << endl;
    assert(false);
}

void svlObjectDetector::learnModel(const char *posDirectory, const char *negDirectory, const char *ext)
{
    vector<vector<double> > posSamples;
    vector<vector<double> > negSamples;
    unsigned maxTrainingImages = (unsigned)getOptionAsInt("maxTrainingImages");

    // read, resize and process positive images 
    bool bAddSubRects = getOptionAsBool("trainOnPosSubRects");
    DIR *dir = opendir(posDirectory);
    if (dir == NULL) {
	cerr << "ERROR: could not open image directory " << posDirectory << endl;
	exit(-1);
    }

    struct dirent *e = readdir(dir);
    while (e != NULL) {
	if (strstr(e->d_name, ext) != NULL) {
	    cerr << "Processing " << e->d_name << "..." << endl;
    	    string imageFilename = string(posDirectory) + string("/") + string(e->d_name);
	    IplImage *img = cvLoadImage(imageFilename.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
    	    if (img == NULL) {
		cerr << "...failed" << endl;
	    } else {
		// add subrectangles to negative images
		if (bAddSubRects && (img->width > 2 * _windowWidth) && (img->height > 2 * _windowHeight)) {
		    cerr << "Processing subimages..." << endl;
		    IplImage *subImage = cvCreateImage(cvSize(_windowWidth, _windowHeight),
			IPL_DEPTH_8U, 1);
		    for (int y = 0; y < img->height - _windowHeight; y += _windowHeight / 2) {
			for (int x = 0; x < img->width - _windowWidth; x += _windowWidth / 2) {
			    cvSetImageROI(img, cvRect(x, y, _windowWidth, _windowHeight));
			    cvCopyImage(img, subImage);
			    negSamples.push_back(vector<double>());
			    computePatchFeatureVector(negSamples.back(), subImage);
			}
		    }
		    cvResetImageROI(img);
		    cvReleaseImage(&subImage);
		}
		// resize image
		if ((img->width != _windowWidth) || (img->height != _windowHeight)) {
		    IplImage *resizedImage = cvCreateImage(cvSize(_windowWidth, _windowHeight),
			IPL_DEPTH_8U, 1);
		    cvResize(img, resizedImage);
		    cvReleaseImage(&img);
		    img = resizedImage;
		}
		// process image
		posSamples.push_back(vector<double>());
		computePatchFeatureVector(posSamples.back(), img);
		
    	    	// free memory
		cvReleaseImage(&img);
	    }
	}
	if (posSamples.size() >= maxTrainingImages) {
	    break;
	}
	e = readdir(dir);	
    }
    closedir(dir);

    // read, resize and process negative images
    bAddSubRects = getOptionAsBool("trainOnNegSubRects");
    dir = opendir(negDirectory);
    if (dir == NULL) {
	cerr << "ERROR: could not open image directory " << negDirectory << endl;
	exit(-1);
    }
    
    e = readdir(dir);
    while (e != NULL) {
	if (strstr(e->d_name, ext) != NULL) {
	    cerr << "Processing " << e->d_name << "..." << endl;
    	    string imageFilename = string(negDirectory) + string("/") + string(e->d_name);
	    IplImage *img = cvLoadImage(imageFilename.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
    	    if (img == NULL) {
		cerr << "...failed" << endl;
	    } else {
		// add subrectangles
		if (bAddSubRects) {
		    cerr << "Processing subimages..." << endl;
		    IplImage *subImage = cvCreateImage(cvSize(_windowWidth, _windowHeight),
			IPL_DEPTH_8U, 1);
		    for (int y = 0; y < img->height - _windowHeight; y += _windowHeight / 2) {
			for (int x = 0; x < img->width - _windowWidth; x += _windowWidth / 2) {
			    cvSetImageROI(img, cvRect(x, y, _windowWidth, _windowHeight));
			    cvCopyImage(img, subImage);
			    negSamples.push_back(vector<double>());
			    computePatchFeatureVector(negSamples.back(), subImage);
			}
		    }
		    cvResetImageROI(img);
		    cvReleaseImage(&subImage);
		}
		// resize image
		if ((img->width != _windowWidth) || (img->height != _windowHeight)) {
		    IplImage *resizedImage = cvCreateImage(cvSize(_windowWidth, _windowHeight),
			IPL_DEPTH_8U, 1);
		    cvResize(img, resizedImage);
		    cvReleaseImage(&img);
		    img = resizedImage;
		}
		// process image
		negSamples.push_back(vector<double>());
		computePatchFeatureVector(negSamples.back(), img);
		
    	    	// free memory
		cvReleaseImage(&img);
	    }
	}
	if (negSamples.size() >= maxTrainingImages) {
	    break;
	}
	e = readdir(dir);
    }
    closedir(dir);
    
    // learn the model
    learnModel(posSamples, negSamples);
}

float svlObjectDetector::evaluateModel(const vector<double>& fv)
{	
    assert(_model != NULL);
    if (_featureVector == NULL) {
	_featureVector = cvCreateMat(1, ((legacy::CvBtClassifier *)_model)->total_features, CV_32F);
    }
    assert(_featureVector->cols == ((legacy::CvBtClassifier *)_model)->total_features);

    for (int j = 0; j < _featureVector->cols; j++) {
    	cvmSet(_featureVector, 0, j, fv[j]);
    }

    return legacy::cvEvalWeakClassifiers((legacy::CvBtClassifier *)_model,
	_featureVector, NULL);
}

float svlObjectDetector::evaluateMaskedModel(IplImage *image, CvRect region)
{
    if (_featureVector == NULL) {
	_featureVector = cvCreateMat(1, ((legacy::CvBtClassifier *)_model)->total_features, CV_32F);
    }
    vector<double> fv;
    computePatchFeatureVector(fv, image);
    
    // TO DO [SG]: Sid, why is this being deleted here?
    cvReleaseImage(&image);
	
    for (int j = 0; j < _featureVector->cols; j++) {
    	cvmSet(_featureVector, 0, j, fv[j]);
    }
	
    return legacy::cvEvalWeakClassifiers((legacy::CvBtClassifier *)_model,
	_featureVector, NULL);
}


float svlObjectDetector::evaluateModel(IplImage *image, CvRect region)
{
    assert(_model != NULL);

    if (_featureVector == NULL) {
	_featureVector = cvCreateMat(1, ((legacy::CvBtClassifier *)_model)->total_features, CV_32F);
    }
    
    assert(_featureVector->cols == ((legacy::CvBtClassifier *)_model)->total_features);

    IplImage *subImage;

    if ((region.width == 0) || (region.height == 0)) 
	{
		subImage = cvCloneImage(image);
    } 
	else 
	{
	svlClipRect(region, image);
	subImage = cvCreateImage(cvSize(region.width, region.height), image->depth, image->nChannels);
	cvSetImageROI(image, region);
	cvCopyImage(image, subImage);
	cvResetImageROI(image);
    }

    if (subImage->nChannels != 1) 
	{
	IplImage *tmpImage = cvCreateImage(cvSize(subImage->width, subImage->height), IPL_DEPTH_8U, 1);
	cvCvtColor(subImage, tmpImage, CV_BGR2GRAY);
	cvReleaseImage(&subImage);
	subImage = tmpImage;
    }

    if ((subImage->width != _windowWidth) || (subImage->height != _windowHeight)) 
	{
	IplImage *tmpImage = cvCreateImage(cvSize(_windowWidth, _windowHeight), IPL_DEPTH_8U, 1);
	cvResize(subImage, tmpImage);
	cvReleaseImage(&subImage);
	subImage = tmpImage;
    }



    vector<double> fv;
    computePatchFeatureVector(fv, subImage);
    cvReleaseImage(&subImage);

    for (int j = 0; j < _featureVector->cols; j++) 
	{
    	cvmSet(_featureVector, 0, j, fv[j]);
    }

    return legacy::cvEvalWeakClassifiers((legacy::CvBtClassifier *)_model,
	_featureVector, NULL);
}
