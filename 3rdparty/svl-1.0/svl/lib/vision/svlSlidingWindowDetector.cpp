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
** FILENAME:    svlSlidingWindowDetector.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**              Sid Batra <sidbatra@stanford.edu>
**
*****************************************************************************/

#include <cassert>
#include <iostream>
#include <fstream>
#include <vector>
#include <sys/types.h>
#include <limits>

#include "svlBase.h"
#include "svlVision.h"

#if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
#define _CRT_SECURE_NO_DEPRECATE
#include "win32/dirent.h"
#else
#include <dirent.h>
#endif

#define VERBOSE
#undef DEBUG

using namespace std;

// Statics and Globals -------------------------------------------------------

#ifndef DEBUG
bool svlSlidingWindowDetector::DEBUG_OUTPUT = false;
#else
bool svlSlidingWindowDetector::DEBUG_OUTPUT = true;
#endif

int svlSlidingWindowDetector::DELTA_X = 4;
int svlSlidingWindowDetector::DELTA_Y = 4;
double svlSlidingWindowDetector::DELTA_SCALE = 1.2;

// Constructors/Destructors --------------------------------------------------

svlSlidingWindowDetector::svlSlidingWindowDetector(const char *name, int w, int h) :
    svlOptions(), _model(NULL), 
    _windowWidth(w), _windowHeight(h), _threshold(0.5),
    _featureVector(NULL), _weakResponses(NULL)
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
}

svlSlidingWindowDetector::svlSlidingWindowDetector(const svlSlidingWindowDetector& c) :
    svlOptions(c), _model(c._model),
    _windowWidth(c._windowWidth), _windowHeight(c._windowHeight), _objectName(c._objectName),
    _threshold(c._threshold), _featureVector(NULL), _weakResponses(NULL)
{
    // do nothing
}

svlSlidingWindowDetector::~svlSlidingWindowDetector()
{
    if (_featureVector != NULL) {
        cvReleaseMat(&_featureVector);
    }

    if (_weakResponses != NULL) {
        cvReleaseMat(&_weakResponses);
    }
}

// Public Member Functions --------------------------------------------------

bool svlSlidingWindowDetector::readModel(const char *filename)
{
    assert(filename != NULL);

    if (_featureVector != NULL) {
        cvReleaseMat(&_featureVector);
        _featureVector = NULL;
    }
    if (_weakResponses != NULL) {
        cvReleaseMat(&_weakResponses);
        _weakResponses = NULL;
    }

    _model = svlSmartPointer<CvBoost>(new CvBoost());
    _model->load(filename);

    return (_model != NULL);
}

bool svlSlidingWindowDetector::writeModel(const char *filename)
{
    assert((_model != NULL) && (filename != NULL));
    ((CvBoost *)_model)->save(filename);
    return true;
}

// Classifies a multi-channel image.
svlObject2dFrame svlSlidingWindowDetector::classifyImage(const vector<IplImage *>& images)
{
    assert((_model != NULL) && (!images.empty()));
    // each image in the vector must not be NULL and all images must be the same size
    for (unsigned i = 0; i < images.size(); i++) 
	{
        assert(images[i] != NULL);
        assert(images[i]->width == images[0]->width);
        assert(images[i]->height == images[0]->height);
    }
	
    svlObject2dFrame objects;
    double scale = 1.0;
    float scoreThreshold = probabilityToScore((float)_threshold);
    int numChannels = (int)images.size();

    SVL_LOG(SVL_LOG_MESSAGE, "Running classifier on " << numChannels << "-channel image");

    // copy of original images for color conversion and rescaling
    vector <IplImage *> originalImages(numChannels);
    vector <IplImage *> scaledImages(numChannels);

    for (int i = 0; i < numChannels; i++) 
	{
        // convert all color (3-channel) images to greyscale
        if ((images[i]->depth == IPL_DEPTH_8U) && (images[i]->nChannels == 3)) {
            originalImages[i] = cvCreateImage(cvGetSize(images[i]), IPL_DEPTH_8U, 1);
            cvCvtColor(images[i], originalImages[i], CV_RGB2GRAY);            
        } else {
            originalImages[i] = cvCloneImage(images[i]);
        }
        
        // initial scale is full image
        scaledImages[i] = cvCloneImage(originalImages[i]);
    }
    
    // iterate over all scales
    while ((scaledImages[0]->width >= _windowWidth) && (scaledImages[0]->height >= _windowHeight)) 
	{

        // iterate over all locations
	vector<CvPoint> locations;
	createWindowLocations(scaledImages[0]->width, scaledImages[0]->height, locations);
	
#if 1
        // check variation within region
	for (int i = (int)locations.size() - 1; i >= 0; i--) 
	{
	    double minVal, maxVal;
            cvSetImageROI(scaledImages[0], cvRect(locations[i].x, locations[i].y, _windowWidth, _windowHeight));
	    cvMinMaxLoc(scaledImages[0], &minVal, &maxVal);
	    cvResetImageROI(scaledImages[0]);
	    if (maxVal - minVal < 32.0) {
                locations.erase(locations.begin() + i);
	    }
	}
#endif
	
	// run classifier
        SVL_LOG(SVL_LOG_VERBOSE, "Computing features over " << locations.size() << " windows for image size "
            << scaledImages[0]->width << " x " << scaledImages[0]->height << "...");

	vector<vector<double> > features;
	features.reserve(locations.size());
	computeImageFeatureVectors(features, locations, scaledImages);
		
	if (DEBUG_OUTPUT) {
	    string filename = string("svlSlidingWindowDetector.classifyImage.")
		+ toString(scale) + string(".txt");
	    ofstream ofs(filename.c_str(), ios::app);
	    for (unsigned i = 0; i < features.size(); i++) {
		ofs << (scale * locations[i].x) << " " << (scale * locations[i].y) << " "
		    << (scale * _windowWidth) << " " << (scale * _windowHeight);
		for (unsigned j = 0; j < features[i].size(); j++) {
		    ofs << " " << features[i][j];
		}
		ofs << "\n";
	    }
	    ofs.close();
	}

        SVL_LOG(SVL_LOG_VERBOSE, "...done");
        SVL_LOG(SVL_LOG_VERBOSE, "Running multi-channel classifier on " << locations.size() << " windows...");

	for (unsigned i = 0; i < locations.size(); i++) {
	    float score = evaluateModel(features[i]);
	    if (score > scoreThreshold) {
		objects.push_back(svlObject2d(scale * locations[i].x, scale * locations[i].y,
                        scale * _windowWidth, scale * _windowHeight));
		objects.back().name = _objectName;
		objects.back().pr = scoreToProbability(score);
	    }
	}

	SVL_LOG(SVL_LOG_VERBOSE, "...done");

        // scale down each image channel
	scale *= this->DELTA_SCALE;
	
        for (int i = 0; i < numChannels; i++) {
            cvReleaseImage(&(scaledImages[i]));
            scaledImages[i] = cvCreateImage(cvSize((int)(originalImages[i]->width / scale),
                    (int)(originalImages[i]->height / scale)), originalImages[i]->depth, 1);
            cvResize(originalImages[i], scaledImages[i]);
        }
    }
    
    // free memory
    for (int i = 0; i < numChannels ; i++) {
        cvReleaseImage(&(scaledImages[i]));
        cvReleaseImage(&(originalImages[i]));
    }
    
    return objects;
}

svlObject2dFrame svlSlidingWindowDetector::classifyImage(const vector<IplImage *>& images,
    const vector<CvPoint>& locations)
{
    assert((_model != NULL) && (!images.empty()));
    // each image in the vector must not be NULL and all images must be the same size
    for (unsigned i = 0; i < images.size(); i++) {
        assert(images[i] != NULL);
        assert(images[i]->width == images[0]->width);
        assert(images[i]->height == images[0]->height);
    }

    svlObject2dFrame objects;
    float scoreThreshold = probabilityToScore((float)_threshold);
    int numChannels = (int)images.size();

    // copy of original images for color conversion
    vector <IplImage *> originalImages(numChannels);
    for (int i = 0; i < numChannels; i++) 
	{
        // convert all color (3-channel) images to greyscale
        if ((images[i]->depth == IPL_DEPTH_8U) && (images[i]->nChannels == 3)) {
            originalImages[i] = cvCreateImage(cvGetSize(images[i]), IPL_DEPTH_8U, 1);
            cvCvtColor(images[i], originalImages[i], CV_RGB2GRAY);            
        } else {
            originalImages[i] = cvCloneImage(images[i]);
        }
    }

    // run classifier
    SVL_LOG(SVL_LOG_VERBOSE, "Computing features over " << locations.size() << " windows for image size "
        << originalImages[0]->width << " x " << originalImages[0]->height << "...");
    
    vector<vector<double> > features;
    features.reserve(locations.size());
	
    computeImageFeatureVectors(features, locations, originalImages);
	
    
    SVL_LOG(SVL_LOG_VERBOSE, "...done");
    SVL_LOG(SVL_LOG_VERBOSE, "Running multi-channel classifier on " << locations.size() << " windows...");

    for (unsigned i = 0; i < locations.size(); i++) 
	{
        float score = evaluateModel(features[i]);
        if (score > scoreThreshold) {
            objects.push_back(svlObject2d(locations[i].x, locations[i].y,
                    _windowWidth, _windowHeight));
            objects.back().name = _objectName;
            objects.back().pr = scoreToProbability(score);
        }
    }
    
    SVL_LOG(SVL_LOG_VERBOSE, "...done");
    
    // free memory
    for (int i = 0; i < numChannels; i++) {
        cvReleaseImage(&(originalImages[i]));
    }

    return objects;
}

// evaluate classifier on given region (and subwindows)
svlObject2dFrame svlSlidingWindowDetector::classifySubImage(const vector<IplImage *>& images,
    CvRect region)
{
    assert((_model != NULL) && (!images.empty()));
    // each image in the vector must not be NULL and all images must be the same size
    for (unsigned i = 0; i < images.size(); i++) {
        assert(images[i] != NULL);
        assert(images[i]->width == images[0]->width);
        assert(images[i]->height == images[0]->height);
    }

    // if entire image, run full detector
    if ((region.x == 0) && (region.width == images[0]->width) &&
	(region.y == 0) && (region.height == images[0]->height)) {
	return classifyImage(images);
    }

    int numChannels = (int)images.size();

    // create subimages
    vector <IplImage *> subImages(numChannels);
    svlClipRect(region, images[0]);
    for (int i = 0; i < numChannels; i++) {
        IplImage *imgCopy = cvCloneImage(images[i]);

        subImages[i] = cvCreateImage(cvSize(region.width, region.height),
            images[i]->depth, images[i]->nChannels);
        assert(subImages[i] != NULL);
        
        cvSetImageROI(imgCopy, region);
        cvCopyImage(imgCopy, subImages[i]);
        cvReleaseImage(&imgCopy);
    }
    
    // run full detector on sub-images
    svlObject2dFrame objects = classifyImage(subImages);

    for (unsigned i = 0; i < objects.size(); i++) {
	objects[i].x += region.x;
	objects[i].y += region.y;
    }

    // free memory
    for (int i = 0; i < numChannels; i++) {
        cvReleaseImage(&subImages[i]);
    }

    return objects;
}

// create sliding window locations for given scale ----------------------------

void svlSlidingWindowDetector::createWindowLocations(int width, int height,
    vector<CvPoint>& locations) const
{
    int maxX = width - _windowWidth;
    int maxY = height - _windowHeight;

    if ((maxX <= 0) || (maxY <= 0))
	return;

    locations.reserve((maxX + DELTA_X) * (maxY + DELTA_Y) / (DELTA_X * DELTA_Y));
    for (int x = 0; x <= maxX; x += DELTA_X) {
	for (int y = 0; y <= maxY; y += DELTA_Y) {
	    locations.push_back(cvPoint(x, y));
	}
    }
}

void svlSlidingWindowDetector::createAllWindowLocations(int width, int height,
    map<double, vector<CvPoint> >& locations) const
{
    double scale = 1.0;
    int w = width;
    int h = height;
    locations.clear();
    while ((w >= _windowWidth) && (h >= _windowHeight)) {
	createWindowLocations(w, h, locations[scale]);
	scale *= DELTA_SCALE;
	w = (int)(width / scale);
	h = (int)(height / scale);
    }    
}

// learning functions -------------------------------------------------------

void svlSlidingWindowDetector::learnModel(const vector<vector<double> >& posSamples,
    const vector<vector<double> >& negSamples)
{
    if (_featureVector != NULL) {
        cvReleaseMat(&_featureVector);
        _featureVector = NULL;
    }
    if (_weakResponses != NULL) {
        cvReleaseMat(&_weakResponses);
        _weakResponses = NULL;
    }

    SVL_LOG(SVL_LOG_MESSAGE, "Training with " << posSamples.size() << " positive and "
        << negSamples.size() << " negative samples");
    SVL_LOG(SVL_LOG_MESSAGE, "Feature vector size is " << posSamples[0].size());

#ifdef DEBUG
    ofstream ofs("svlObjectDetector.learnModel.txt");
    for (unsigned i = 0; i < posSamples.size(); i++) {
	for (unsigned j = 0; j < posSamples[i].size(); j++) {
	    ofs << posSamples[i][j] << " ";
	}
	ofs << 1 << endl;
    }
    for (unsigned i = 0; i < negSamples.size(); i++) {
	for (unsigned j = 0; j < negSamples[i].size(); j++) {
	    ofs << negSamples[i][j] << " ";
	}
	ofs << 0 << endl;
    }    
    ofs.close();
#endif

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
    CvBoostParams parameters(CvBoost::GENTLE, getOptionAsInt("boostingRounds"),
        getOptionAsDouble("trimRate"), getOptionAsInt("numSplits"), false, NULL);
    CvMat *varType = cvCreateMat(data->width + 1, 1, CV_8UC1);
    for (int i = 0; i < data->width; i++) {
	CV_MAT_ELEM(*varType, unsigned char, i, 0) = CV_VAR_NUMERICAL;
    }
    CV_MAT_ELEM(*varType, unsigned char, data->width, 0) = CV_VAR_CATEGORICAL;

    SVL_LOG(SVL_LOG_VERBOSE, "Creating boosting model...");
    _model = svlSmartPointer<CvBoost>(new CvBoost(data, CV_ROW_SAMPLE, labels,
            NULL, NULL, varType, NULL, parameters));
    SVL_LOG(SVL_LOG_VERBOSE, "...done");
    cvReleaseMat(&varType);

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
    
    SVL_LOG(SVL_LOG_MESSAGE, "Evaluation (TP, FN, TN, FP): " << tp << ", " << fn << ", " << tn << ", " << fp);

    // release memory
    cvReleaseMat(&labels);
    cvReleaseMat(&data);
}


void svlSlidingWindowDetector::learnModel(const char *posDirectory,  const char *negDirectory, const char *extensions[], int numChannels)
{
    vector<vector<double> > posSamples;
    vector<vector<double> > negSamples;

    unsigned maxTrainingImages = (unsigned)getOptionAsInt("maxTrainingImages");

    // read, resize and process positive images 
    DIR *dir = opendir(posDirectory);    
    if (dir == NULL) {
        SVL_LOG(SVL_LOG_FATAL, "could not open image directory " << posDirectory);
    }

    // load images (based on first channel image existing)
    struct dirent *e = readdir(dir);
    while (e != NULL) {
        if (strstr(e->d_name, extensions[0]) != NULL) {
            SVL_LOG(SVL_LOG_MESSAGE, "Processing " << e->d_name << "...");
    		            
            // holds all channels for each image 
            vector<IplImage*> imgs;

            for (int i = 0; i < numChannels; i++) {
                // form filename of the image for the current channel
                string imageFilename = string(e->d_name);
                imageFilename.replace(imageFilename.length() - strlen(extensions[0]),
                    strlen(extensions[0]), string(extensions[i]));
                imageFilename = string(posDirectory) + string("/") + imageFilename;

                // TO DO: make this more generic
                IplImage *img = NULL;				
                if (i == 0) {
                    img = cvLoadImage(imageFilename.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
                } else {
                    img = readMatrixAsIplImage(imageFilename.c_str(), _windowWidth, _windowHeight);
                }    		
                assert(img != NULL);
                
                // resize image
                if ((img->width != _windowWidth) || (img->height != _windowHeight)) {
                    IplImage *resizedImage = NULL;
                    resizedImage = cvCreateImage(cvSize(_windowWidth, _windowHeight), img->depth, 1);
                    
                    cvResize(img, resizedImage);
                    cvReleaseImage(&img);
                    img = resizedImage;
                }

                // store for feature vector computation
                imgs.push_back(img);
            } // loop over numChannels
                     
            // process image
            posSamples.push_back(vector<double>());
            computePatchFeatureVector(posSamples.back(), imgs);
				
            // free memory
            for (int i = 0; i < numChannels; i++) {
                cvReleaseImage(&(imgs[i]));
            }
        }

        if (posSamples.size() >= maxTrainingImages) {
            break;
        }

        e = readdir(dir);	
    } // next image
    closedir(dir);
    
    // read, resize and process negative images 
    dir = opendir(negDirectory);
    if (dir == NULL) {
        SVL_LOG(SVL_LOG_FATAL, "could not open image directory " << negDirectory);
    }

    e = readdir(dir);
    while (e != NULL) {
        if (strstr(e->d_name, extensions[0]) != NULL) {
            SVL_LOG(SVL_LOG_MESSAGE, "Processing " << e->d_name << "...");
    		            
            // holds all channels for each image 
            vector<IplImage*> imgs;

            for (int i = 0; i < numChannels; i++) {
                // form filename of the image for the current channel
                string imageFilename = string(e->d_name);
                imageFilename.replace(imageFilename.length() - strlen(extensions[0]),
                    strlen(extensions[0]), string(extensions[i]));
                imageFilename = string(posDirectory) + string("/") + imageFilename;

                // TO DO: make this more generic
                IplImage *img = NULL;				
                if (i == 0) {
                    img = cvLoadImage(imageFilename.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
                } else {
                    img = readMatrixAsIplImage(imageFilename.c_str(), _windowWidth, _windowHeight);
                }    		
                assert(img != NULL);
                
                // resize image
                if ((img->width != _windowWidth) || (img->height != _windowHeight)) {
                    IplImage *resizedImage = NULL;
                    resizedImage = cvCreateImage(cvSize(_windowWidth, _windowHeight), img->depth, 1);
                    
                    cvResize(img, resizedImage);
                    cvReleaseImage(&img);
                    img = resizedImage;
                }

                // store for feature vector computation
                imgs.push_back(img);
            } // loop over numChannels
                     
            // process image
            negSamples.push_back(vector<double>());
            computePatchFeatureVector(negSamples.back(), imgs);
				
            // free memory
            for (int i = 0; i < numChannels; i++) {
                cvReleaseImage(&(imgs[i]));
            }
        }

        if (negSamples.size() >= maxTrainingImages) {
            break;
        }

        e = readdir(dir);	
    } // next image
    closedir(dir);

    // learn the model
    learnModel(posSamples, negSamples);
}

float svlSlidingWindowDetector::evaluateModel(const vector<double>& fv)
{
    assert(_model != NULL);
    if (_featureVector == NULL) {
	_featureVector = cvCreateMat(1, (int)fv.size(), CV_32FC1);
    }
    if (_weakResponses == NULL) {
        _weakResponses = cvCreateMat(_model->get_params().weak_count, 1, CV_32FC1);
    }

    for (int j = 0; j < _featureVector->cols; j++) {
    	cvmSet(_featureVector, 0, j, fv[j]);
    }


    _model->predict(_featureVector, NULL, _weakResponses);
    float score = (float)cvSum(_weakResponses).val[0];

    return score;
}

float svlSlidingWindowDetector::evaluateModel(const vector<IplImage *>& images, CvRect region)
{
    vector<double> fv;

    assert((_model != NULL) && (!images.empty()));
    // each image in the vector must not be NULL and all images must be the same size
    for (unsigned i = 0; i < images.size(); i++) {
        assert(images[i] != NULL);
        assert(images[i]->width == images[0]->width);
        assert(images[i]->height == images[0]->height);
    }

    // if entire image, run full detector
    if ((region.x == 0) && (region.width == images[0]->width) &&
	(region.y == 0) && (region.height == images[0]->height)) {
        computePatchFeatureVector(fv, images);
        return evaluateModel(fv);
    }

    int numChannels = (int)images.size();

    // create subimages
    vector <IplImage *> subImages(numChannels);
    svlClipRect(region, images[0]);
    for (int i = 0; i < numChannels; i++) {
        IplImage *imgCopy = cvCloneImage(images[i]);

        subImages[i] = cvCreateImage(cvSize(region.width, region.height),
            images[i]->depth, images[i]->nChannels);
        assert(subImages[i] != NULL);
        
        cvSetImageROI(imgCopy, region);
        cvCopyImage(imgCopy, subImages[i]);
        cvReleaseImage(&imgCopy);
    }
    
    // compute feature vector
    computePatchFeatureVector(fv, subImages);

    // free memory
    for (int i = 0; i < numChannels; i++) {
        cvReleaseImage(&subImages[i]);
    }

    return evaluateModel(fv);
}

float svlSlidingWindowDetector::scoreToProbability(float score) const
{
    return 1.0f / (1.0f + exp(-score));
}

float svlSlidingWindowDetector::probabilityToScore(float pr) const
{
    return (float)(log(pr) - log(1.0f - pr));
}

// svlSingleChannelSlidingWindowDetector -------------------------------------

svlSingleChannelSlidingWindowDetector::svlSingleChannelSlidingWindowDetector() :
    svlSlidingWindowDetector()
{
    // do nothing
}


svlSingleChannelSlidingWindowDetector::~svlSingleChannelSlidingWindowDetector()
{
    // do nothing
}

// evaluate classifer over all shifts and scales
svlObject2dFrame svlSingleChannelSlidingWindowDetector::classifyImage(IplImage *image)
{
    vector<IplImage*> v(1, image);
    return svlSlidingWindowDetector::classifyImage(v);
}

// evaluate classifier at base scale and given shifts
svlObject2dFrame svlSingleChannelSlidingWindowDetector::classifyImage(IplImage *image,
    const std::vector<CvPoint>& locations)
{
    vector<IplImage*> v(1, image);
    return svlSlidingWindowDetector::classifyImage(v, locations);
}

// evaluate classifier on given region (and subwindows)
svlObject2dFrame svlSingleChannelSlidingWindowDetector::classifySubImage(IplImage *image,
    CvRect region)
{
    vector<IplImage*> v(1, image);
    return svlSlidingWindowDetector::classifySubImage(v, region);
}

// evaluate the model on a given feature vector/patch
float svlSingleChannelSlidingWindowDetector::evaluateModel(IplImage * image,
    CvRect region)
{
    vector<IplImage*> v(1, image);
    return svlSlidingWindowDetector::evaluateModel(v, region);
}
