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
** FILENAME:    svlSegImage.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**
*****************************************************************************/

#include <cstdlib>
#include <string>
#include <vector>
#include <set>
#include <map>

#include "cv.h"
#include "cxcore.h"

#include "svlSegImage.h"
#include "svlVisionUtils.h"

using namespace std;

bool svlSegImage::bAllowVoidAsMaxClassLabel = true;

svlSegImage::svlSegImage(const IplImage *image, const CvMat *seg, 
    const CvMat *labels) :
    _image(NULL), _segmentation(NULL), _pixelLabels(NULL)
{
    assert((image != NULL) && (seg != NULL));
    assert((image->width == seg->cols) && (image->height == seg->rows));
    assert((image->nChannels == 3) && (image->depth == IPL_DEPTH_8U));
    assert(CV_MAT_TYPE(seg->type) == CV_32SC1);

    _image = cvCloneImage(image);
    _segmentation = cvCloneMat(seg);

    if (labels != NULL) {
	assert((labels->rows == _segmentation->rows) &&
	    (labels->cols == _segmentation->cols) &&
	    (CV_MAT_TYPE(labels->type) == CV_32SC1));
	_pixelLabels = cvCloneMat(labels);
    }

    computeSegmentData();
    computeSegmentLabels();    
}
    
svlSegImage::svlSegImage(const char *imgFilename, const char *segFilename,
    const char *labelFilename) :
    _image(NULL), _segmentation(NULL), _pixelLabels(NULL)
{
    assert((imgFilename != NULL) && (segFilename != NULL));
    
    _name = string(imgFilename);
    // TO DO: check this---takes directory name, not filename
    if (_name.rfind("/") != string::npos) {
	_name.erase(_name.rfind("/"));
	_name.erase(0, _name.rfind("/"));
    }

    //cerr << "svlSegImage::svlSegImage() reading image..." << endl;
    _image = cvLoadImage(imgFilename, CV_LOAD_IMAGE_COLOR);
    assert(_image != NULL);

    //cerr << "svlSegImage::svlSegImage() reading segmentation..." << endl;
    _segmentation = cvCreateMat(_image->height, _image->width, CV_32SC1);
    bool success = readMatrix(_segmentation, segFilename);
    assert(success);

    //cerr << "svlSegImage::svlSegImage() reading labels..." << endl;
    if (labelFilename != NULL) {
	_pixelLabels = cvCreateMat(_image->height, _image->width, CV_32SC1);
	if (!readMatrix(_pixelLabels, labelFilename)) {
            cerr << "WARNING: failed to read groundtruth file " << labelFilename << endl;
	    cvReleaseMat(&_pixelLabels);
	    _pixelLabels = NULL;
	}
    }

    //cerr << "svlSegImage::svlSegImage() computing segmentation data..." << endl;
    computeSegmentData();
    //cerr << "svlSegImage::svlSegImage() computing segmentation labels..." << endl;
    computeSegmentLabels();
    //cerr << "...done" << endl;
}

svlSegImage::svlSegImage(const svlSegImage& segImg) :
    _name(segImg._name), _image(NULL), _segmentation(NULL), 
    _pixelLabels(NULL)
{
    _image = cvCloneImage(segImg._image);
    _segmentation = cvCloneMat(segImg._segmentation);
    if (segImg._pixelLabels != NULL) {
	_pixelLabels = cvCloneMat(segImg._pixelLabels);
    }
    
    _numSegments = segImg._numSegments;
    _segLabels = segImg._segLabels;
    _segPixels = segImg._segPixels;
    _segNeighbors = segImg._segNeighbors;
}

svlSegImage::~svlSegImage()
{
    if (_pixelLabels != NULL)
	cvReleaseMat(&_pixelLabels);
    cvReleaseMat(&_segmentation);
    cvReleaseImage(&_image);
}

// returns centroid for a given segment
const CvPoint svlSegImage::getCentroid(int segment) const {
    assert((segment >= 0) && (segment < _numSegments));
    CvPoint c;

    if (_segPixels[segment].empty()) {
	c.x = width() / 2;
	c.y = height() / 2;
	return c;
    }

    c.x = c.y = 0;
    for (unsigned i = 0; i < _segPixels[segment].size(); i++) {
	c.x += _segPixels[segment][i].x;
	c.y += _segPixels[segment][i].y;
    }
    c.x = (int)c.x / _segPixels[segment].size();
    c.y = (int)c.y / _segPixels[segment].size();
    
    return c;
}

// returns adjacency list (egdes)
vector<pair<int, int> > svlSegImage::getAdjacencyList() const
{
    vector<pair<int,int> > edges;
    assert(_segNeighbors.size() == (unsigned)_numSegments);
    for (int i = 0; i < _numSegments; i++) {
	for (set<int>::const_iterator j = _segNeighbors[i].begin();
	     j != _segNeighbors[i].end(); j++) {
	    // only insert edges once (i < j)
	    if (*j > i) {
		edges.push_back(make_pair(i, *j));
	    }
	}
    }
    
    return edges;    
}

// returns NxN matrix of edge indices (-1 for no edge)
vector<vector<int> > svlSegImage::getAdjacencyMap() const
{
    vector<vector<int> > edgeMap(_numSegments);
    for (unsigned i = 0; i < (unsigned)_numSegments; i++) {
	edgeMap[i] = vector<int>(_numSegments, -1);
    }

    vector<pair<int,int> > edges = getAdjacencyList();
    for (unsigned i = 0; i < edges.size(); i++) {
	edgeMap[edges[i].first][edges[i].second] = i;
	edgeMap[edges[i].second][edges[i].first] = i;
    }

    return edgeMap;
}

void svlSegImage::setLabels(const CvMat *labels)
{
    if (_pixelLabels)
	cvReleaseMat(&_pixelLabels);

    if (labels == NULL) {
	_pixelLabels = NULL;
    } else {
	assert((labels->cols == width()) && (labels->rows == height()));
	assert(CV_MAT_TYPE(labels->type) == CV_32SC1);    
	_pixelLabels = cvCloneMat(labels);
    }
   
    computeSegmentLabels();
}

void svlSegImage::setLabels(const vector<int>& labels)
{
    assert(labels.size() == (unsigned)_numSegments);
    _segLabels = labels;
    if (_pixelLabels == NULL) {
	_pixelLabels = cvCreateMat(height(), width(), CV_32SC1);
    }
    for (int y = 0; y < height(); y++) {
	for (int x = 0; x < width(); x++) {
	    int seg = CV_MAT_ELEM(*_segmentation, int, y, x);
	    CV_MAT_ELEM(*_pixelLabels, int, y, x) = _segLabels[seg];
	}
    }
}

// debugging and visualization -- caller must free image
IplImage *svlSegImage::visualize() const
{
    IplImage *debugImage = cvCloneImage(_image);

    for (int y = 1; y < height(); y++) {
	for (int x = 1; x < width(); x++) {
	    if ((CV_MAT_ELEM(*_segmentation, int, y, x) != 
		CV_MAT_ELEM(*_segmentation, int, y - 1, x)) ||
		(CV_MAT_ELEM(*_segmentation, int, y, x) != 
		    CV_MAT_ELEM(*_segmentation, int, y, x - 1))) {

		int baseIndx = y * debugImage->widthStep + 3 * x;
		debugImage->imageData[baseIndx + 2] = 0;
		debugImage->imageData[baseIndx + 1] = 0;
		debugImage->imageData[baseIndx + 0] = 0;		
	    }
	}
    }
    
    return debugImage;
}

void svlSegImage::colorSegment(IplImage *canvas, int segment, unsigned char grey) const
{
    colorSegment(canvas, segment, grey, grey, grey, 0.0);
}

void svlSegImage::colorSegment(IplImage *canvas, int segment, unsigned char red,
    unsigned char green, unsigned char blue, double alpha) const
{
    assert(canvas != NULL);
    assert((canvas->width == width()) && (canvas->height == height()));
    assert((canvas->depth == IPL_DEPTH_8U) && (canvas->nChannels == 3));
    if (alpha < 0.0) alpha = 0.0;
    if (alpha > 1.0) alpha = 1.0;
    
    for (int y = 0; y < height(); y++) {
	for (int x = 0; x < width(); x++) {
	    if (CV_MAT_ELEM(*_segmentation, int, y, x) == segment) {
		int baseIndx = y * canvas->widthStep + 3 * x;
		canvas->imageData[baseIndx + 2] = (unsigned char)(alpha * red + 
                    (1.0 - alpha) * canvas->imageData[baseIndx + 2]);
		canvas->imageData[baseIndx + 1] = (unsigned char)(alpha * green + 
                    (1.0 - alpha) * canvas->imageData[baseIndx + 1]);
		canvas->imageData[baseIndx + 0] = (unsigned char)(alpha * blue + 
                    (1.0 - alpha) * canvas->imageData[baseIndx + 0]);
	    }
	}
    }
}

void svlSegImage::colorBoundaries(IplImage *canvas, unsigned char red,
    unsigned char green, unsigned char blue) const
{
    assert(canvas != NULL);
    assert((canvas->width == width()) && (canvas->height == height()));
    assert((canvas->depth == IPL_DEPTH_8U) && (canvas->nChannels == 3));
    
    for (int y = 1; y < height(); y++) {
	for (int x = 1; x < width(); x++) {
	    if ((CV_MAT_ELEM(*_segmentation, int, y, x) != 
		CV_MAT_ELEM(*_segmentation, int, y - 1, x)) ||
		(CV_MAT_ELEM(*_segmentation, int, y, x) != 
		    CV_MAT_ELEM(*_segmentation, int, y, x - 1))) {

		int baseIndx = y * canvas->widthStep + 3 * x;
		canvas->imageData[baseIndx + 2] = red;
		canvas->imageData[baseIndx + 1] = green;
		canvas->imageData[baseIndx + 0] = blue;		
	    }
	}
    }
}

// private member functions

void svlSegImage::computeSegmentData()
{
    // set segment pixels
    _numSegments = 0;
    _segPixels.clear();
    CvPoint p;
    for (p.y = 0; p.y < height(); p.y++) {
	for (p.x = 0; p.x < width(); p.x++) {
	    int segId = CV_MAT_ELEM(*_segmentation, int, p.y, p.x);
	    assert(segId >= 0);
	    if (_numSegments <= segId) {
		_numSegments = segId + 1;
		_segPixels.resize(_numSegments);
	    }
	    _segPixels[segId].push_back(p);
	}
    }

    // we expect at least two segments
    assert(_numSegments > 1);

    // set segment neighbours
    _segNeighbors.clear();
    _segNeighbors.resize(_numSegments);
    for (int y = 1; y < height(); y++) {
	for (int x = 1; x < width(); x++) {
	    int segId = CV_MAT_ELEM(*_segmentation, int, y, x);
	    int nbrId = CV_MAT_ELEM(*_segmentation, int, y, x - 1);
	    if (nbrId != segId) {
		_segNeighbors[segId].insert(nbrId);
		_segNeighbors[nbrId].insert(segId);
	    }
	    nbrId = CV_MAT_ELEM(*_segmentation, int, y - 1, x);
	    if (nbrId != segId) {
		_segNeighbors[segId].insert(nbrId);
		_segNeighbors[nbrId].insert(segId);
	    }
	}
    }    

    // set labels to unassigned
    _segLabels.clear();
    _segLabels.resize(_numSegments, -1);
}

void svlSegImage::computeSegmentLabels()
{
    if (_pixelLabels == NULL) {
	_segLabels.clear();
	_segLabels.resize(_numSegments, -1);
	return;
    }

    map<int, unsigned> counts;
    unsigned maxCount;
    for (int i = 0; i < _numSegments; i++) {
	counts.clear();
	maxCount = 0;
	_segLabels[i] = -1;
	
	for (vector<CvPoint>::const_iterator it = _segPixels[i].begin();
	     it != _segPixels[i].end(); it++) {
	    int lbl = CV_MAT_ELEM(*_pixelLabels, int, it->y, it->x);
	    
	    if (!bAllowVoidAsMaxClassLabel && (lbl < 0))
		continue;

	    if (counts.find(lbl) == counts.end()) {
		counts[lbl] = 0;
	    }
	    counts[lbl] = counts[lbl] + 1;
	    if (counts[lbl] > maxCount) {
		_segLabels[i] = lbl;
		maxCount = counts[lbl];
	    }
	}

	assert(bAllowVoidAsMaxClassLabel || (maxCount != 0));
    }
}



