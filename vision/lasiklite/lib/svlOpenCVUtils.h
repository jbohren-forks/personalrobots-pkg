/*****************************************************************************
** STAIR VISION PROJECT
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
** FILENAME:    svlOpenUtils.h
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu> , Sid Batra<sidbatra@cs.stanford.edu>
** DESCRIPTION:
**  OpenCV utility functions.
**
*****************************************************************************/

#pragma once

#include <vector>
#include <iostream>

#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

#ifndef CV_LOAD_IMAGE_GRAYSCALE
#define CV_LOAD_IMAGE_GRAYSCALE 0
#define CV_LOAD_IMAGE_COLOR 1
#endif

using namespace std;

// dump data to stdout
void dump(const IplImage& image, int maxWidth = -1, int maxHeight = -1);
void dump(const CvMat& matrix, int maxColumns = -1, int maxRows = -1);

// read and write (matrix must be pre-allocated)

void readMatrix(CvMat *matrix, istream& is);
void readMatrix(CvMat *matrix, const char *filename);
void writeMatrixAsIplImage(IplImage *matrix, const char *filename);
void writeMatrixAsIplImage(IplImage *matrix, const char *filename , int x , int y , int width , int height);
IplImage* readMatrixAsIplImage(const char *filename);
void writeMatrix(const CvMat *matrix, ostream& os);
void writeMatrix(const CvMat *matrix, const char *filename);

// scale image or matrix
void scaleToRange(CvArr *array, double minValue = 0.0, double maxValue = 1.0);

// Assemble images into one big image. All images must be of the same format
// and rows * cols must be smaller than images.size(). If negative then will
// choose a square (rows = cols = ceil(sqrt(images.size()))).
IplImage *combineImages(const vector<IplImage *>& images, int rows = -1, int cols = -1);

// Return greyscale/color version of image
IplImage *greyImage(const IplImage *color);
IplImage *colorImage(const IplImage *grey); 

// Resize image inplace
void resizeInPlace(IplImage **image, int height, int width);
void resizeInPlace(CvMat **matrix, int rows, int cols);

// Converts a byte stream to an image and vice versa. Caller is responsable for
// freeing memory.
IplImage *makeIplImage(const unsigned char *data, int width, int height, int channels = 3);
unsigned char *makeByteStream(IplImage *image);

// Compute estimate of point normals from 3d point cloud projection into image.
// Data must be arranged in N-by-M matrices and allocated externally. Normals
// returned in nX, nY, nZ and will point towards the origin.
void estimatePointNormals(const CvMat *X, const CvMat *Y, const CvMat *Z,
    CvMat *nX, CvMat *nY, CvMat *nZ, int windowSize = 3);
void estimatePointNormalsFast(const CvMat *X, const CvMat *Y, const CvMat *Z,
    CvMat *nX, CvMat *nY, CvMat *nZ);

// Clip a rectangle region to a bounding box or image
void svlClipRect(CvRect& r, int width, int height);
void svlClipRect(CvRect& r, const IplImage *img);

// Rotate and translate a (dense) point cloud represented as three matrices.
void svlRotatePointCloud(CvMat *X, CvMat *Y, CvMat *Z, const CvMat *R);
void svlTranslatePointCloud(CvMat *X, CvMat *Y, CvMat *Z,
    double dx, double dy, double dz);

// Basic operations on CvMat's
void svlAddSquared(CvMat *src1, CvMat *src2, CvMat *dst, bool bNormalize = false);

// Fill zero points with value from nearest-neighbour
void svlNearestNeighbourFill(CvMat *X, float emptyToken = 0.0);
void svlNearestNeighbourFill(IplImage *X, unsigned char emptyToken = 0);

