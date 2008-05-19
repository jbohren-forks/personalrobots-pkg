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
** FILENAME:    svlIntrinsics.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**
*****************************************************************************/

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cassert>
#include <iostream>
#include <fstream>
#include <vector>

#include <cv.h>
#include <cxcore.h>

#include "svlIntrinsics.h"

using namespace std;

//----------------------------------------------------------------------------
// Camera Intrinsics Class
//----------------------------------------------------------------------------

svlCameraIntrinsics::svlCameraIntrinsics()
{
    focalLength[0] = focalLength[1] = 1.0;
    principalPoint[0] = principalPoint[1] = 0.0;
    skew = 0.0;
    for (int i = 0; i < 4; i++) {
        distortion[i] = 0.0;
    }

    intrinsicMatrix = NULL;
    distortionMatrix = NULL;
    mapX = NULL;
    mapY = NULL;
    inverseMapX = NULL;
    inverseMapY = NULL;
    inverseMapX2 = NULL;
    inverseMapY2 = NULL;
    convergenceMap2 = NULL;

    imageBuffer = NULL;
    rayBuffer = cvCreateMat(3, 1, CV_32FC1);
}

svlCameraIntrinsics::svlCameraIntrinsics(const svlCameraIntrinsics& c)
{
    focalLength[0] = c.focalLength[0];
    focalLength[1] = c.focalLength[1];
    principalPoint[0] = c.principalPoint[0];
    principalPoint[1] = c.principalPoint[1];
    skew = c.skew;
    for (int i = 0; i < 4; i++) {
        distortion[i] = c.distortion[i];
    }

    intrinsicMatrix = cvCloneMat(c.intrinsicMatrix);
    distortionMatrix = cvCloneMat(c.distortionMatrix);

    mapX = NULL;
    mapY = NULL;
    inverseMapX = NULL;
    inverseMapY = NULL;
    inverseMapX2 = NULL;
    inverseMapY2 = NULL;
    convergenceMap2 = NULL;

    imageBuffer = NULL;
    rayBuffer = cvCreateMat(3, 1, CV_32FC1);
}

svlCameraIntrinsics::~svlCameraIntrinsics()
{
    cvReleaseMat(&rayBuffer);
    if (intrinsicMatrix != NULL) cvReleaseMat(&intrinsicMatrix);
    if (distortionMatrix != NULL) cvReleaseMat(&distortionMatrix);
    freeMaps();
}

bool svlCameraIntrinsics::initialize(const char *filename)
{
    ifstream ifs(filename);
    assert(!ifs.fail());

    return initialize(ifs);
}

bool svlCameraIntrinsics::initialize(istream& is)
{
    // read paramters
    vector<double> v;
    v.resize(9);
    for (int i = 0; i < (int)v.size(); i++) {
        is >> v[i];
    }

    return this->initialize(v);
}

bool svlCameraIntrinsics::initialize(const vector<double>& v)
{
    // check parameters
    if (v.size() != 9) {
        return false;
    }
    
    // set parameters
    return this->initialize(v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]);
}

bool svlCameraIntrinsics::initialize(double fc_x, double fc_y, double cc_x, double cc_y,
    double alpha_c, double kc_0, double kc_1, double kc_2, double kc_3)
{
    // set parameters
    focalLength[0] = fc_x;
    focalLength[1] = fc_y;
    principalPoint[0] = cc_x;
    principalPoint[1] = cc_y;
    skew = alpha_c;
    distortion[0] = kc_0;
    distortion[1] = kc_1;
    distortion[2] = kc_2;
    distortion[3] = kc_3;

    // free previous matrices
    if (intrinsicMatrix != NULL) cvReleaseMat(&intrinsicMatrix);
    if (distortionMatrix != NULL) cvReleaseMat(&distortionMatrix);
    freeMaps();

    // create intrinsic matrix
    intrinsicMatrix = cvCreateMat(3, 3, CV_32FC1);
    cvSetZero(intrinsicMatrix);
    cvSetReal2D(intrinsicMatrix, 0, 0, focalLength[0]);
    cvSetReal2D(intrinsicMatrix, 0, 1, skew * focalLength[0]);
    cvSetReal2D(intrinsicMatrix, 0, 2, principalPoint[0]);
    cvSetReal2D(intrinsicMatrix, 1, 1, focalLength[1]);
    cvSetReal2D(intrinsicMatrix, 1, 2, principalPoint[1]);
    cvSetReal2D(intrinsicMatrix, 2, 2, 1.0);
    
    // create distortion
    distortionMatrix = cvCreateMat(1, 4, CV_32FC1);
    for (int i = 0; i < 4; i++) {
	cvSetReal2D(distortionMatrix, 0, i, distortion[i]);
    }

    return true;
}

void svlCameraIntrinsics::rescale(double factor)
{
    double factor2 = factor * factor;

    initialize(focalLength[0] * factor,
        focalLength[1] * factor,
        principalPoint[0] * factor,
        principalPoint[1] * factor,
        skew,
        distortion[0] * factor2,
        distortion[1] * factor2 * factor2,
        distortion[2] * factor2,
        distortion[3] * factor2);
}

const IplImage *svlCameraIntrinsics::undistort(const IplImage *image)
{
    assert(image != NULL);

    if ((imageBuffer == NULL) ||
        (imageBuffer->width != image->width) ||
        (imageBuffer->height != image->height)) {
        // free existing data and create new maps
        createMaps(image->width, image->height);
        imageBuffer = cvCreateImage(cvSize(image->width, image->height),
				    image->depth, image->nChannels);
    }

    if ((mapX == NULL) || (mapY == NULL)) {
        cvCopyImage(image, imageBuffer);
    } else {
        cvRemap(image, imageBuffer, mapX, mapY);
    }

#ifdef WIN32
    cvFlip(imageBuffer);
#endif

    return imageBuffer;
}

const CvMat *svlCameraIntrinsics::ray(double x, double y, bool bNormalize) 
{
    cvmSet(rayBuffer, 2, 0, 1.0);
    cvmSet(rayBuffer, 1, 0, (y - principalPoint[1]) / focalLength[1]);
    cvmSet(rayBuffer, 0, 0, (x - principalPoint[0]) / focalLength[0] - 
	skew * cvmGet(rayBuffer, 1, 0) );
    
    if (bNormalize) {
	cvScale(rayBuffer, rayBuffer, 1.0 / cvNorm(rayBuffer));
    }

    return rayBuffer;
}

svlPoint3d svlCameraIntrinsics::ray_pt(double x, double y, bool bNormalize) 
{
    svlPoint3d ret(1.0);
    ret.y = (y - principalPoint[1]) / focalLength[1];
    ret.x = (x - principalPoint[0]) / focalLength[0] - skew * ret.y;
    if ( bNormalize ) ret.normalize();
    return ret;
}

CvPoint svlCameraIntrinsics::point(const CvMat *ray) const
{
    double x, y;
    
    if (ray == NULL) {
	return cvPoint((int)principalPoint[0], (int)principalPoint[1]);
    }
    
    x = focalLength[0] * (cvmGet(ray, 0, 0) + skew * cvmGet(ray, 1, 0)) / cvmGet(ray, 2, 0) + principalPoint[0];
    y = focalLength[1] * cvmGet(ray, 1, 0) / cvmGet(ray, 2, 0) + principalPoint[1];
    
    return cvPoint((int)x, (int)y);
}

void svlCameraIntrinsics::calibratedXY(const CvMat *Z, CvMat *X, CvMat *Y)
{
    assert((Z != NULL) && (X != NULL) && (Y != NULL));
    assert((Z->width == X->width) && (Z->width == Y->width));
    assert((Z->height == X->height) && (Z->height == Y->height));
    assert((cvGetElemType(Z) == CV_32FC1) && (cvGetElemType(X) == CV_32FC1) && (cvGetElemType(Y) == CV_32FC1));

    for (int v = 0; v < Z->height; v++) {
        for (int u = 0; u < Z->width; u++) {
            float s = CV_MAT_ELEM(*Z, float, v, u);
            float py = (float)(v - principalPoint[1]) / (float)focalLength[1];
            float px = (float)(u - principalPoint[0] - skew * py) / (float)focalLength[0];
            CV_MAT_ELEM(*Y, float, v, u) = s * py;
            CV_MAT_ELEM(*X, float, v, u) = s * px;
        }
    }

}

svlCameraIntrinsics& svlCameraIntrinsics::operator=(const svlCameraIntrinsics& c)
{
    focalLength[0] = c.focalLength[0];
    focalLength[1] = c.focalLength[1];
    principalPoint[0] = c.principalPoint[0];
    principalPoint[1] = c.principalPoint[1];
    skew = c.skew;
    for (int i = 0; i < 4; i++) {
        distortion[i] = c.distortion[i];
    }

    intrinsicMatrix = cvCloneMat(c.intrinsicMatrix);
    distortionMatrix = cvCloneMat(c.distortionMatrix);

    mapX = NULL;
    mapY = NULL;
    inverseMapX = NULL;
    inverseMapY = NULL;
    inverseMapX2 = NULL;
    inverseMapY2 = NULL;
    convergenceMap2 = NULL;

    imageBuffer = NULL;
    rayBuffer = cvCreateMat(3, 1, CV_32FC1);

    return *this;
}

void svlCameraIntrinsics::createMaps(int width, int height)
{
    freeMaps();
    
    if ((intrinsicMatrix == NULL) || (distortionMatrix == NULL)) {
        cerr << "ERROR: no intrinsic parameters for creating maps" << endl;
        return;
    }
    
    mapX = cvCreateMat(height, width, CV_32FC1);
    mapY = cvCreateMat(height, width, CV_32FC1);
    
    cvInitUndistortMap(intrinsicMatrix, distortionMatrix, mapX, mapY);
}

void svlCameraIntrinsics::freeMaps()
{
    if (mapX != NULL) cvReleaseMat(&mapX);
    if (mapY != NULL) cvReleaseMat(&mapY);
    if (inverseMapX != NULL) cvReleaseMat(&inverseMapX);
    if (inverseMapY != NULL) cvReleaseMat(&inverseMapY);
    if (inverseMapX2 != NULL) cvReleaseMat(&inverseMapX2);
    if (inverseMapY2 != NULL) cvReleaseMat(&inverseMapY2);
    if (convergenceMap2 != NULL) cvReleaseMat(&convergenceMap2);
    
    if (imageBuffer != NULL) cvReleaseImage(&imageBuffer);
}
