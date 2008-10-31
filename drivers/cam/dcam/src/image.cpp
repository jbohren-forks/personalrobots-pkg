/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/


//
// image.cpp
// classes for monocular and stereo image
//

#include "image.h"

#include <sstream>
#include <iostream>

#define PRINTF(a...) printf(a)

using namespace cam;

// image class fns

ImageData::ImageData()
{
  imWidth = imHeight = 0;

  // buffers
  im = NULL;
  imColor = NULL;
  imRect = NULL;
  imRectColor = NULL;
  imRaw = NULL;
  imRawType = PIXEL_CODING_NONE;
  imType = COLOR_CODING_NONE;
  imColorType = COLOR_CODING_NONE;
  imRectType = COLOR_CODING_NONE;
  imRectColorType = COLOR_CODING_NONE;
  
  // rectification mapping
  hasRectification = false;
  initRect = false;
  rMapxy = NULL;
  rMapa = NULL;

  // calibration matrices
  rD = cvCreateMat(5,1,CV_64F);
  rK = cvCreateMat(3,3,CV_64F);
  rR = cvCreateMat(3,3,CV_64F);
  rKp = cvCreateMat(3,3,CV_64F);

  // temp images, these will be changed when used
  srcIm = cvCreateImageHeader(cvSize(640,480), IPL_DEPTH_8U, 1);
  dstIm = cvCreateImageHeader(cvSize(640,480), IPL_DEPTH_8U, 1);
}

ImageData::~ImageData()
{
  releaseBuffers();
  cvReleaseImageHeader(&srcIm);
  cvReleaseImageHeader(&dstIm);
}

// storage

void
ImageData::releaseBuffers()
{
  // should we release im_raw???
  if (im)
    MEMFREE(im);
  if (imColor)
    MEMFREE(imColor);
  if (imRect)
    MEMFREE(imRect);
  if (imRectColor)
    MEMFREE(imRectColor);
  im = NULL;
  imColor = NULL;
  imRect = NULL;
  imRectColor = NULL;
  imRaw = NULL;
  imRawType = PIXEL_CODING_NONE;
  imType = COLOR_CODING_NONE;
  imColorType = COLOR_CODING_NONE;
  imRectType = COLOR_CODING_NONE;
  imRectColorType = COLOR_CODING_NONE;

  if (rMapxy)
    cvReleaseMat(&rMapxy);
  if (rMapa)
    cvReleaseMat(&rMapa);
  rMapxy = NULL;
  rMapa = NULL;
}



// rectification

bool
ImageData::initRectify()
{
  if (!hasRectification || imWidth == 0 || imHeight == 0)
    return false;

  if (initRect)
    return true;		// already done

  // set values of cal matrices
  for (int i=0; i<3; i++)
    for (int j=0; j<3; j++)
      CV_MAT_ELEM(*rK, double, i, j) = K[i*3+j];

  // rectified K matrix, from projection matrix
  for (int i=0; i<3; i++)
    for (int j=0; j<3; j++)
      CV_MAT_ELEM(*rKp, double, i, j) = P[i*4+j];

  for (int i=0; i<3; i++)
    for (int j=0; j<3; j++)
      CV_MAT_ELEM(*rR, double, i, j) = R[i*3+j];
  
  for (int i=0; i<5; i++)
    CV_MAT_ELEM(*rD, double, i, 0) = D[i];

  // Set up rectification mapping
  rMapxy = cvCreateMat(imHeight, imWidth, CV_16SC2);
  rMapa  = cvCreateMat(imHeight, imWidth, CV_16SC1);
  mx = cvCreateMat(imHeight, imWidth, CV_32FC1);
  my = cvCreateMat(imHeight, imWidth, CV_32FC1);
  cvInitUndistortRectifyMap(rK,rD,rR,rKp,mx,my);
  cvConvertMaps(mx,my,rMapxy,rMapa);

  initRect = true;
  return true;
}


bool
ImageData::doRectify()
{
  if (!hasRectification)
    return false;		// has no rectification

  if (imWidth == 0 || imHeight == 0)
    return false;

  if (imType == COLOR_CODING_NONE && imColorType == COLOR_CODING_NONE)
    return false;		// nothing to rectify

  if (!((imType != COLOR_CODING_NONE && imRectType == COLOR_CODING_NONE) ||
	(imColorType != COLOR_CODING_NONE && imRectColorType == COLOR_CODING_NONE)))
    return true;		// already done

  initRectify();		// ok to call multiple times

  CvSize size = cvSize(imWidth,imHeight);

  // rectify grayscale image
  if (imType != COLOR_CODING_NONE)
    {
      // set up rectified data buffer
      if (imRectType == COLOR_CODING_NONE)
	{
	  imRectType = imType;
	  imRect = (uint8_t *)MEMALIGN(imWidth*imHeight);
	}

      // set up images 
      cvInitImageHeader(srcIm, size, IPL_DEPTH_8U, 1);
      cvInitImageHeader(dstIm, size, IPL_DEPTH_8U, 1);
      cvSetData(srcIm, im, imWidth);
      cvSetData(dstIm, imRect, imWidth);
      
      //      printf("rect ptrs: %08x %08x\n", im, imRect);
      //      cvRemap(srcIm,dstIm,rMapxy,rMapa);
      cvRemap(srcIm,dstIm,mx,my);
    }

  // rectify color image
  // assumes RGB
  if (imColorType != COLOR_CODING_NONE)
    {
      // set up rectified data buffer
      if (imRectColorType == COLOR_CODING_NONE)
	{
	  imRectColorType = imColorType;
	  imRectColor = (uint8_t *)MEMALIGN(imWidth*imHeight*3);
	}

      // set up images 
      cvInitImageHeader(srcIm, size, IPL_DEPTH_8U, 3);
      cvInitImageHeader(dstIm, size, IPL_DEPTH_8U, 3);
      cvSetData(srcIm, imColor, imWidth);
      cvSetData(dstIm, imRectColor, imWidth);
      
      cvRemap(srcIm,dstIm,mx,my);
      //      cvRemap(srcIm,dstIm,rMapxy,rMapa);
    }
  return true;
}



// stereo class fns

StereoData::StereoData()
{
  imLeft = new ImageData();
  imRight = new ImageData();

  // nominal values
  imWidth = 640;
  imHeight = 480;
  corrSize = 15;
  filterSize = 11;
  horOffset = 0;
  setDispOffsets();

  textureThresh = 10;
  uniqueThresh = 12;

  hasRectification = false;
}

StereoData::~StereoData()
{
  releaseBuffers();
  free(imLeft);
  free(imRight);
}

void
StereoData::setDispOffsets()
{
  /*
   * disparity image size
   * ====================
   * dleft  : (logs + corrs - 2)/2 - 1 + offx
   * dwidth : w - (logs + corrs + offx - 2)
   * dtop   : (logs + corrs - 2)/2
   * dheight: h - (logs + corrs)
   *
   */

  imDtop = (filterSize + corrSize - 2)/2;
  imDleft = (filterSize + corrSize - 2)/2 - 1 + horOffset;
  imDwidth = imWidth - (filterSize + corrSize + horOffset - 2);
  imDheight = imHeight - (filterSize + corrSize);
}

void
StereoData::releaseBuffers()
{
  if (imDisp)
    MEMFREE(imDisp);
  imDisp = NULL;
  hasDisparity = false;
  imLeft->releaseBuffers();
  imRight->releaseBuffers();
}



// image size
void
StereoData::setSize(int width, int height)
{
  imWidth = width;
  imHeight = height;
  imLeft->imWidth = width;
  imLeft->imHeight = height;
  imRight->imWidth = width;
  imRight->imHeight = height;
}



//
// param sting parsing routines
//

template <class T>
void extract(std::string& data, std::string section, std::string param, T& t)
{
  std::istringstream iss(data.substr( data.find(param, data.find(section) ) + 
				      param.length()));
  iss >> t;
}

void extract(std::string& data, std::string section, 
		  std::string param, double *m, int n)
{
  std::istringstream iss(data.substr( data.find(param, data.find(section) ) + 
				      param.length()));

  double v;
  for (int i=0; i<n; i++)
    {
      iss >> v;
      m[i] = v;
    }
}


void
StereoData::extractParams(char *ps)
{
  std::string params;
  params = ps;
  double *pp;

  //  std::cout << params << "\n\n";

  // left image
  for (int i=0; i<9; i++) imLeft->K[i] = 0.0; // original camera matrix
  extract(params, "[left camera]", "f ", imLeft->K[0]); // ?? have to use "f " here
  extract(params, "[left camera]", "fy", imLeft->K[4]);
  extract(params, "[left camera]", "Cx", imLeft->K[2]);
  extract(params, "[left camera]", "Cy", imLeft->K[5]);
  imLeft->K[8] = 1.0;	  

  PRINTF("[dcam] Left camera matrix\n");
  for (int i=0; i<3; i++)
    {
      for (int j=0; j<3; j++)
	PRINTF(" %.4f",imLeft->K[i*3+j]);
      PRINTF("\n");
    }

  for (int i=0; i<5; i++) imLeft->D[i] = 0.0; // distortion params
  extract(params, "[left camera]", "kappa1", imLeft->D[0]);
  extract(params, "[left camera]", "kappa2", imLeft->D[1]);
  extract(params, "[left camera]", "tau1", imLeft->D[2]);
  extract(params, "[left camera]", "tau2", imLeft->D[3]);
  extract(params, "[left camera]", "kappa3", imLeft->D[4]);

  PRINTF("[dcam] Left distortion vector\n");
  for (int i=0; i<5; i++)
    PRINTF(" %.4f",imLeft->D[i]);
  PRINTF("\n");

  pp = (double *)imLeft->R; // rectification matrix
  for (int i=0; i<9; i++) pp[i] = 0.0;
  extract(params, "[left camera]", "rect",  pp, 9);
	  
  PRINTF("[dcam] Left rectification matrix\n");
  for (int i=0; i<3; i++)
    {
      for (int j=0; j<3; j++)
	PRINTF(" %.4f",imLeft->R[i*3+j]);
      PRINTF("\n");
    }
	  
  pp = (double *)imLeft->P; // projection matrix
  for (int i=0; i<12; i++) pp[i] = 0.0;
  extract(params, "[left camera]", "proj",  pp, 12);

  PRINTF("[dcam] Left projection matrix\n");
  for (int i=0; i<3; i++)
    {
      for (int j=0; j<4; j++)
	PRINTF(" %.4f",imLeft->P[i*4+j]);
      PRINTF("\n");
    }

  // check for camera matrix
  if (imLeft->K[0] == 0.0) 
    hasRectification = false;
  else
    {
      hasRectification = true;
      imLeft->hasRectification = true;
      imLeft->initRect = false;	// haven't initialized arrays, wait for image size
    }

  // right image
  for (int i=0; i<9; i++) imRight->K[i] = 0.0; // original camera matrix
  extract(params, "[right camera]", "f ", imRight->K[0]); // ?? have to use "f " here
  extract(params, "[right camera]", "fy", imRight->K[4]);
  extract(params, "[right camera]", "Cx", imRight->K[2]);
  extract(params, "[right camera]", "Cy", imRight->K[5]);
  imRight->K[8] = 1.0;	  

  PRINTF("[dcam] Right camera matrix\n");
  for (int i=0; i<3; i++)
    {
      for (int j=0; j<3; j++)
	PRINTF(" %.4f",imRight->K[i*3+j]);
      PRINTF("\n");
    }

  for (int i=0; i<5; i++) imRight->D[i] = 0.0; // distortion params
  extract(params, "[right camera]", "kappa1", imRight->D[0]);
  extract(params, "[right camera]", "kappa2", imRight->D[1]);
  extract(params, "[right camera]", "tau1", imRight->D[2]);
  extract(params, "[right camera]", "tau2", imRight->D[3]);
  extract(params, "[right camera]", "kappa3", imRight->D[4]);

  PRINTF("[dcam] Right distortion vector\n");
  for (int i=0; i<5; i++)
    PRINTF(" %.4f",imRight->D[i]);
  PRINTF("\n");

  pp = (double *)imRight->R; // rectification matrix
  for (int i=0; i<9; i++) pp[i] = 0.0;
  extract(params, "[right camera]", "rect",  pp, 9);
	  
  PRINTF("[dcam] Right rectification matrix\n");
  for (int i=0; i<3; i++)
    {
      for (int j=0; j<3; j++)
	PRINTF(" %.4f",imRight->R[i*3+j]);
      PRINTF("\n");
    }
	  
  pp = (double *)imRight->P; // projection matrix
  for (int i=0; i<12; i++) pp[i] = 0.0;
  extract(params, "[right camera]", "proj",  pp, 12);
  imRight->P[3] *= .001;	// convert from mm to m

  PRINTF("[dcam] Right projection matrix\n");
  for (int i=0; i<3; i++)
    {
      for (int j=0; j<4; j++)
	PRINTF(" %.4f",imRight->P[i*4+j]);
      PRINTF("\n");
    }

  // external params of undistorted cameras
  for (int i=0; i<3; i++) T[i] = 0.0;
  for (int i=0; i<3; i++) Om[i] = 0.0;
  extract(params, "[external]", "Tx", T[0]);
  extract(params, "[external]", "Ty", T[1]);
  extract(params, "[external]", "Tz", T[2]);  
  extract(params, "[external]", "Rx", Om[0]);
  extract(params, "[external]", "Ry", Om[1]);
  extract(params, "[external]", "Rz", Om[2]);  

  PRINTF("[dcam] External translation vector\n");
  for (int i=0; i<3; i++)
    PRINTF(" %.4f",T[i]);
  PRINTF("\n");

  PRINTF("[dcam] External rotation vector\n");
  for (int i=0; i<3; i++)
    PRINTF(" %.4f",Om[i]);
  PRINTF("\n");

  extract(params, "[stereo]", "corrxsize", corrSize);
  PRINTF("[dcam] Correlation window: %d\n", corrSize);
  extract(params, "[stereo]", "convx", filterSize);
  PRINTF("[dcam] Prefilter window: %d\n", filterSize);
  extract(params, "[stereo]", "ndisp", numDisp);
  PRINTF("[dcam] Number of disparities: %d\n", numDisp);

  // check for camera matrix
  if (imRight->K[0] == 0.0) 
    hasRectification = false;
  else
    {
      imRight->hasRectification = true;
      imRight->initRect = false; // haven't initialized arrays, wait for image size
    }
}
