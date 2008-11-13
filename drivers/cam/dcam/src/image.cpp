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
  imRawType = COLOR_CODING_NONE;
  imRawSize = 0;
  imType = COLOR_CODING_NONE;
  imSize = 0;
  imColorType = COLOR_CODING_NONE;
  imColorSize = 0;
  imRectType = COLOR_CODING_NONE;
  imRectSize = 0;
  imRectColorType = COLOR_CODING_NONE;
  imRectColorSize = 0;  

  // color conversion
  colorConvertType = COLOR_CONVERSION_BILINEAR;
  //  colorConvertType = COLOR_CONVERSION_EDGE;

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
  MEMFREE(im);
  MEMFREE(imColor);
  MEMFREE(imRect);
  MEMFREE(imRectColor);
  im = NULL;
  imColor = NULL;
  imRect = NULL;
  imRectColor = NULL;
  imRaw = NULL;

  imRawType = COLOR_CODING_NONE;
  imRawSize = 0;
  imType = COLOR_CODING_NONE;
  imSize = 0;
  imColorType = COLOR_CODING_NONE;
  imColorSize = 0;
  imRectType = COLOR_CODING_NONE;
  imRectSize = 0;
  imRectColorType = COLOR_CODING_NONE;
  imRectColorSize = 0;  

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
      if (imRectSize < imSize)
	{
	  MEMFREE(imRect);
	  imRectType = imType;
	  imSize = imWidth*imHeight;
	  imRect = (uint8_t *)MEMALIGN(imSize);
	}

      // set up images 
      cvInitImageHeader(srcIm, size, IPL_DEPTH_8U, 1);
      cvInitImageHeader(dstIm, size, IPL_DEPTH_8U, 1);
      cvSetData(srcIm, im, imWidth);
      cvSetData(dstIm, imRect, imWidth);
      
      //      cvRemap(srcIm,dstIm,rMapxy,rMapa);
      cvRemap(srcIm,dstIm,mx,my);
    }

  // rectify color image
  // assumes RGB
  if (imColorType != COLOR_CODING_NONE)
    {
      // set up rectified data buffer
      if (imRectColorSize < imColorSize)
	{
	  MEMFREE(imRectColor);
	  imRectColorType = imColorType;
	  imSize = imWidth*imHeight*3;
	  imRectColor = (uint8_t *)MEMALIGN(imSize);
	}

      // set up images 
      cvInitImageHeader(srcIm, size, IPL_DEPTH_8U, 3);
      cvInitImageHeader(dstIm, size, IPL_DEPTH_8U, 3);
      cvSetData(srcIm, imColor, imWidth*3);
      cvSetData(dstIm, imRectColor, imWidth*3);
      
      //      cvRemap(srcIm,dstIm,rMapxy,rMapa);
      cvRemap(srcIm,dstIm,mx,my);
    }
  return true;
}



// stereo class fns

StereoData::StereoData()
{
  imLeft = new ImageData();
  imRight = new ImageData();

  // disparity buffer
  imDisp = NULL;
  imDispSize = 0;

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

  // point array/vector
  numPts = 0;
  imPts = NULL;
  isPtArray = false;
  imPtsSize = 0;
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
  MEMFREE(imDisp);
  imDisp = NULL;
  imDispSize = 0;
  hasDisparity = false;
  MEMFREE(imPts);
  imPtsSize = 0;
  numPts = 0;
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


//
// gets params from a string
// "SVS"-type parameter strings use mm for the projection matrices, convert to m
// "OST"-type parameter strings use m for projection matrices
//

void
StereoData::extractParams(char *ps)
{
  std::string params;
  params = ps;
  double *pp;
  bool isSVS = false;

  // std::cout << params << "\n\n";

  if (strncmp(ps,"# SVS",5)==0) // SVS-type parameters
    {
      PRINTF("[dcam] SVS-type parameters\n");
      isSVS = true;
    }

  // left image
  for (int i=0; i<9; i++) imLeft->K[i] = 0.0; // original camera matrix
  extract(params, "[left camera]", "f ", imLeft->K[0]); // have to use space after "f"
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
  PRINTF("\n");

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
  PRINTF("\n");
	  
  pp = (double *)imLeft->P; // projection matrix
  for (int i=0; i<12; i++) pp[i] = 0.0;
  extract(params, "[left camera]", "proj",  pp, 12);
  if (isSVS)
    imLeft->P[3] *= .001;	// convert from mm to m

  PRINTF("[dcam] Left projection matrix\n");
  for (int i=0; i<3; i++)
    {
      for (int j=0; j<4; j++)
	PRINTF(" %.4f",imLeft->P[i*4+j]);
      PRINTF("\n");
    }
  PRINTF("\n");

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
  PRINTF("\n");

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
  PRINTF("\n");
	  
  pp = (double *)imRight->P; // projection matrix
  for (int i=0; i<12; i++) pp[i] = 0.0;
  extract(params, "[right camera]", "proj",  pp, 12);
  if (isSVS)
    imRight->P[3] *= .001;	// convert from mm to m

  PRINTF("[dcam] Right projection matrix\n");
  for (int i=0; i<3; i++)
    {
      for (int j=0; j<4; j++)
	PRINTF(" %.4f",imRight->P[i*4+j]);
      PRINTF("\n");
    }
  PRINTF("\n");


  // reprojection matrix
  double Tx = imRight->P[0] /  imRight->P[3];
  // first column
  RP[0] = 1.0;
  RP[4] = RP[8] = RP[12] = 0.0;
  
  // second column
  RP[5] = 1.0;
  RP[1] = RP[9] = RP[13] = 0.0;

  // third column
  RP[2] = RP[6] = RP[10] = 0.0;
  RP[14] = -Tx;

  // fourth column
  RP[3] = -imLeft->P[2];	// cx
  RP[7] = -imLeft->P[6];	// cy
  RP[11] = imLeft->P[0];	// fx, fy
  RP[15] = (imLeft->P[2] - imRight->P[2] - (double)offx) / Tx;

  PRINTF("[dcam] Reprojection matrix\n");
  for (int i=0; i<4; i++)
    {
      for (int j=0; j<4; j++)
	PRINTF(" %.4f",RP[i*4+j]);
      PRINTF("\n");
    }
  PRINTF("\n");



  // external params of undistorted cameras
  for (int i=0; i<3; i++) T[i] = 0.0;
  for (int i=0; i<3; i++) Om[i] = 0.0;
  extract(params, "[external]", "Tx", T[0]);
  extract(params, "[external]", "Ty", T[1]);
  extract(params, "[external]", "Tz", T[2]);  
  extract(params, "[external]", "Rx", Om[0]);
  extract(params, "[external]", "Ry", Om[1]);
  extract(params, "[external]", "Rz", Om[2]);  

  if (isSVS)			// in mm, convert to m
    {
      T[0] *= .001;
      T[1] *= .001;
      T[2] *= .001;
    }


  PRINTF("[dcam] External translation vector\n");
  for (int i=0; i<3; i++)
    PRINTF(" %.4f",T[i]);
  PRINTF("\n");
  PRINTF("\n");

  PRINTF("[dcam] External rotation vector\n");
  for (int i=0; i<3; i++)
    PRINTF(" %.4f",Om[i]);
  PRINTF("\n");
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


//
// color processing
// two algorithms: linear interpolation, and edge-tracking interpolation
//

// convert from Bayer to RGB (3 bytes)

#define AVG(a,b) (((int)(a) + (int)(b))>>1)

void
ImageData::doBayerColorRGB()
{
  // check allocation
  size_t size = imWidth*imHeight;
  if (imSize < size)
    {
      MEMFREE(im);
      im = (uint8_t *)MEMALIGN(size);
      imSize = size;
    }
  if (imColorSize < size*3)
    {
      MEMFREE(imColor);
      imColor = (uint8_t *)MEMALIGN(size*3);
      imColorSize = size*3;
    }
  convertBayerRGGBColorRGB(imRaw, imColor, im, imWidth, imHeight, colorConvertType);
  imType = COLOR_CODING_MONO8;
  imColorType = COLOR_CODING_RGB8;
}


void 
ImageData::doBayerMono()
{
  // check allocation
  size_t size = imWidth*imHeight;
  if (imSize < size)
    {
      MEMFREE(im);
      im = (uint8_t *)MEMALIGN(size);
      imSize = size;
    }
  convertBayerRGGBMono(imRaw, im, imWidth, imHeight, colorConvertType);
  imType = COLOR_CODING_MONO8;
}


// real funtion to do the job
// converts to RGB

void
ImageData::convertBayerRGGBColorRGB(uint8_t *src, uint8_t *dstc, uint8_t *dstm, 
				    int width, int height, color_conversion_t colorAlg)
{
  uint8_t *s;
  int i, j;
  int ll = width;
  int ll2 = width*2;

  s = src;
  int pp2 = width*3*2;          // previous 2 color lines
  int pp = width*3;             // previous color line
  uint8_t *cd = dstc;		// color
  uint8_t *md = dstm;		// monochrome

  // simple, but has "zipper" artifacts
  if (colorAlg == COLOR_CONVERSION_BILINEAR)
    {
      for (i=0; i<height; i+=2)
	{
	  // red line (GRGR...)
	  for (j=0; j<width; j+=2, cd+=6, md+=2)
	    {
	      *md = *(cd+1) = *s++;	// green pixel
	      *(cd+3+0) = *s++;	// red pixel          
	      *(cd+0) = AVG(*(cd+3+0), *(cd-3+0)); // interpolated red pixel
	      if (i > 1)
		{
		  *(cd-pp+0) = AVG(*(cd-pp2+0), *(cd+0)); // interpolated red pixel
		  *(cd-pp+3+0) = AVG(*(cd-pp2+3+0), *(cd+3+0)); // interpolated red pixel
		  *(md-ll) = *(cd-pp+1) = ((int)*(cd+1) + (int)*(cd-pp-3+1) + (int)*(cd-pp+3+1) + (int)*(cd-pp2+1)) >> 2;
		}
	    }		  	

	  // blue line (BGBG...)
	  *(cd+2) = *s;		// blue pixel          
	  for (j=0; j<width-2; j+=2, cd+=6, md+=2)
	    {
	      s++;
	      *(md+1) = *(cd+3+1) = *s++; // green pixel
	      *(cd+6+2) = *s;
	      *(cd+3+2) = AVG(*(cd+2), *(cd+6+2)); // interpolated blue pixel
	      if (i > 1)
		{
		  *(cd-pp+2) = AVG(*(cd-pp2+2), *(cd+2)); // interpolated blue pixel
		  *(cd-pp+3+2) = AVG(*(cd-pp2+3+2), *(cd+3+2)); // interpolated blue pixel
		  *(md-ll+1) = *(cd-pp+3+1) = ((int)*(cd+3+1) + (int)*(cd-pp+1) + (int)*(cd-pp+6+1) + (int)*(cd-pp2+3+1)) >> 2;
		}
	    }		  	
	  // last pixels
	  s++;
	  *(md+1) = *(cd+3+1) = *s++;      // green pixel
	  *(cd+3+2) = *(cd+2);	// interpolated blue pixel
	  if (i > 1)
	    {
	      *(cd-pp+2) = AVG(*(cd-pp2+2), *(cd+2)); // interpolated blue pixel
	      *(cd-pp+3+2) = AVG(*(cd-pp2+3+2), *(cd+3+2)); // interpolated blue pixel
	    }
	  cd +=6;
	  md +=2;
	}
    }

  // EDGE color algorithm, better but slower
  else
    {
      int a,b,c,d;
      int dc, dv, dh;
      int ww;

      // do first two lines
      cd += pp2;
      s += ll2;

      for (i=0; i<height-4; i+=2)
	{
	  // GR line
	  // do first two pixels
	  cd += 6;
	  md += 2;
	  s += 2;

	  // do most of line
	  for (j=0; j<width-4; j+=2, cd+=6, md+=2)
	    {
	      // green pixels
	      *md = *(cd+1) = *s++;
	      dc = 2*(int)*(s);
	      dh = dc - (int)*(s-2) - (int)*(s+2);
	      if (dh < 0) dh = -dh;
	      dv = dc - (int)*(s-ll2) - (int)*(s+ll2);
	      if (dv < 0) dv = -dv;
	      if (dv > dh) // vert is stronger, use horz
		*(md+1) = *(cd+3+1) = ((int)*(s-1) + (int)*(s+1))>>1;
	      else	// horz is stronger, use vert
		*(md+1) = *(cd+3+1) = ((int)*(s-ll) + (int)*(s+ll))>>1;

	      // color pixels
	      *(cd+3+0) = *s;	// red pixel

	      a = (int)*(s) - (int)*(cd+3+1);
	      b = (int)*(s-2) - (int)*(cd-3+1);
	      c = (int)*(s-ll2) - (int)*(cd-pp2+3+1);
	      d = (int)*(s-ll2-2) - (int)*(cd-pp2-3+1);

	      ww = 2*(int)*(cd+1) + (a + b);
	      if (ww < 0) ww = 0;
	      *(cd+0) = ww>>1;	// interpolated red pixel 

	      ww = 2*(int)*(cd-pp+3+1) + (a + c);
	      if (ww < 0) ww = 0;
	      *(cd-pp+3+0) = ww>>1; // interpolated red pixel

	      ww = 4*(int)*(cd-pp+1) + (a + b + c + d);
	      if (ww < 0) ww = 0;
	      *(cd-pp+0) = ww>>2; // interpolated red pixel

	      s++;
	    }
	  // last two pixels
	  cd += 6;
	  md += 2;
	  s += 2;

	  // BG line
	  // do first two pixels
	  cd += 6;
	  md += 2;
	  s += 2;

	  // do most of line
	  for (j=0; j<width-4; j+=2, cd+=6, md+=2)
	    {
	      dc = 2*(int)*s;
	      dh = dc - (int)*(s-2) - (int)*(s+2);
	      if (dh < 0) dh = -dh;
	      dv = dc - (int)*(s-ll2) - (int)*(s+ll2);
	      if (dv < 0) dv = -dv;
	      if (dh < dv) // vert is stronger, use horz
		*md = *(cd+1) = ((int)*(s-1) + (int)*(s+1))>>1;
	      else	// horz is stronger, use vert
		*md = *(cd+1) = ((int)*(s-ll) + (int)*(s+ll))>>1;

	      *(md+1) = *(cd+3+1) = *(s+1); // green pixel

	      // color pixels
	      *(cd+3+2) = *s;	// blue pixel

	      a = (int)*(s) - (int)*(cd+3+1);
	      b = (int)*(s-2) - (int)*(cd-3+1);
	      c = (int)*(s-ll2) - (int)*(cd-pp2+3+1);
	      d = (int)*(s-ll2-2) - (int)*(cd-pp2-3+1);

	      ww = 2*(int)*(cd+1) + (a + b);
	      if (ww < 0) ww = 0;
	      *(cd+2) = ww>>1;	// interpolated blue pixel 

	      ww = 2*(int)*(cd-pp+3+1) + (a + c);
	      if (ww < 0) ww = 0;
	      *(cd-pp+3+2) = ww>>1; // interpolated blue pixel

	      ww = 4*(int)*(cd-pp+1) + (a + b + c + d);
	      if (ww < 0) ww = 0;
	      *(cd-pp+2) = ww>>2; // interpolated blue pixel

	      s+=2;
	    }
	  // last two pixels
	  cd += 6;
	  md += 2;
	  s += 2;
	}

      // last two lines
      for (j=0; j<width; j+=2)
	{
	}
    }

}


// real function to do the job
// converts to monochrome

void
ImageData::convertBayerRGGBMono(uint8_t *src, uint8_t *dstm, 
				int width, int height, color_conversion_t colorAlg)
{
  uint8_t *s;
  int i, j;
  int ll = width;
  int ll2 = width*2;

  s = src;
  uint8_t *md = dstm;		// monochrome

  // simple, but has "zipper" artifacts
  if (colorAlg == COLOR_CONVERSION_BILINEAR)
    {
      for (i=0; i<height; i+=2)
	{
	  // red line (GRGR...)
	  for (j=0; j<width; j+=2, md+=2)
	    {
	      *md = *s++;	// green pixel
	      s++;	// red pixel          
	      if (i > 1)
		*(md-ll) = ((int)*(md) + (int)*(md-ll-1) + (int)*(md-ll+1) + (int)*(md-ll2)) >> 2;
	    }		  	

	  // blue line (BGBG...)
	  for (j=0; j<width-2; j+=2, md+=2)
	    {
	      s++;
	      *(md+1) = *s++; // green pixel
	      if (i > 1)
		*(md-ll+1) = ((int)*(md+1) + (int)*(md-ll) + (int)*(md-ll+2) + (int)*(md-ll2+1)) >> 2;
	    }		  	
	  // last pixels
	  s++;
	  *(md+1) = *s++;	// green pixel
	  md +=2;
	}
    }

  // EDGE color algorithm, better but slower
  else
    {
      int dc, dv, dh;

      // do first two lines
      s += ll2;

      for (i=0; i<height-4; i+=2)
	{
	  // GR line
	  // do first two pixels
	  md += 2;
	  s += 2;

	  // do most of line
	  for (j=0; j<width-4; j+=2, md+=2)
	    {
	      // green pixels
	      *md = *s++;
	      dc = 2*(int)*(s);
	      dh = dc - (int)*(s-2) - (int)*(s+2);
	      if (dh < 0) dh = -dh;
	      dv = dc - (int)*(s-ll2) - (int)*(s+ll2);
	      if (dv < 0) dv = -dv;
	      if (dv > dh) // vert is stronger, use horz
		*(md+1) = ((int)*(s-1) + (int)*(s+1))>>1;
	      else	// horz is stronger, use vert
		*(md+1) = ((int)*(s-ll) + (int)*(s+ll))>>1;

	      s++;
	    }
	  // last two pixels
	  md += 2;
	  s += 2;

	  // BG line
	  // do first two pixels
	  md += 2;
	  s += 2;

	  // do most of line
	  for (j=0; j<width-4; j+=2, md+=2)
	    {
	      dc = 2*(int)*s;
	      dh = dc - (int)*(s-2) - (int)*(s+2);
	      if (dh < 0) dh = -dh;
	      dv = dc - (int)*(s-ll2) - (int)*(s+ll2);
	      if (dv < 0) dv = -dv;
	      if (dh < dv) // vert is stronger, use horz
		*md = ((int)*(s-1) + (int)*(s+1))>>1;
	      else	// horz is stronger, use vert
		*md = ((int)*(s-ll) + (int)*(s+ll))>>1;

	      *(md+1) = *(s+1); // green pixel

	      s+=2;
	    }
	  // last two pixels
	  md += 2;
	  s += 2;
	}

      // last two lines
      for (j=0; j<width; j+=2)
	{
	}
    }

}
