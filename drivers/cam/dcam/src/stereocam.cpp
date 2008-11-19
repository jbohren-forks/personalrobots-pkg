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
// Stereo driver and processor
// Main class is StereoCam for processing
// Subclasses:
//   StereoDcam - gets stereo images from DCAM (IEEE1394) devices
//   

#include "stereocam.h"
#include "stereolib.h"

#define PRINTF(a...) printf(a)


// StereoCam class
// Performs rectification and disparity calculation
//

using namespace cam;

StereoCam::StereoCam()
{
  stIm = new StereoData();
  buf = NULL;
  flim = NULL;
  frim = NULL;
}

StereoCam::~StereoCam()
{
  // should free all buffers
  MEMFREE(buf);
  MEMFREE(flim);
  MEMFREE(frim);
  free(stIm);
}

bool
StereoCam::doRectify()
{
  bool res = stIm->imLeft->doRectify();
  res = res && stIm->imRight->doRectify();
  return res;
}


bool 
StereoCam::doDisparity()
{
  uint8_t *lim, *rim;

  // first do any rectification necessary
  doRectify();

  // check if the rectified images are present
  if (stIm->imLeft->imRectType == COLOR_CODING_NONE ||
      stIm->imRight->imRectType == COLOR_CODING_NONE)
    return false;

  // variables
  lim = (uint8_t *)stIm->imLeft->imRect;
  rim = (uint8_t *)stIm->imRight->imRect;
  int xim = stIm->imWidth;
  int yim = stIm->imHeight;

  // some parameters
  int ftzero = 31;		// max 31 cutoff for prefilter value (31 default)
  int dlen   = stIm->numDisp;	// number of disparities
  int corr   = stIm->corrSize;	// correlation window size
  int tthresh = stIm->textureThresh; // texture threshold
  int uthresh = stIm->uniqueThresh; // uniqueness threshold, percent

  // allocate buffers
  if (!stIm->imDisp)
    stIm->imDisp = (int16_t *)MEMALIGN(xim*yim*2);

  if (!buf)
    buf  = (uint8_t *)malloc(yim*dlen*(corr+5)); // local storage for the algorithm
  if (!flim)
    flim = (uint8_t *)MEMALIGN(xim*yim); // feature image
  if (!frim)
    frim = (uint8_t *)MEMALIGN(xim*yim); // feature image

  // prefilter
  do_prefilter(lim, flim, xim, yim, ftzero, buf);
  do_prefilter(rim, frim, xim, yim, ftzero, buf);

  // stereo
  do_stereo(flim, frim, stIm->imDisp, NULL, xim, yim, 
	    ftzero, corr, corr, dlen, tthresh, uthresh, buf);

  stIm->hasDisparity = true;
  return true;
}


bool
StereoCam::setTextureThresh(int thresh)
{
  stIm->textureThresh = thresh;
  return true;
}

bool
StereoCam::setUniqueThresh(int thresh)
{
  stIm->uniqueThresh = thresh;
  return true;
}


//
// Conversion to 3D points
// Convert to vector or image array of pts
//


bool
StereoCam::doCalcPts()
{
  stIm->numPts = 0;
  if (!stIm->hasDisparity)
    return false;

  int ix = stIm->imDleft;
  int iy = stIm->imDtop;
  int ih = stIm->imDheight;
  int iw = stIm->imDwidth;
  int w = stIm->imWidth;
  int h = stIm->imHeight;

  if (stIm->imPtsSize < 4*w*h*sizeof(float))
    {
      MEMFREE(stIm->imPts);
      stIm->imPtsSize = 4*w*h*sizeof(float);
      stIm->imPts = (float *)MEMALIGN(stIm->imPtsSize);
      MEMFREE(stIm->imPtsColor);
      stIm->imPtsColor = (uint8_t *)MEMALIGN(3*w*h);
    }

  float *pt;
  int y = iy;
  float cx = (float)stIm->RP[3];
  float cy = (float)stIm->RP[7];
  float f  = (float)stIm->RP[11];
  float itx = (float)stIm->RP[14];
  itx *= 1.0 / (float)stIm->dpp; // adjust for subpixel interpolation
  pt = stIm->imPts;
      
  for (int j=0; j<ih; j++, y++)
    {
      int x = ix;
      int16_t *p = stIm->imDisp + x + y*w;

      for (int i=0; i<iw; i++, x++, p++)
	{
	  if (*p > 0) 
	    {
	      float ax = (float)x + cx;
	      float ay = (float)y + cy;
	      float aw = 1.0 / (itx * (float)*p);
	      *pt++ = ax*aw;	// X
	      *pt++ = ay*aw;	// Y
	      *pt++ = f*aw;	// Z
	      stIm->numPts++;
	    }
	}
    }

  if (stIm->imLeft->imRectColorType != COLOR_CODING_NONE) // ok, have color
    {
      y = iy;
      uint8_t *pcout = stIm->imPtsColor;
      for (int j=0; j<ih; j++, y++)
	{
	  int x = ix;
	  int16_t *p = stIm->imDisp + x + y*w;
	  uint8_t *pc = stIm->imLeft->imRectColor + (x + y*w)*3;

	  for (int i=0; i<iw; i++, x++, p++, pc+=3)
	    {
	      if (*p > 0) 
		{
		  *pcout++ = *pc;
		  *pcout++ = *(pc+1);
		  *pcout++ = *(pc+2);
		}
	    }
	}
    }
  else if (stIm->imLeft->imRectType != COLOR_CODING_NONE) // ok, have mono
    {
      y = iy;
      uint8_t *pcout = stIm->imPtsColor;
      for (int j=0; j<ih; j++, y++)
	{
	  int x = ix;
	  int16_t *p = stIm->imDisp + x + y*w;
	  uint8_t *pc = stIm->imLeft->imRect + (x + y*w);

	  for (int i=0; i<iw; i++, p++, pc++)
	    {
	      if (*p > 0) 
		{
		  *pcout++ = *pc;
		  *pcout++ = *pc;
		  *pcout++ = *pc;
		}
	    }
	}
    }


  //  printf("[Calc Pts] Number of pts: %d\n", stIm->numPts);
  return true;
}




// StereoDcam class
// Conjoins image grabbing and stereo
//

StereoDcam::StereoDcam(uint64_t guid, size_t buffersize)
  : Dcam(guid,buffersize)
{
  // currently just set up a Videre stereo cam
  if (isVidereStereo)
    {
      char *params;
      params = getParameters(); // ok, these are the params

      if (params == NULL)
	{
	  PRINTF("Whoa nellie - no params!\n");
	}
      else			// parse out the stereo params
	stIm->extractParams(params);
    }
}


StereoDcam::~StereoDcam()
{
}
 
// format of image

void 
StereoDcam::setFormat(dc1394video_mode_t video, dc1394framerate_t fps,
		      dc1394speed_t speed)
{
  stIm->releaseBuffers();	// release all buffers in the stereo data structure
  Dcam::setFormat(video, fps, speed);
}



// starting and stopping

void
StereoDcam::start()
{
  Dcam::start();
  Dcam::setCompanding(true);	// must be set after starting camera
  Dcam::setUniqueThresh(stIm->uniqueThresh);
  Dcam::setTextureThresh(stIm->textureThresh);
}

void
StereoDcam::stop()
{
  Dcam::stop();
}


// other stuff

bool
StereoDcam::getImage(int ms)	// gets the next image, with timeout
{
  bool ret = Dcam::getImage(ms);
  if (!ret)			// did we get a good image?
    return false;

  // image size
  stIm->setSize(camIm->imWidth,camIm->imHeight);

  // zero out image data indicators
  stIm->imLeft->imType = COLOR_CODING_NONE;
  stIm->imLeft->imColorType = COLOR_CODING_NONE;
  stIm->imLeft->imRectType = COLOR_CODING_NONE;
  stIm->imLeft->imRectColorType = COLOR_CODING_NONE;
  stIm->imRight->imType = COLOR_CODING_NONE;
  stIm->imRight->imColorType = COLOR_CODING_NONE;
  stIm->imRight->imRectType = COLOR_CODING_NONE;
  stIm->imRight->imRectColorType = COLOR_CODING_NONE;
  stIm->hasDisparity = false;

  // check for single-device stereo, and process
  if (isVidereStereo)
    {
      switch (rawType)
	{
	case VIDERE_STOC_RECT_RECT:
	  stereoDeinterlace(camIm->imRaw, &stIm->imLeft->imRect, &stIm->imLeft->imRectSize,
			    &stIm->imRight->imRect, &stIm->imRight->imRectSize);
	  stIm->imLeft->imRectType = COLOR_CODING_MONO8;
	  stIm->imRight->imRectType = COLOR_CODING_MONO8;
	  break;

	case VIDERE_STOC_RAW_RAW_MONO:
	case VIDERE_STEREO_MONO:
	  stereoDeinterlace(camIm->imRaw, &stIm->imLeft->im, &stIm->imLeft->imSize, 
			    &stIm->imRight->im, &stIm->imRight->imSize);
	  stIm->imLeft->imType = COLOR_CODING_MONO8;
	  stIm->imRight->imType = COLOR_CODING_MONO8;
	  stIm->imLeft->doBayerMono();
	  stIm->imRight->doBayerMono();
	  break;

	case VIDERE_STOC_RAW_RAW_RGGB:
	case VIDERE_STEREO_RGGB:
	case VIDERE_STEREO_BGGR:
	case VIDERE_STEREO_GRBG:
	  stereoDeinterlace(camIm->imRaw, &stIm->imLeft->imRaw, &stIm->imLeft->imRawSize,
			    &stIm->imRight->imRaw, &stIm->imRight->imRawSize);
	  stIm->imLeft->imRawType = COLOR_CODING_BAYER8_RGGB;
	  stIm->imRight->imRawType = COLOR_CODING_BAYER8_RGGB;
	  stIm->imLeft->doBayerColorRGB();
	  stIm->imRight->doBayerColorRGB();
	  break;


	case VIDERE_STOC_RECT_DISP:
	  stereoDeinterlace2(camIm->imRaw, &stIm->imLeft->imRect, &stIm->imLeft->imRectSize, 
			    &stIm->imDisp, &stIm->imDispSize);
	  stIm->imLeft->imRectType = COLOR_CODING_MONO8;
	  stIm->hasDisparity = true;
	  break;

	case VIDERE_STOC_RAW_DISP_MONO:
	  stereoDeinterlace2(camIm->imRaw, &stIm->imLeft->im, &stIm->imLeft->imSize, 
			    &stIm->imDisp, &stIm->imDispSize);
	  stIm->imLeft->imType = COLOR_CODING_MONO8;
	  stIm->imLeft->doBayerMono();
	  stIm->hasDisparity = true;
	  break;

	case VIDERE_STOC_RAW_DISP_RGGB:
	  stereoDeinterlace2(camIm->imRaw, &stIm->imLeft->imRaw, &stIm->imLeft->imRawSize, 
			    &stIm->imDisp, &stIm->imDispSize);
	  stIm->imLeft->imRawType = COLOR_CODING_BAYER8_RGGB;
	  stIm->imLeft->doBayerColorRGB();
	  stIm->hasDisparity = true;
	  break;

	default:
	  break;
	}
    }

  return ret;
}

void
StereoDcam::setCapturePolicy(dc1394capture_policy_t policy)
{
  Dcam::setCapturePolicy(policy);
}

void
StereoDcam::setFeature(dc1394feature_t feature, uint32_t value, uint32_t value2)
{
  Dcam::setFeature(feature, value, value2);
}


void 
StereoDcam::setFeatureAbsolute(dc1394feature_t feature, float value)
{
  Dcam::setFeatureAbsolute(feature, value);
}


void 
StereoDcam::setFeatureMode(dc1394feature_t feature, dc1394feature_mode_t mode)
{
  Dcam::setFeatureMode(feature, mode);
}


void 
StereoDcam::setRegister(uint64_t offset, uint32_t value)
{
  Dcam::setRegister(offset, value);
}


bool
StereoDcam::setTextureThresh(int thresh)
{
  StereoCam::setTextureThresh(thresh);
  return Dcam::setTextureThresh(thresh);
}

bool
StereoDcam::setUniqueThresh(int thresh)
{
  StereoCam::setUniqueThresh(thresh);
  return Dcam::setUniqueThresh(thresh);
}


//
// utility functions
//


// de-interlace stereo data, reserving storage if necessary

void
StereoDcam::stereoDeinterlace(uint8_t *src, uint8_t **d1, size_t *s1, 
			     uint8_t **d2, size_t *s2)
{
  size_t size = stIm->imWidth*stIm->imHeight;
  if (*s1 < size)		// need to check alignment here...
    {
      MEMFREE(*d1);
      *d1 = (uint8_t *)MEMALIGN(size);
      *s1 = size;
    }
  if (*s2 < size)
    {
      MEMFREE(*d2);
      *d2 = (uint8_t *)MEMALIGN(size);
      *s2 = size;
    }
  
  uint8_t *dd1 = *d1;
  uint8_t *dd2 = *d2;
  for (int i=0; i<(int)size; i++)
    {
      *dd2++ = *src++;
      *dd1++ = *src++;
    }
}


// de-interlace stereo data, reserving storage if necessary
// second buffer is 16-bit disparity data

void
StereoDcam::stereoDeinterlace2(uint8_t *src, uint8_t **d1, size_t *s1, 
			      int16_t **d2, size_t *s2)
{
  int w = stIm->imWidth;
  int h = stIm->imHeight;
  size_t size = w*h;

  if (*s1 < size)		// need to check alignment here...
    {
      MEMFREE(*d1);
      *d1 = (uint8_t *)MEMALIGN(size);
      *s1 = size;
    }
  if (*s2 < size*2)
    {
      MEMFREE(*d2);
      *d2 = (int16_t *)MEMALIGN(size*2);
      *s2 = size*2;
    }
  
  uint8_t *dd1 = *d1;
  int16_t *dd2 = *d2;

  int dt = stIm->imDtop;
  int dl = stIm->imDleft;
  int dw = stIm->imDwidth;
  int dh = stIm->imDheight;

  /*
   * source rectangle
   * ================
   * (w-dwidth, h-dheight) => upper left
   * (dwidth, dheight)     => size
   *
   * dest rectangle
   * ==============
   * (dleft-6,dtop)        => upper left
   * (dwidth,dheight)      => size
   */
  
  dd2 += (dt*w + dl - 6) - ((h-dh)*w + w - dw);

  size = (h-dh)*w;
  for (int i=0; i<(int)size; i++)
    {
      dd2++;
      src++;
      *dd1++ = *src++;
    }

  size = dh*w;
  for (int i=0; i<(int)size; i++)
    {
      *dd2++ = ((uint16_t)*src++)<<2;
      *dd1++ = *src++;
    }
}


