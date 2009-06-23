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

#ifndef STEREOCAM_H
#define STEREOCAM_H

//
// StereoCam class
// Processes two images to create a disparity image
// Subclasses
//   StereoDcam - input from Dcam driver (IEEE1394 cameras)
//   StereoFIle - input from files
//

#include <stdexcept>
#include "image.h"
#include "dcam.h"

typedef enum
  {
    SIDE_LEFT = 0,
    SIDE_RIGHT
  } stereo_side_t;


namespace dcam
{

  class StcamException : public std::runtime_error
  {
  public:
    StcamException(const char* msg) : std::runtime_error(msg) {}
  };

  class StereoDcam
    : public Dcam
  {
  public:
    StereoDcam(uint64_t guid, size_t buffersize = 8);
    virtual ~StereoDcam();

    StereoData *stIm;		// stereo image and processing

    void setFormat(dc1394video_mode_t video = DC1394_VIDEO_MODE_640x480_YUV422,
			   dc1394framerate_t fps = DC1394_FRAMERATE_30,
			   dc1394speed_t speed = DC1394_ISO_SPEED_400);

    void start();
    void stop();

    bool getImage(int ms); // gets the next image, with timeout
    void setCapturePolicy(dc1394capture_policy_t policy = DC1394_CAPTURE_POLICY_WAIT);

    void setFeature(dc1394feature_t feature, uint32_t value, uint32_t value2 = 0);
    void setFeatureAbsolute(dc1394feature_t feature, float value);
    void setFeatureMode(dc1394feature_t feature, dc1394feature_mode_t mode);

    void setRegister(uint64_t offset, uint32_t value);
    
    // processing parameters
    bool setTextureThresh(int thresh);
    bool setUniqueThresh(int thresh);
    bool setSmoothnessThresh(int thresh);
    bool setHoropter(int thresh);
    bool setSpeckleSize(int size);
    bool setSpeckleDiff(int diff);
    bool setCorrsize(int size);
    bool setNumDisp(int ndisp);
    bool setRangeMax(double thresh);
    bool setRangeMin(double thresh);
    bool setUniqueCheck(bool unique_check);

    // visible calls to StereoData functions
    void doBayerColorRGB();
    void doBayerMono();
    bool doRectify();
    bool doDisparity(stereo_algorithm_t alg=NORMAL_ALGORITHM);
    bool doCalcPts();

  protected:
    // Videre camera de-interlacing
    void stereoDeinterlace(uint8_t *src, uint8_t **d1, size_t *s1, uint8_t **d2, size_t *s2);
    void stereoDeinterlace2(uint8_t *src, uint8_t **d1, size_t *s1, int16_t **d2, size_t *s2);

  protected:
    Dcam *rcam;			// right camera of two-camera setup

  };

}

#endif // STEREOCAM_H
