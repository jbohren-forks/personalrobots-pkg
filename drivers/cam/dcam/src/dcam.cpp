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
// Basic driver for grabbing frames and controlling IIDC 1.3x cameras
// Uses libdc1394 under Linux
// Uses CMU driver under MSW
//

#include "dcam.h"

#include <errno.h>

#define PRINTF(a...) printf(a)

#define CHECK_READY() \
  if (!dcam::dcRef) { \
    char msg[256]; \
    snprintf(msg, 256, "Tried to call %s before calling dcam::init()", __FUNCTION__); \
    throw DcamException(msg); \
  }

#define CHECK_ERR(fnc, amsg) \
  { \
  dc1394error_t err = fnc; \
  if (err != DC1394_SUCCESS) { \
    char msg[256]; \
    snprintf(msg, 256, "%s: %s", dc1394_error_get_string(err), amsg);        \
    throw DcamException(msg); \
  } \
  }

#define CHECK_ERR_CLEAN(fnc, amsg) \
  { \
  dc1394error_t err = fnc; \
  if (err != DC1394_SUCCESS) { \
    cleanup(); \
    char msg[256]; \
    snprintf(msg, 256, "%s: %s", dc1394_error_get_string(err), amsg);        \
    throw DcamException(msg); \
  }\
  }


dc1394_t *dcam::dcRef = NULL;	// system object, dc1394_t* for Linux

void
dcam::init() 
{
  if (dcam::dcRef == NULL)
  {
    dcam::dcRef = dc1394_new();

    if (numCameras() > 0)
    {
      dc1394camera_t *camera = dc1394_camera_new((dc1394_t *)dcam::dcRef, getGuid(0));
      if (!camera) {
        char msg[256];
        snprintf(msg, 256, "Could not acquire camera to reset bus in %s", __FUNCTION__);
        throw DcamException(msg);
      }
      
      dc1394_reset_bus(camera);
      dc1394_camera_free(camera);
      dc1394_free((dc1394_t *)dcam::dcRef);

      dcam::dcRef = dc1394_new();
    }

    usleep(500000);

  }
}

void
dcam::fini()
{
  if (dcam::dcRef != NULL)
  {
    dc1394_free((dc1394_t*)dcam::dcRef);
  }
}


size_t dcam::numCameras()
{
  CHECK_READY();

  dc1394camera_list_t * list;
  CHECK_ERR( dc1394_camera_enumerate(dcam::dcRef, &list), "Could not enumerate cameras" );

  size_t num = list->num;

  dc1394_camera_free_list (list);

  return num;
}


uint64_t 
dcam::getGuid(size_t i)
{
  if (!dcam::dcRef) {
    char msg[256];
    snprintf(msg, 256, "Tried to call %s before calling dcam::init()", __FUNCTION__);
    throw DcamException(msg);
  }

  dc1394camera_list_t * list;
  CHECK_ERR( dc1394_camera_enumerate(dcam::dcRef, &list), "Could not enumerate cameras" );

  if (i >= list->num)
    throw DcamException("Tried to get Guid of non-existant camera");

  uint64_t guid = list->ids[i].guid;

  dc1394_camera_free_list (list);

  return guid;
}


// Model name

char *
dcam::getModel(size_t i)
{
  char *name = NULL;
  CHECK_READY();

  if (i < numCameras())
    {
      dc1394camera_t *camera = dc1394_camera_new(dcam::dcRef, getGuid(i));
      if (!camera) 
	{
	  char msg[256];
	  snprintf(msg, 256, "Could not acquire camera %d %s", i, __FUNCTION__);
	  throw DcamException(msg);
	}
      
      if (camera->model)
	name = strdup(camera->model);

      dc1394_camera_free(camera);
    }

  return name;
}


// Vendor name

char *
dcam::getVendor(size_t i)
{
  char *name = NULL;
  CHECK_READY();

  if (i < numCameras())
    {
      dc1394camera_t *camera = dc1394_camera_new(dcam::dcRef, getGuid(i));
      if (!camera) 
	{
	  char msg[256];
	  snprintf(msg, 256, "Could not acquire camera %d %s", i, __FUNCTION__);
	  throw DcamException(msg);
	}
      
      if (camera->vendor)
	name = strdup(camera->vendor);
      dc1394_camera_free(camera);
    }

  return name;
}


static char *modestrings[DC1394_VIDEO_MODE_NUM] =
  {
    "DC1394_VIDEO_MODE_160x120_YUV444",
    "DC1394_VIDEO_MODE_320x240_YUV422",
    "DC1394_VIDEO_MODE_640x480_YUV411",
    "DC1394_VIDEO_MODE_640x480_YUV422",
    "DC1394_VIDEO_MODE_640x480_RGB8",
    "DC1394_VIDEO_MODE_640x480_MONO8",
    "DC1394_VIDEO_MODE_640x480_MONO16",
    "DC1394_VIDEO_MODE_800x600_YUV422",
    "DC1394_VIDEO_MODE_800x600_RGB8",
    "DC1394_VIDEO_MODE_800x600_MONO8",
    "DC1394_VIDEO_MODE_1024x768_YUV422",
    "DC1394_VIDEO_MODE_1024x768_RGB8",
    "DC1394_VIDEO_MODE_1024x768_MONO8",
    "DC1394_VIDEO_MODE_800x600_MONO16",
    "DC1394_VIDEO_MODE_1024x768_MONO16",
    "DC1394_VIDEO_MODE_1280x960_YUV422",
    "DC1394_VIDEO_MODE_1280x960_RGB8",
    "DC1394_VIDEO_MODE_1280x960_MONO8",
    "DC1394_VIDEO_MODE_1600x1200_YUV422",
    "DC1394_VIDEO_MODE_1600x1200_RGB8",
    "DC1394_VIDEO_MODE_1600x1200_MONO8",
    "DC1394_VIDEO_MODE_1280x960_MONO16",
    "DC1394_VIDEO_MODE_1600x1200_MONO16",
    "DC1394_VIDEO_MODE_EXIF",
    "DC1394_VIDEO_MODE_FORMAT7_0",
    "DC1394_VIDEO_MODE_FORMAT7_1",
    "DC1394_VIDEO_MODE_FORMAT7_2",
    "DC1394_VIDEO_MODE_FORMAT7_3",
    "DC1394_VIDEO_MODE_FORMAT7_4",
    "DC1394_VIDEO_MODE_FORMAT7_5",
    "DC1394_VIDEO_MODE_FORMAT7_6",
    "DC1394_VIDEO_MODE_FORMAT7_7"
  };

// mode strings from mode
char *
dcam::getModeString(dc1394video_mode_t mode)
{
  if (mode < DC1394_VIDEO_MODE_MAX)
    return modestrings[mode-DC1394_VIDEO_MODE_MIN];
  else
    return "";
}


// Set up a camera object

dcam::Dcam::Dcam(uint64_t guid, size_t bsize)
{
  CHECK_READY();
  dcCam = dc1394_camera_new(dcRef, guid);

  if (!dcCam)
    throw DcamException("Could not create camera");

  CHECK_ERR( dc1394_video_get_supported_modes(dcCam, &camModes), 
	     "Could not get supported modes" );

  bufferSize = bsize;
  camPolicy = DC1394_CAPTURE_POLICY_POLL;
  camFrame = NULL;
  camIm = new ImageData();
  camIm->params = NULL;
  camIm->imRaw = NULL;
  camIm->im = NULL;
  camIm->imColor = NULL;
  camIm->imRect = NULL;
  camIm->imRectColor = NULL;
  camIm->imWidth = 0;
  camIm->imHeight = 0;
  isSTOC = false;
  isVidereStereo = false;
  isColor = false;
  procMode = PROC_MODE_RECTIFIED;

  //  dc1394_camera_print_info(dcCam,stdout);

  // Check Videre camera type and local params
  if (!strcmp(getModel(),"MDS-STH")) // Videre-type camera
    {
      isVidereStereo = true;

      PRINTF("[dcam] Videre camera, getting local params\n");
      uint32_t qval;

      // firmware level
      qval = getRegister(VIDERE_LOCAL_PARAM_BASE+VIDERE_CAM_FW_LEVEL_OFFSET);
      int major = (qval & 0x0000ff00)>>8;
      int minor = (qval & 0x000000ff);

      if ((qval >> 16) != 0 || major < 2 || minor > 10)	// check for local parameters
	PRINTF("[dcam] No local parameters");
      else
	{
	  // Camera and imager firmware
	  camFirmware = qval & 0xffff;
	  PRINTF("[dcam] Camera firmware: %02d.%02d\n", major, minor);
	  qval = getRegister(VIDERE_LOCAL_PARAM_BASE+VIDERE_CAM_LEVEL_OFFSET);      
	  imFirmware = qval & 0xff;
	  PRINTF("[dcam] Imager firmware: %04x\n", imFirmware);

	  // STOC
	  qval = getRegister(VIDERE_LOCAL_PARAM_BASE+VIDERE_CAM_PROC_OFFSET);
	  stocFirmware = (qval & 0xffff00) >> 8;
	  major = (stocFirmware & 0xff00)>>8;
	  minor = stocFirmware & 0xff;
	  PRINTF("[dcam] STOC version: %02d.%02d\n", major, minor);
	  if (major > 0 && major < 0xff && minor != 0xff) // check for odd firmware values
	    {
	      isSTOC = true;
	      procMode = (videre_proc_mode_t)(qval & 0x000000ff);

	      // this sets the Config bits on faulty FPGA firmware (version 4.1 and below)
	      qval = 0x08000000 | (0x9C << 16);
	      setRegister(VIDERE_LOCAL_PARAM_BASE, qval);
	    }

	  // STOC thresholds
	  qval = getRegister(VIDERE_LOCAL_PARAM_BASE+VIDERE_CAM_PROC_THRESH_OFFSET);
	  PRINTF("[dcam] STOC thresholds: %08x\n", qval);

	  // parameter string
	  if (getParameters() != NULL)
	    PRINTF("[dcam] Calibration, %d bytes\n", strlen(camIm->params));
	  else
	    PRINTF("[dcam] No calibration\n");

	}
    }

  // check for color/monochrome camera
  isColor = false;
  if (hasFeature(DC1394_FEATURE_WHITE_BALANCE))
    {
      isColor = true;
      PRINTF("[dcam] Color device\n");
    }
  else
    PRINTF("[dcam] Monochrome device\n");

  setRawType();

  //  dc1394_iso_release_bandwidth(dcCam, 10000000);

  //  CHECK_ERR_CLEAN( dc1394_reset_bus(dcCam), "Could not reset bus" );

}


// Tear down camera object

dcam::Dcam::~Dcam()
{
  if (dcCam != NULL)
    cleanup();
  free(camIm);
}

void
dcam::Dcam::cleanup()
{
  dc1394_video_set_transmission(dcCam, DC1394_OFF);
  dc1394_capture_stop(dcCam);
  dc1394_camera_free(dcCam);
  dcCam = NULL;
}



// Return a list of modes

dc1394video_modes_t *
dcam::Dcam::getModes()
{
  CHECK_READY();
  return &camModes;
}


// Model name

char *
dcam::Dcam::Dcam::getModel()
{
  return dcCam->model;
}


// Vendor name

char *
dcam::Dcam::getVendor()
{
  return dcCam->vendor;
}


// Set up image format

void
dcam::Dcam::setFormat(dc1394video_mode_t video,
		      dc1394framerate_t fps, 
		      dc1394speed_t speed)
{
  dc1394_capture_stop(dcCam);	// tear down any previous capture setup
  
  // check for valid video mode
  size_t i;
  for (i=0; i<camModes.num; i++)
    {
      if (camModes.modes[i] == video)
	break;
    }

  if (i >= camModes.num)	// oops, haven't found it
    {
      char msg[256];
      snprintf(msg, 256, "setFormat: not a valid mode: %s", 
	       getModeString(video));
      throw DcamException(msg);
    }

  CHECK_ERR_CLEAN( dc1394_video_set_mode(dcCam, video),
		   "Could not set video mode");
  CHECK_ERR_CLEAN( dc1394_video_set_iso_speed(dcCam, speed),
		   "Could not set iso speed");
  CHECK_ERR_CLEAN( dc1394_video_set_framerate(dcCam, fps), 
		   "Could not set framerate");
  CHECK_ERR_CLEAN( dc1394_capture_setup(dcCam, bufferSize, DC1394_CAPTURE_FLAGS_DEFAULT), 
		   "Could not setup camera.");
  setRawType();
}


// Start and stop streaming

void
dcam::Dcam::start()
{
  CHECK_READY();
  CHECK_ERR_CLEAN( dc1394_video_set_transmission(dcCam, DC1394_ON), 
		   "Could not start camera iso transmission");

  // now check if we have started transmission, no error from set_transmission
  dc1394switch_t pwr;
  dc1394_video_get_transmission(dcCam, &pwr);
  if (pwr == DC1394_OFF)
    throw DcamException("Could not start ISO transmission");

  started = true;
}



void
dcam::Dcam::stop()
{
  if (camFrame)
    CHECK_ERR_CLEAN( dc1394_capture_enqueue(dcCam, camFrame), "Could not release frame");    

  CHECK_READY();
  CHECK_ERR_CLEAN( dc1394_video_set_transmission(dcCam, DC1394_OFF),
		   "Could not stop camera iso transmission");

  started = false;
}


// Getting images
// Waits for the next image available, up to ms for timeout
//   Assumes capture policy of POLL
// Stores the next available image into the class instance

bool
dcam::Dcam::getImage(int ms)
{
    CHECK_READY();

    if (!started)
      return false;

    // release previous frame, if it exists
    if (camFrame)
      CHECK_ERR_CLEAN( dc1394_capture_enqueue(dcCam, camFrame), 
		       "Could not release frame");

    camFrame = NULL;

    // get the image
    while (1)
      {
	CHECK_ERR_CLEAN( dc1394_capture_dequeue(dcCam, camPolicy, &camFrame), 
		       "Could not capture frame");

	if (camFrame == NULL)
	  {
	    if (ms <= 0) break;
	    ms -= 10;
	    usleep(10000);
	  }
	else
	  {
	    //	    break;
	    while (1)		// flush the buffer, get latest one
	      {
		dc1394video_frame_t *f = NULL;
		dc1394_capture_dequeue(dcCam, camPolicy, &f);
		if (f != NULL)
		  {
		    dc1394_capture_enqueue(dcCam,camFrame);
		    camFrame = f;
		  }
		else
		  break;
	      }
	    break;
	  }
      }

    // transfer info
    if (camFrame)
      {
	// clear everything out first
	camIm->imRaw = NULL;
	camIm->imRawType = rawType;
	camIm->im = NULL;
	camIm->imType = COLOR_CODING_NONE;
	camIm->imColor = NULL;
	camIm->imColorType = COLOR_CODING_NONE;
	camIm->imRect = NULL;
	camIm->imRectType = COLOR_CODING_NONE;
	camIm->imRectColor = NULL;
	camIm->imRectColorType = COLOR_CODING_NONE;

	// check raw modes
	if (rawType != COLOR_CODING_NONE)
	  camIm->imRaw = camFrame->image;	    
	else			// ???
	  camIm->im = camFrame->image;

	camIm->imWidth = camFrame->size[0];
	camIm->imHeight = camFrame->size[1];
	camIm->im_time = camFrame->timestamp;
//	printf("Time: %llu\n", camFrame->timestamp);
      }

#if 0
    if (camFrame != NULL &&
        (camFrame->frames_behind > 1  || 
         dc1394_capture_is_frame_corrupt(dcCam, camFrame) == DC1394_TRUE)
        )
      {
	dc1394_capture_enqueue(dcCam, camFrame);
	camFrame = NULL;
      }
#endif

    return (camFrame != NULL);
}



void 
dcam::Dcam::setCapturePolicy(dc1394capture_policy_t p)
{
  camPolicy = p;
}


// Features

bool
dcam::Dcam::hasFeature(dc1394feature_t feature)
{
  CHECK_READY();
  dc1394bool_t present;
  CHECK_ERR_CLEAN( dc1394_feature_is_present(dcCam, feature, &present), 
		   "Could not check if feature was present");
  return (present == DC1394_TRUE);
}

void
dcam::Dcam::setFeature(dc1394feature_t feature, uint32_t value, uint32_t value2)
{
  CHECK_READY();
  if (feature == DC1394_FEATURE_WHITE_BALANCE)
  {
    CHECK_ERR_CLEAN( dc1394_feature_whitebalance_set_value_blind(dcCam, value, value2), "Could not set feature");
  }
  else
  {
    CHECK_ERR_CLEAN( dc1394_feature_set_value_blind(dcCam, feature, value), "Could not set feature");
  }
}

void
dcam::Dcam::getFeatureBoundaries(dc1394feature_t feature, uint32_t& min, uint32_t& max)
{
  CHECK_READY();
  dc1394bool_t present;
  CHECK_ERR_CLEAN( dc1394_feature_is_present(dcCam, feature, &present), 
		   "Could not check if feature was present");
  if (present == DC1394_TRUE)
  {
    CHECK_ERR_CLEAN( dc1394_feature_get_boundaries(dcCam, feature, &min, &max),
		     "Could not find feature boundaries");
  }
}

void
dcam::Dcam::setFeatureAbsolute(dc1394feature_t feature, float value)
{
  CHECK_READY();
  dc1394bool_t present;
  CHECK_ERR_CLEAN( dc1394_feature_is_present(dcCam, feature, &present),
		   "Could not check if feature was present");
  if (present == DC1394_TRUE)
  {
    CHECK_ERR_CLEAN( dc1394_feature_set_absolute_control(dcCam, feature,  DC1394_ON),
		     "Could not enable absolute control.");
    CHECK_ERR_CLEAN( dc1394_feature_set_absolute_value(dcCam, feature, value),
		     "Could not set feature");
  }
}

void
dcam::Dcam::setFeatureMode(dc1394feature_t feature, dc1394feature_mode_t mode)
{
  CHECK_READY();
  CHECK_ERR_CLEAN( dc1394_feature_set_mode_blind(dcCam, feature, mode), "Could not set feature");
}


void
dcam::Dcam::setRegister(uint64_t offset, uint32_t value)
{
  CHECK_READY();
  CHECK_ERR_CLEAN( dc1394_set_control_register(dcCam, offset, value), 
		   "Could not set control register");
}

uint32_t
dcam::Dcam::getRegister(uint64_t offset)
{
  CHECK_READY();
  uint32_t value;
  CHECK_ERR_CLEAN( dc1394_get_control_register(dcCam, offset, &value),
		   "Could not get control register");
  return value;
}


// STOC modes

bool
dcam::Dcam::setProcMode(videre_proc_mode_t mode)
{
  CHECK_READY();
  if (!isSTOC)
    return false;

  procMode = mode;

  // set it while running
  uint32_t qval1 = 0x08000000 | (0x90 << 16) | ( ( mode & 0x7) << 16);
  uint32_t qval2 = 0x08000000 | (0x9C << 16);
  
  setRegister(0xFF000, qval1);
  setRegister(0xFF000, qval2);

  // set it while stopped
  uint32_t qval = (stocFirmware << 8) | procMode;
  setRegister(VIDERE_LOCAL_PARAM_BASE+VIDERE_CAM_PROC_OFFSET,qval);

  setRawType();			// set up image type

  return true;
}


// Raw type
void
dcam::Dcam::setRawType()
{
  if (isSTOC)
    {
      switch (procMode)
	{
	case PROC_MODE_OFF:
	case PROC_MODE_NONE:
	case PROC_MODE_TEST:
	  if (isColor)
	    rawType = VIDERE_STOC_RAW_RAW_RGGB;
	  else
	    rawType = VIDERE_STOC_RAW_RAW_MONO;
	  break;

	case PROC_MODE_RECTIFIED:
	  rawType = VIDERE_STOC_RECT_RECT;
	  break;
		
	case PROC_MODE_DISPARITY:
	  rawType = VIDERE_STOC_RECT_DISP;
	  break;

	case PROC_MODE_DISPARITY_RAW:
	  if (isColor)
	    rawType = VIDERE_STOC_RAW_DISP_RGGB;
	  else
	    rawType = VIDERE_STOC_RAW_DISP_MONO;
	  break;
	}
    }
  else if (isVidereStereo) // stereo device
    {
      camIm->imRaw = camFrame->image;
      if (isColor)
	rawType = VIDERE_STEREO_RGGB;
      else
	rawType = VIDERE_STEREO_MONO;
    }
  else
    rawType = COLOR_CODING_NONE;
}


// Parameters

char *
dcam::Dcam::getParameters()
{
  if (camIm->params)
    free(camIm->params);

  uint32_t qval = getRegister(VIDERE_LOCAL_BASE+VIDERE_CALIB_OFFSET);
  if (qval == 0xffffffff)
    camIm->params = NULL;
  else
    {
      char *buf = new char[4096*4];
      int n = 4096*4;
      char* bb = buf;
      
      // read in each byte
      int pos = 0;
      uint32_t quad;
      quad = getRegister(VIDERE_LOCAL_BASE+VIDERE_CALIB_OFFSET+pos);

      while (quad != 0x0 && quad != 0xffffffff && n > 3)
	{
	  int val;
	  pos += 4;
	  n -= 4;
	  val = (quad >> 24) & 0xff;
	  *bb++ = val;
	  val = (quad >> 16) & 0xff;
	  *bb++ = val;
	  val = (quad >> 8) & 0xff;
	  *bb++ = val;
	  val = quad & 0xff;
	  *bb++ = val;
	  quad = getRegister(VIDERE_LOCAL_BASE+VIDERE_CALIB_OFFSET+pos);
	}
      *bb = 0;	       // just in case we missed the last zero
      camIm->params = buf;
//    PRINTF(buf);
    }
  return camIm->params;
}

bool
dcam::Dcam::setParameters(char *params)
{
  return false;
}


// companding and HDR

bool
dcam::Dcam::setCompanding(bool on)
{
  usleep(50000);

  if (on)
    setRegister(0xFF000, 0x041C0003);
  else
    setRegister(0xFF000, 0x041C0002);

  return true;
}

bool
dcam::Dcam::setHDR(bool on)
{
  usleep(50000);

  if (on)
    setRegister(0xFF000, 0x040F0051);
  else
    setRegister(0xFF000, 0x040F0011);

  return true;
}


// thresholds

bool
dcam::Dcam::setTextureThresh(int thresh)
{
  usleep(50000);

  if (thresh < 0)
    thresh = 0;
  if (thresh > 63)
    thresh = 63;

  uint32_t t_thresh = 0x08000000 | (0x40 << 16) | ( thresh << 16);
  setRegister(0xFF000, t_thresh);

  return true;
}

bool
dcam::Dcam::setUniqueThresh(int thresh)
{
  usleep(50000);

  if (thresh < 0)
    thresh = 0;
  if (thresh > 63)
    thresh = 63;

  uint32_t u_thresh = 0x08000000 | (0x00 << 16) | ( thresh << 16);
  setRegister(0xFF000, u_thresh);

  return true;
}
