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

#include "dc1394_cam/dc1394_cam.h"
#include <errno.h>


#define CHECK_READY() \
  if (!dcRef) { \
    char msg[256]; \
    snprintf(msg, 256, "Tried to call %s before calling dc1394_cam::Cam::init()", __FUNCTION__); \
    throw CamException(msg); \
  }

#define CHECK_ERR(fnc, amsg) \
  { \
  dc1394error_t err = fnc; \
  if (err != DC1394_SUCCESS) { \
    char msg[256]; \
    snprintf(msg, 256, "%s: %s", dc1394_error_get_string(err), amsg);        \
    throw CamException(msg); \
  } \
  }

#define CHECK_ERR_CLEAN(fnc, amsg) \
  { \
  dc1394error_t err = fnc; \
  if (err != DC1394_SUCCESS) { \
    cleanup(); \
    char msg[256]; \
    snprintf(msg, 256, "%s: %s", dc1394_error_get_string(err), amsg);        \
    throw CamException(msg); \
  }\
  }



dc1394_t* dc1394_cam::Cam::dcRef = NULL;
fd_set dc1394_cam::Cam::camFds;


void
dc1394_cam::init() {
  if (dc1394_cam::Cam::dcRef == NULL)
  {

    dc1394_cam::Cam::dcRef = dc1394_new();
    FD_ZERO(&Cam::camFds);

    if (numCams() > 0)
    {
       dc1394camera_t *camera = dc1394_camera_new(dc1394_cam::Cam::dcRef, getGuid(0));
      if (!camera) {
        char msg[256];
        snprintf(msg, 256, "Could not acquire camera to reset bus in %s", __FUNCTION__);
        throw CamException(msg);
      }
      
      dc1394_reset_bus (camera);
      dc1394_camera_free (camera);

      dc1394_free (dc1394_cam::Cam::dcRef);

      dc1394_cam::Cam::dcRef = dc1394_new();
    }

    usleep(500000);

  }
}


void
dc1394_cam::fini()
{
  if (dc1394_cam::Cam::dcRef != NULL)
  {
    dc1394_free(dc1394_cam::Cam::dcRef);
  }
}


size_t dc1394_cam::numCams()
{
  if (!dc1394_cam::Cam::dcRef) {
    char msg[256];
    snprintf(msg, 256, "Tried to call %s before calling dc1394_cam::Cam::init()", __FUNCTION__);
    throw CamException(msg);
  }

  dc1394camera_list_t * list;
  CHECK_ERR( dc1394_camera_enumerate(dc1394_cam::Cam::dcRef, &list), "Could not enumerate cameras" );

  size_t num = list->num;

  dc1394_camera_free_list (list);

  return num;
}


uint64_t dc1394_cam::getGuid(size_t i)
{
  if (!dc1394_cam::Cam::dcRef) {
    char msg[256];
    snprintf(msg, 256, "Tried to call %s before calling dc1394_cam::Cam::init()", __FUNCTION__);
    throw CamException(msg);
  }

  dc1394camera_list_t * list;
  CHECK_ERR( dc1394_camera_enumerate(dc1394_cam::Cam::dcRef, &list), "Could not enumerate cameras" );

  if (i >= list->num)
    throw CamException("Tried to get Guid of non-existant camera");

  uint64_t guid = list->ids[i].guid;

  dc1394_camera_free_list (list);

  return guid;
}


bool dc1394_cam::waitForData(int usec)
{
  struct timeval timeout;
  timeout.tv_sec = usec / 1000000;
  timeout.tv_usec = usec % 1000000;

  fd_set fds = dc1394_cam::Cam::camFds;

  int retval = select(FD_SETSIZE, &fds, NULL, NULL, &timeout);
  
  if (retval < 0)
  {
    fprintf(stderr, "Select() returned error: %s", strerror(errno));
  }

  if (retval <= 0)
    return false;

  return true;
}


dc1394_cam::FrameWrapper
dc1394_cam::debayerFrame(FrameWrapper& f, dc1394color_filter_t bayer, dc1394bayer_method_t method)
{
  dc1394video_frame_t* f2 = (dc1394video_frame_t*)calloc(1,sizeof(dc1394video_frame_t));
  f.getFrame()->color_filter = bayer;

  if (dc1394_debayer_frames(f.getFrame(), f2, DC1394_BAYER_METHOD_BILINEAR) != DC1394_SUCCESS)
    throw CamException("Debayering frame failed");
  else
  {
    FrameWrapper fw(f.getName(), f2, f.getParent(), FRAME_OWNS_BOTH);
    return fw;
  }
}


void
dc1394_cam::initUndistortFrame(FrameWrapper& fw,
                               CvMat *intrinsic, CvMat *distortion, CvMat *rectification, CvMat *rectified_intrinsic,
                               IplImage **mapx, IplImage **mapy)
{
  if (*mapx)
    cvReleaseImage(mapx);
  if (*mapy)
    cvReleaseImage(mapy);

  *mapx = cvCreateImage( cvSize(fw.getFrame()->size[0], fw.getFrame()->size[1]), IPL_DEPTH_32F, 1 );
  *mapy = cvCreateImage( cvSize(fw.getFrame()->size[0], fw.getFrame()->size[1]), IPL_DEPTH_32F, 1 );        
  
  cvInitUndistortRectifyMap(intrinsic,
                            distortion,
                            rectification,
                            rectified_intrinsic,
                            *mapx,
                            *mapy);  
}



dc1394_cam::FrameWrapper
dc1394_cam::undistortFrame(FrameWrapper& f1, IplImage *mapx, IplImage *mapy)
{
  dc1394video_frame_t* dcf2 = (dc1394video_frame_t*)calloc(1,sizeof(dc1394video_frame_t));
  *dcf2 = *(f1.getFrame());
  dcf2->image = (unsigned char*)malloc(dcf2->total_bytes);

  int channels = 1;
  
  if (f1.getFrame()->color_coding == DC1394_COLOR_CODING_MONO8)
    channels = 1;
  else if (f1.getFrame()->color_coding == DC1394_COLOR_CODING_RGB8)
    channels = 3;
  
  IplImage* cv_img1 = cvCreateImageHeader(cvSize(f1.getFrame()->size[0], f1.getFrame()->size[1]), IPL_DEPTH_8U, channels);
  cv_img1->imageData = (char*)(f1.getFrame()->image);
  cv_img1->imageSize = f1.getFrame()->image_bytes;
  
  IplImage* cv_img2 = cvCreateImageHeader(cvSize(dcf2->size[0], dcf2->size[1]), IPL_DEPTH_8U, channels);
  cv_img2->imageData = (char*)(dcf2->image);
  cv_img2->imageSize = dcf2->image_bytes;

  cvRemap(cv_img1, cv_img2, mapx, mapy);
    
  cvReleaseImageHeader(&cv_img1);
  cvReleaseImageHeader(&cv_img2);

  return FrameWrapper(f1.getName(), dcf2, f1.getParent(), FRAME_OWNS_BOTH);
}


void
dc1394_cam::FrameWrapper::releaseFrame()
{
  if (frame_ != NULL)
  {
    if (ownership_ == CAM_OWNS_BOTH)
    {
      parent_->releaseDc1394Frame(frame_);
    } else if (ownership_ == FRAME_OWNS_BOTH)
    {
      free(frame_->image);
      free(frame_);
    } else  if (ownership_ == FRAME_OWNS_FRAME)
    {
      free(frame_);
    } 
    frame_ = NULL;
  }
}



dc1394_cam::Cam::Cam(uint64_t guid, 
                     dc1394speed_t speed, 
                     dc1394video_mode_t video,
                     dc1394framerate_t fps,
                     size_t bufferSize)        : started(false), dcCam(NULL), colorize_(false), bayer_(DC1394_COLOR_FILTER_RGGB), mapx_(NULL), mapy_(NULL)
{
  CHECK_READY();

  dcCam = dc1394_camera_new(dcRef, guid);

  if (!dcCam)
    throw CamException("Could not create camera");

  //CHECK_ERR_CLEAN( dc1394_reset_bus(dcCam), "Could not reset bus" );

  CHECK_ERR_CLEAN( dc1394_video_set_iso_speed(dcCam, speed), "Could not set iso speed");
  
  CHECK_ERR_CLEAN( dc1394_video_set_mode(dcCam, video), "Could not set video mode");
  
  CHECK_ERR_CLEAN( dc1394_video_set_framerate(dcCam, fps), "Could not set framerate");

  CHECK_ERR_CLEAN( dc1394_capture_setup(dcCam, bufferSize, DC1394_CAPTURE_FLAGS_DEFAULT), "Could not setup camera.");

  FD_SET(dc1394_capture_get_fileno(dcCam), &camFds);

  intrinsic_ = cvCreateMat(3,3,CV_32FC1);
  distortion_ = cvCreateMat(5,1,CV_32FC1);
  rectification_ = cvCreateMat(3,3,CV_32FC1);
  rectified_intrinsic_ = cvCreateMat(3,3,CV_32FC1);

  rectify_ = false;
  init_rectify_ = false;
}



dc1394_cam::Cam::~Cam()
{

  cvReleaseMat(&intrinsic_);
  cvReleaseMat(&distortion_);
  cvReleaseMat(&rectification_);
  cvReleaseMat(&rectified_intrinsic_);

  cvReleaseImage(&mapx_);
  cvReleaseImage(&mapy_);

  if (dcCam != NULL)
    cleanup();
}



void
dc1394_cam::Cam::start()
{
  CHECK_READY();

  CHECK_ERR_CLEAN( dc1394_video_set_transmission(dcCam, DC1394_ON), "Could not start camera iso transmission");

  started = true;
}



void
dc1394_cam::Cam::stop()
{
  CHECK_READY();
  CHECK_ERR_CLEAN( dc1394_video_set_transmission(dcCam, DC1394_OFF), "Could not stop camera iso transmission");

  started = false;
}


void
dc1394_cam::Cam::enableRectification(double fx, double fy, double cx, double cy, double k1, double k2, double k3, double p1, double p2)
{
  rectify_ = true;
  init_rectify_ = true;
  
  CV_MAT_ELEM(*intrinsic_, float, 0, 0) = fx;
  CV_MAT_ELEM(*intrinsic_, float, 0, 2) = cx;
  CV_MAT_ELEM(*intrinsic_, float, 1, 1) = fy;
  CV_MAT_ELEM(*intrinsic_, float, 1, 2) = cy;
  CV_MAT_ELEM(*intrinsic_, float, 2, 2) = 1;
  
  CV_MAT_ELEM(*distortion_, float, 0, 0) = k1;
  CV_MAT_ELEM(*distortion_, float, 1, 0) = k2;
  CV_MAT_ELEM(*distortion_, float, 2, 0) = p1;
  CV_MAT_ELEM(*distortion_, float, 3, 0) = p2;
  CV_MAT_ELEM(*distortion_, float, 4, 0) = k3;

  CV_MAT_ELEM(*rectification_, float, 0, 0) = 1.0;
  CV_MAT_ELEM(*rectification_, float, 1, 1) = 1.0;
  CV_MAT_ELEM(*rectification_, float, 2, 2) = 1.0;

  CV_MAT_ELEM(*rectified_intrinsic_, float, 0, 0) = fx;
  CV_MAT_ELEM(*rectified_intrinsic_, float, 0, 2) = cx;
  CV_MAT_ELEM(*rectified_intrinsic_, float, 1, 1) = fy;
  CV_MAT_ELEM(*rectified_intrinsic_, float, 1, 2) = cy;
  CV_MAT_ELEM(*rectified_intrinsic_, float, 2, 2) = 1;
      
}


dc1394_cam::FrameSet
dc1394_cam::Cam::getFrames(dc1394capture_policy_t policy)
{
  CHECK_READY();

  dc1394video_frame_t* frame = getDc1394Frame(policy);

  FrameSet fs;

  if (frame != NULL)
  {
    FrameWrapper fw("image", frame, this, CAM_OWNS_BOTH);

    if (colorize_)
    {
      FrameWrapper fw_color = dc1394_cam::debayerFrame(fw, bayer_);

      fw.releaseFrame();
      fw = fw_color;
    }

    if (rectify_)
    {
      if (init_rectify_)
      {
        initUndistortFrame(fw, intrinsic_, distortion_, rectification_, rectified_intrinsic_, &mapx_, &mapy_);
        init_rectify_ = false;
      }

      FrameWrapper fw_rect = dc1394_cam::undistortFrame(fw, mapx_, mapy_);

      fw.releaseFrame();
      fw = fw_rect;
    }

    fs.push_back(fw);
  }

  return fs;
}


dc1394video_frame_t*
dc1394_cam::Cam::getDc1394Frame(dc1394capture_policy_t policy)
{
    CHECK_READY();

    if (!started)
      start();

    dc1394video_frame_t* f = NULL;
    CHECK_ERR_CLEAN( dc1394_capture_dequeue(dcCam, policy, &f), "Could not capture frame");

    if (f != NULL &&
        (f->frames_behind > 1  || 
         dc1394_capture_is_frame_corrupt(dcCam, f) == DC1394_TRUE)
        )
    {
      releaseDc1394Frame(f);
      f = NULL;
    }

    return f;
}

void
dc1394_cam::Cam::releaseDc1394Frame(dc1394video_frame_t* f)
{
    CHECK_READY();
    CHECK_ERR_CLEAN( dc1394_capture_enqueue(dcCam, f), "Could not release frame");
}

bool
dc1394_cam::Cam::hasFeature(dc1394feature_t feature)
{
  CHECK_READY();
  dc1394bool_t present;
  CHECK_ERR_CLEAN( dc1394_feature_is_present(dcCam, feature, &present), "Could not check if feature was present");
  return (present == DC1394_TRUE);
}

void
dc1394_cam::Cam::setFeature(dc1394feature_t feature, uint32_t value, uint32_t value2)
{
  CHECK_READY();
  dc1394bool_t present;
  CHECK_ERR_CLEAN( dc1394_feature_is_present(dcCam, feature, &present), "Could not check if feature was present");
  if (present == DC1394_TRUE)
  {
    if (feature == DC1394_FEATURE_WHITE_BALANCE)
    {
      CHECK_ERR_CLEAN( dc1394_feature_whitebalance_set_value(dcCam, value, value2), "Could not set feature");
    }
    else
    {
      CHECK_ERR_CLEAN( dc1394_feature_set_value(dcCam, feature, value), "Could not set feature");
    }
  }
}

void
dc1394_cam::Cam::getFeatureBoundaries(dc1394feature_t feature, uint32_t& min, uint32_t& max)
{
  CHECK_READY();
  dc1394bool_t present;
  CHECK_ERR_CLEAN( dc1394_feature_is_present(dcCam, feature, &present), "Could not check if feature was present");
  if (present == DC1394_TRUE)
  {
    CHECK_ERR_CLEAN( dc1394_feature_get_boundaries(dcCam, feature, &min, &max), "Could not find feature boundaries");
  }
}

void
dc1394_cam::Cam::setFeatureAbsolute(dc1394feature_t feature, float value)
{
  CHECK_READY();
  dc1394bool_t present;
  CHECK_ERR_CLEAN( dc1394_feature_is_present(dcCam, feature, &present), "Could not check if feature was present");
  if (present == DC1394_TRUE)
  {
    CHECK_ERR_CLEAN( dc1394_feature_set_absolute_control(dcCam, feature,  DC1394_ON), "Could not enable absolute control.");
    CHECK_ERR_CLEAN( dc1394_feature_set_absolute_value(dcCam, feature, value), "Could not set feature");
  }
}

void
dc1394_cam::Cam::setFeatureMode(dc1394feature_t feature, dc1394feature_mode_t mode)
{
  CHECK_READY();
  dc1394bool_t present;
  CHECK_ERR_CLEAN( dc1394_feature_is_present(dcCam, feature, &present), "Could not check if feature was present");
  if (present == DC1394_TRUE)
  {
    CHECK_ERR_CLEAN( dc1394_feature_set_mode(dcCam, feature, mode), "Could not set feature");
  }
}


void
dc1394_cam::Cam::setControlRegister(uint64_t offset, uint32_t value)
{
  CHECK_READY();
  CHECK_ERR_CLEAN( dc1394_set_control_register(dcCam, offset, value), "Could not set control register");
}

uint32_t
dc1394_cam::Cam::getControlRegister(uint64_t offset)
{
  CHECK_READY();
  uint32_t value;
  CHECK_ERR_CLEAN( dc1394_get_control_register(dcCam, offset, &value), "Could not get control register");
  return value;
}


void
dc1394_cam::Cam::cleanup()
{
  if (FD_ISSET(dc1394_capture_get_fileno(dcCam), &camFds))
    FD_CLR(dc1394_capture_get_fileno(dcCam), &camFds);

  dc1394_video_set_transmission(dcCam, DC1394_OFF);
  dc1394_capture_stop(dcCam);
  dc1394_camera_free(dcCam);
  
  dcCam = NULL;
}

