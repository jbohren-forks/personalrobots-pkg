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

#ifndef CAM_BRIDGE_HH
#define CAM_BRIDGE_HH

#include "image_msgs/RawStereo.h"
#include "image_msgs/FillImage.h"
#include "image.h"

namespace cam_bridge
{
  void CamDataToRawStereo(cam::ImageData* im, image_msgs::Image& im_msg, image_msgs::CamInfo& info_msg, int8_t& type)
  {
    if (im->imRawType != COLOR_CODING_NONE)
    {
      fillImage(im_msg,  "image_raw",
                im->imHeight, im->imWidth, 1,
                "mono", "byte",
                im->imRaw );
      type = image_msgs::RawStereo::IMAGE_RAW;
    }
    else if (im->imType != COLOR_CODING_NONE)
    {
      fillImage(im_msg,  "image",
                im->imHeight, im->imWidth, 1,
                "mono", "byte",
                im->im );
      type = image_msgs::RawStereo::IMAGE;
    }
    else if (im->imColorType != COLOR_CODING_NONE && im->imColorType == COLOR_CODING_RGBA8)
    {
      fillImage(im_msg,  "image_color",
                im->imHeight, im->imWidth, 4,
                "rgba", "byte",
                im->imColor );
      type = image_msgs::RawStereo::IMAGE_COLOR;
    }
    else if (im->imColorType != COLOR_CODING_NONE && im->imColorType == COLOR_CODING_RGB8)
    {
      fillImage(im_msg,  "image_color",
                im->imHeight, im->imWidth, 3,
                "rgb", "byte",
                im->imColor );
      type = image_msgs::RawStereo::IMAGE_COLOR;
    }
    else if (im->imRectType != COLOR_CODING_NONE)
    {
      fillImage(im_msg,  "image_rect",
                im->imHeight, im->imWidth, 1,
                "mono", "byte",
                im->imRect );
      type = image_msgs::RawStereo::IMAGE_RECT;
    }
    else if (im->imRectColorType != COLOR_CODING_NONE && im->imRectColorType == COLOR_CODING_RGBA8)
    {
      fillImage(im_msg,  "image_rect_color",
                im->imHeight, im->imWidth, 4,
                "rgba", "byte",
                im->imRectColor );
      type = image_msgs::RawStereo::IMAGE_RECT_COLOR;
    }
    else if (im->imRectColorType != COLOR_CODING_NONE && im->imRectColorType == COLOR_CODING_RGB8)
    {
      fillImage(im_msg,  "image_rect_color",
                im->imHeight, im->imWidth, 3,
                "rgb", "byte",
                im->imRectColor );
      type = image_msgs::RawStereo::IMAGE_RECT_COLOR;
    }

    info_msg.height = im->imHeight;
    info_msg.width  = im->imWidth;

    memcpy((char*)(&info_msg.D[0]), (char*)(im->D),  5*sizeof(double));
    memcpy((char*)(&info_msg.K[0]), (char*)(im->K),  9*sizeof(double));
    memcpy((char*)(&info_msg.R[0]), (char*)(im->R),  9*sizeof(double));
    memcpy((char*)(&info_msg.P[0]), (char*)(im->P), 12*sizeof(double));
  }

  void StereoDataToRawStereo(cam::StereoData* stIm, image_msgs::RawStereo& raw_stereo)
  {
    raw_stereo.header.stamp = ros::Time().fromNSec(stIm->imLeft->im_time * 1000);

    if (stIm->hasDisparity)
    {
      fillImage(raw_stereo.disparity_image,  "disparity",
                stIm->imHeight, stIm->imWidth, 1,
                "mono", "int16",
                stIm->imDisp );
      
      raw_stereo.stereo_info.has_disparity = true;
    } else {
      clearImage(raw_stereo.disparity_image);
      raw_stereo.stereo_info.has_disparity = false;
    }

    raw_stereo.stereo_info.height = stIm->imHeight;
    raw_stereo.stereo_info.width = stIm->imWidth;
    raw_stereo.stereo_info.dpp = stIm->dpp;
    raw_stereo.stereo_info.num_disp = stIm->numDisp;
    raw_stereo.stereo_info.im_Dtop = stIm->imDtop;
    raw_stereo.stereo_info.im_Dleft = stIm->imDleft;
    raw_stereo.stereo_info.im_Dwidth = stIm->imDwidth;
    raw_stereo.stereo_info.im_Dheight = stIm->imDheight;
    raw_stereo.stereo_info.corr_size = stIm->corrSize;
    raw_stereo.stereo_info.filter_size = stIm->filterSize;
    raw_stereo.stereo_info.hor_offset = stIm->horOffset;
    raw_stereo.stereo_info.texture_thresh = stIm->textureThresh;
    raw_stereo.stereo_info.unique_thresh = stIm->uniqueThresh;
    raw_stereo.stereo_info.smooth_thresh = stIm->smoothThresh;
    raw_stereo.stereo_info.speckle_diff = stIm->speckleDiff;
    raw_stereo.stereo_info.speckle_region_size = stIm->speckleRegionSize;
    raw_stereo.stereo_info.unique_check = stIm->unique_check;
    memcpy((char*)(&raw_stereo.stereo_info.T[0]),  (char*)(stIm->T),   3*sizeof(double));
    memcpy((char*)(&raw_stereo.stereo_info.Om[0]), (char*)(stIm->Om),  3*sizeof(double));
    memcpy((char*)(&raw_stereo.stereo_info.RP[0]), (char*)(stIm->RP), 16*sizeof(double));

    CamDataToRawStereo(stIm->imLeft,  raw_stereo.left_image,  raw_stereo.left_info,  raw_stereo.left_type);
    CamDataToRawStereo(stIm->imRight, raw_stereo.right_image, raw_stereo.right_info, raw_stereo.right_type);
  }


  void extractImage(std_msgs::ByteMultiArray& arr, size_t* sz, uint8_t **d)
  {
    size_t new_size = arr.layout.dim[0].stride;

    if (*sz < new_size);
    {
      MEMFREE(*d);
      *d = (uint8_t *)MEMALIGN(new_size);
      *sz = new_size;
    }
    memcpy((char*)(*d), (char*)(&arr.data[0]), new_size);
  }

  void extractImage(std_msgs::Int16MultiArray& arr, size_t* sz, int16_t **d)
  {
    size_t new_size = arr.layout.dim[0].stride*2;

    if (*sz < new_size);
    {
      MEMFREE(*d);
      *d = (int16_t *)MEMALIGN(new_size);
      *sz = new_size;
    }
    memcpy((char*)(*d), (char*)(&arr.data[0]), new_size);
  }

  void RawStereoToCamData(image_msgs::Image& im_msg, image_msgs::CamInfo& info_msg, int8_t& type, cam::ImageData* im)
  {

    im->imRawType = COLOR_CODING_NONE;
    im->imType = COLOR_CODING_NONE;
    im->imColorType = COLOR_CODING_NONE;
    im->imRectType = COLOR_CODING_NONE;
    im->imRectColorType = COLOR_CODING_NONE;

    if (type == image_msgs::RawStereo::IMAGE_RAW)
    {
      extractImage(im_msg.byte_data, &im->imRawSize, &im->imRaw);
      im->imRawType = COLOR_CODING_BAYER8_RGGB;
    }
    else if (type == image_msgs::RawStereo::IMAGE)
    {
      extractImage(im_msg.byte_data, &im->imSize, &im->im);
      im->imType = COLOR_CODING_MONO8;
    }
    else if (type == image_msgs::RawStereo::IMAGE_COLOR && im_msg.encoding == "rgba")
    {
      extractImage(im_msg.byte_data, &im->imColorSize, &im->imColor);
      im->imColorType = COLOR_CODING_RGBA8;
    }
    else if (type == image_msgs::RawStereo::IMAGE_COLOR && im_msg.encoding == "rgb")
    {
      extractImage(im_msg.byte_data, &im->imColorSize, &im->imColor);
      im->imColorType = COLOR_CODING_RGB8;
    }
    else if (type == image_msgs::RawStereo::IMAGE_RECT)
    {
      extractImage(im_msg.byte_data, &im->imRectSize, &im->imRect);
      im->imRectType = COLOR_CODING_MONO8;
    }
    else if (type == image_msgs::RawStereo::IMAGE_RECT_COLOR && im_msg.encoding == "rgba")
    {
      extractImage(im_msg.byte_data, &im->imRectColorSize, &im->imRectColor);
      im->imRectColorType = COLOR_CODING_RGBA8;
    }
    else if (type == image_msgs::RawStereo::IMAGE_RECT_COLOR && im_msg.encoding == "rgb")
    {
      extractImage(im_msg.byte_data, &im->imRectColorSize, &im->imRectColor);
      im->imRectColorType = COLOR_CODING_RGB8;
    }

    im->imHeight = info_msg.height;
    im->imWidth  = info_msg.width;

    memcpy((char*)(im->D), (char*)(&info_msg.D[0]),  5*sizeof(double));
    memcpy((char*)(im->K), (char*)(&info_msg.K[0]),  9*sizeof(double));
    memcpy((char*)(im->R), (char*)(&info_msg.R[0]),  9*sizeof(double));
    memcpy((char*)(im->P), (char*)(&info_msg.P[0]),  12*sizeof(double));
    im->hasRectification = true;
  }

  void RawStereoToStereoData(image_msgs::RawStereo& raw_stereo, cam::StereoData* stIm)
  {
    stIm->imLeft->im_time = raw_stereo.header.stamp.toNSec() / 1000;
    stIm->imRight->im_time = raw_stereo.header.stamp.toNSec() / 1000;
    stIm->setSize(raw_stereo.stereo_info.width, raw_stereo.stereo_info.height);
   
    stIm->hasDisparity = false;
 
    if (raw_stereo.stereo_info.has_disparity)
    {
      extractImage(raw_stereo.disparity_image.int16_data, &stIm->imDispSize, &stIm->imDisp);
      stIm->hasDisparity = true;
    }


    stIm->dpp                 =     raw_stereo.stereo_info.dpp;
    stIm->numDisp             =     raw_stereo.stereo_info.num_disp;
    stIm->imDtop              =     raw_stereo.stereo_info.im_Dtop;
    stIm->imDleft             =     raw_stereo.stereo_info.im_Dleft;
    stIm->imDwidth            =     raw_stereo.stereo_info.im_Dwidth;
    stIm->imDheight           =     raw_stereo.stereo_info.im_Dheight;
    stIm->corrSize            =     raw_stereo.stereo_info.corr_size;
    stIm->filterSize          =     raw_stereo.stereo_info.filter_size;
    stIm->horOffset           =     raw_stereo.stereo_info.hor_offset;
    stIm->textureThresh       =     raw_stereo.stereo_info.texture_thresh;
    stIm->uniqueThresh        =     raw_stereo.stereo_info.unique_thresh;
    stIm->smoothThresh        =     raw_stereo.stereo_info.smooth_thresh;
    stIm->speckleDiff         =     raw_stereo.stereo_info.speckle_diff;
    stIm->speckleRegionSize =     raw_stereo.stereo_info.speckle_region_size;
    stIm->unique_check        =     raw_stereo.stereo_info.unique_check;
    memcpy((char*)(stIm->T),  (char*)(&raw_stereo.stereo_info.T[0]),  3*sizeof(double));
    memcpy((char*)(stIm->Om), (char*)(&raw_stereo.stereo_info.Om[0]), 3*sizeof(double));
    memcpy((char*)(stIm->RP), (char*)(&raw_stereo.stereo_info.RP[0]), 16*sizeof(double));

    RawStereoToCamData(raw_stereo.left_image,  raw_stereo.left_info,  raw_stereo.left_type, stIm->imLeft);
    RawStereoToCamData(raw_stereo.right_image, raw_stereo.right_info, raw_stereo.right_type, stIm->imRight);
  }
}

#endif
