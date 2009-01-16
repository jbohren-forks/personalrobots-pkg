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

#include <cstdio>

#include "image.h"

#include "ros/node.h"
#include "image_msgs/RawStereo.h"
#include "cam_bridge.h"

#include "image_msgs/Image.h"
#include "image_msgs/FillImage.h"
#include "image_msgs/CamInfo.h"
#include "image_msgs/StereoInfo.h"
#include "std_msgs/PointCloud.h"


#include "diagnostic_updater/diagnostic_updater.h"

using namespace std;

class StereoProc : public ros::Node
{

  bool do_colorize_;
  bool do_rectify_;
  bool do_stereo_;
  bool do_calc_points_;

  image_msgs::RawStereo    raw_stereo_;

  image_msgs::Image         img_;
  std_msgs::PointCloud      cloud_;
  image_msgs::CamInfo       cam_info_;
  image_msgs::DisparityInfo disparity_info_;
  image_msgs::StereoInfo    stereo_info_;


  DiagnosticUpdater<StereoProc> diagnostic_;
  int count_;
  double desired_freq_;

  string frame_id_;

  std::string stereo_name_;

public:

  cam::StereoData* stdata_;

  StereoProc() : ros::Node("stereoproc"), diagnostic_(this), count_(0), stdata_(NULL)
  {

    stereo_name_ = mapName("stereo") + std::string("/");

    diagnostic_.addUpdater( &StereoProc::freqStatus );

    param(stereo_name_ + std::string("do_colorize"), do_colorize_, false);

    param(stereo_name_ + std::string("do_rectify"), do_rectify_, false);
    
    param(stereo_name_ + std::string("do_stereo"), do_stereo_, false);
    
    param(stereo_name_ + std::string("do_calc_points"), do_calc_points_, false);
    
    // Must do stereo if calculating points
    do_stereo_ = do_stereo_ || do_calc_points_;
    
    // Must rectify if doing stereo
    do_rectify_ = do_rectify_ || do_stereo_;
    
    stdata_ = new cam::StereoData;

    stdata_->setUniqueThresh(36);
    stdata_->setTextureThresh(30);
    stdata_->setSpeckleRegionSize(100);
    stdata_->setSpeckleDiff(10);

    subscribe(stereo_name_ + std::string("raw_stereo"), raw_stereo_, &StereoProc::rawCb, 1);
  }


  ~StereoProc()
  {
    if (stdata_)
      delete stdata_;
  }


  void rawCb()
  {

    cam_bridge::RawStereoToStereoData(raw_stereo_, stdata_);

    if (do_colorize_)
    {
      stdata_->doBayerColorRGB();
    }

    if (do_rectify_)
    {
      stdata_->doRectify();
    }

    if (do_stereo_)
    {
      stdata_->doDisparity();
    }

    if (do_calc_points_)
    {
      stdata_->doCalcPts();
    }

    advertiseCam();
    publishCam();
    
    count_++;
  }


  void publishCam()
  {
    publishImages(stereo_name_ + std::string("left/"), stdata_->imLeft);
    publishImages(stereo_name_ + std::string("right/"), stdata_->imRight);
      
    if (stdata_->hasDisparity)
    {
      disparity_info_.header.stamp = raw_stereo_.header.stamp;
      disparity_info_.header.frame_id = raw_stereo_.header.frame_id;

      disparity_info_.height = stdata_->imHeight;
      disparity_info_.width = stdata_->imWidth;

      disparity_info_.dpp = stdata_->dpp;
      disparity_info_.num_disp = stdata_->numDisp;
      disparity_info_.im_Dtop = stdata_->imDtop;
      disparity_info_.im_Dleft = stdata_->imDleft;
      disparity_info_.im_Dwidth = stdata_->imDwidth;
      disparity_info_.im_Dheight = stdata_->imDheight;
      disparity_info_.corr_size = stdata_->corrSize;
      disparity_info_.filter_size = stdata_->filterSize;
      disparity_info_.hor_offset = stdata_->horOffset;
      disparity_info_.texture_thresh = stdata_->textureThresh;
      disparity_info_.unique_thresh = stdata_->uniqueThresh;
      disparity_info_.smooth_thresh = stdata_->smoothThresh;
      disparity_info_.speckle_diff = stdata_->speckleDiff;
      disparity_info_.speckle_region_size = stdata_->speckleRegionSize;
      disparity_info_.unique_check = stdata_->unique_check;

      publish(stereo_name_ + std::string("disparity_info"), disparity_info_);

      fillImage(img_,  "disparity",
                stdata_->imHeight, stdata_->imWidth, 1,
                "mono", "uint16",
                stdata_->imDisp );

      img_.header.stamp = raw_stereo_.header.stamp;
      img_.header.frame_id = raw_stereo_.header.frame_id;
      publish(stereo_name_ + std::string("disparity"), img_);
    }

    if (do_calc_points_)
    {
      cloud_.header.stamp = raw_stereo_.header.stamp;
      cloud_.header.frame_id = raw_stereo_.header.frame_id;
      cloud_.pts.resize(stdata_->numPts);
      cloud_.chan.resize(1);
      cloud_.chan[0].name = "rgb";
      cloud_.chan[0].vals.resize(stdata_->numPts);
        
      for (int i = 0; i < stdata_->numPts; i++)
      {
        cloud_.pts[i].y = -stdata_->imPts[3*i + 0];
        cloud_.pts[i].z = -stdata_->imPts[3*i + 1];
        cloud_.pts[i].x = stdata_->imPts[3*i + 2];
      }

      for (int i = 0; i < stdata_->numPts; i++)
      {
        int rgb = (stdata_->imPtsColor[3*i] << 16) | (stdata_->imPtsColor[3*i + 1] << 8) | stdata_->imPtsColor[3*i + 2];
        cloud_.chan[0].vals[i] = *(float*)(&rgb);
      }

      publish(stereo_name_ + std::string("cloud"), cloud_);
    }

    stereo_info_.header.stamp = raw_stereo_.header.stamp;

    stereo_info_.height = stdata_->imHeight;
    stereo_info_.width = stdata_->imWidth;

    memcpy((char*)(&stereo_info_.T[0]),  (char*)(stdata_->T),   3*sizeof(double));
    memcpy((char*)(&stereo_info_.Om[0]), (char*)(stdata_->Om),  3*sizeof(double));
    memcpy((char*)(&stereo_info_.RP[0]), (char*)(stdata_->RP), 16*sizeof(double));

    publish(stereo_name_ + std::string("stereo_info"), stereo_info_);
  }

  void publishImages(std::string base_name, cam::ImageData* img_data)
  {
    if (img_data->imRawType != COLOR_CODING_NONE)
    {
      fillImage(img_,  "image_raw",
                img_data->imHeight, img_data->imWidth, 1,
                "mono", "uint8",
                img_data->imRaw );

      img_.header.stamp = raw_stereo_.header.stamp;
      img_.header.frame_id = raw_stereo_.header.frame_id;

      publish(base_name + std::string("image_raw"), img_);
    }

    if (img_data->imType != COLOR_CODING_NONE)
    {
      fillImage(img_,  "image",
                img_data->imHeight, img_data->imWidth, 1,
                "mono", "uint8",
                img_data->im );
      img_.header.stamp = raw_stereo_.header.stamp;
      img_.header.frame_id = raw_stereo_.header.frame_id;
      publish(base_name + std::string("image"), img_);
    }

    if (img_data->imColorType != COLOR_CODING_NONE && img_data->imColorType == COLOR_CODING_RGB8)
    {
      fillImage(img_,  "image_color",
                img_data->imHeight, img_data->imWidth, 3,
                "rgb", "uint8",
                img_data->imColor );

      img_.header.stamp = raw_stereo_.header.stamp;
      img_.header.frame_id = raw_stereo_.header.frame_id;
      publish(base_name + std::string("image_color"), img_);
    }

    if (img_data->imRectType != COLOR_CODING_NONE)
    {
      fillImage(img_,  "image_rect",
                img_data->imHeight, img_data->imWidth, 1,
                "mono", "uint8",
                img_data->imRect );
      img_.header.stamp = raw_stereo_.header.stamp;
      img_.header.frame_id = raw_stereo_.header.frame_id;
      publish(base_name + std::string("image_rect"), img_);
    }

    if (img_data->imRectColorType != COLOR_CODING_NONE && img_data->imRectColorType == COLOR_CODING_RGB8)
    {
      fillImage(img_,  "image_rect_color",
                img_data->imHeight, img_data->imWidth, 3,
                "rgb", "uint8",
                img_data->imRectColor );
      img_.header.stamp = raw_stereo_.header.stamp;
      img_.header.frame_id = raw_stereo_.header.frame_id;
      publish(base_name + std::string("image_rect_color"), img_);
    }

    if (img_data->imRectColorType != COLOR_CODING_NONE && img_data->imRectColorType == COLOR_CODING_RGBA8)
    {
      fillImage(img_,  "image_rect_color",
                img_data->imHeight, img_data->imWidth, 4,
                "rgba", "uint8",
                img_data->imRectColor );
      img_.header.stamp = raw_stereo_.header.stamp;
      img_.header.frame_id = raw_stereo_.header.frame_id;
      publish(base_name + std::string("image_rect_color"), img_);
    }


    cam_info_.header.stamp = raw_stereo_.header.stamp;
    cam_info_.header.frame_id = raw_stereo_.header.frame_id;
    cam_info_.height = img_data->imHeight;
    cam_info_.width  = img_data->imWidth;

    memcpy((char*)(&cam_info_.D[0]), (char*)(img_data->D),  5*sizeof(double));
    memcpy((char*)(&cam_info_.K[0]), (char*)(img_data->K),  9*sizeof(double));
    memcpy((char*)(&cam_info_.R[0]), (char*)(img_data->R),  9*sizeof(double));
    memcpy((char*)(&cam_info_.P[0]), (char*)(img_data->P), 12*sizeof(double));

    publish(base_name + std::string("cam_info"), cam_info_);

  }


  void advertiseImages(std::string base_name, cam::ImageData* img_data)
  {
    advertise<image_msgs::CamInfo>(base_name + std::string("cam_info"), 1);

    if (img_data->imRawType != COLOR_CODING_NONE)
      advertise<image_msgs::Image>(base_name + std::string("image_raw"), 1);

    if (img_data->imType != COLOR_CODING_NONE)
      advertise<image_msgs::Image>(base_name + std::string("image"), 1);

    if (img_data->imColorType != COLOR_CODING_NONE)
      advertise<image_msgs::Image>(base_name + std::string("image_color"), 1);

    if (img_data->imRectType != COLOR_CODING_NONE)
      advertise<image_msgs::Image>(base_name + std::string("image_rect"), 1);

    if (img_data->imRectColorType != COLOR_CODING_NONE)
      advertise<image_msgs::Image>(base_name + std::string("image_rect_color"), 1);

  }

  void advertiseCam()
  {
    advertise<image_msgs::StereoInfo>(stereo_name_ + std::string("stereo_info"), 1);

    advertiseImages(stereo_name_ + std::string("left/"), stdata_->imLeft);
    advertiseImages(stereo_name_ + std::string("right/"), stdata_->imRight);
    
    if (stdata_->hasDisparity)
    {
      advertise<image_msgs::DisparityInfo>(stereo_name_ + std::string("disparity_info"), 1);
      advertise<image_msgs::Image>(stereo_name_ + std::string("disparity"), 1);
    }
    
    if (do_calc_points_)
      advertise<std_msgs::PointCloud>(stereo_name_ + std::string("cloud"),1);
  }


  void freqStatus(robot_msgs::DiagnosticStatus& status)
  {
    status.name = "Frequency Status";

    double freq = (double)(count_)/diagnostic_.getPeriod();

    if (freq < (.9*desired_freq_))
    {
      status.level = 2;
      status.message = "Desired frequency not met";
    }
    else
    {
      status.level = 0;
      status.message = "Desired frequency met";
    }

    status.set_values_size(3);
    status.values[0].label = "Images in interval";
    status.values[0].value = count_;
    status.values[1].label = "Desired frequency";
    status.values[1].value = desired_freq_;
    status.values[2].label = "Actual frequency";
    status.values[2].value = freq;

    printf("%g fps\n", freq);

    count_ = 0;
  }

  bool spin()
  {
    // Start up the camera
    while (ok())
    {
      usleep(100000);
      diagnostic_.update();
    }

    return true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);

  StereoProc sp;

  sp.spin();

  ros::fini();
  return 0;
}

