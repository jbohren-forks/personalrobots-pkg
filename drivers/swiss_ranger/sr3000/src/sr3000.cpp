/*********************************************************************
*
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include <stdlib.h>
#include <libusbSR.h>
#include <ros/ros.h>
#include <robot_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/FillImage.h>


#define MODE (AM_MEDIAN | AM_COR_FIX_PTRN | AM_SW_ANF | AM_SR3K_2TAP_PROC | AM_MEDIANCROSS)
#define CAM_ROWS 144
#define CAM_COLS 176

namespace sr3000 {
  class SR3000{
    public:
      SR3000(){}
      int setup();
      int shutdown();
      void run();

    private:
      CMesaDevice* sr_cam_;
      ImgEntry* img_entry_array_;
      float *buffer_, *xp_, *yp_, *zp_;

      unsigned int rows_, cols_, inr_;
      robot_msgs::PointCloud cam_cloud_;
      sensor_msgs::Image img_;

      int auto_illumination_, integration_time_, modulation_freq_, amp_threshold_;
      bool publish_point_cloud_, publish_dist_img_, publish_int_img_;
      std::string sr_frame_;
  };

  int SR3000::setup(){
    ros::NodeHandle n;
    n.param("~publish_point_cloud", publish_point_cloud_, true);
    n.param("~publish_dist_img", publish_dist_img_, true);
    n.param("~publish_int_img", publish_int_img_, true);

    n.param("~frame_id", sr_frame_, std::string("sr_mount"));

    //returns the device id of the camera
    SR_OpenUSB(&sr_cam_, 0);

    ROS_INFO("Attempting to connect to the SR3000 camera");
    
    rows_ = SR_GetRows(sr_cam_);
    cols_ = SR_GetCols(sr_cam_);
    inr_ = SR_GetImageList(sr_cam_, &img_entry_array_);
    modulation_freq_  = SR_GetModulationFrequency (sr_cam_);
    integration_time_ = SR_GetIntegrationTime (sr_cam_);

    if((cols_ != CAM_COLS) || (rows_ != CAM_ROWS) || (inr_ < 1) || (img_entry_array_ == 0)){
      ROS_ERROR("Error while connecting to camera! Expected rows: %d, cols: %d, image list: >=%d, but got rows: %d, cols: %d, image list: %d", 
          CAM_ROWS, CAM_COLS, 1, rows_, cols_, inr_);
      SR_Close(sr_cam_);
      return -1;
    }

    ROS_INFO("Successfully connected to the SR3000 camera");


    SR_SetMode(sr_cam_, MODE);

    // Points array
    size_t buffer_size = rows_ * cols_ * 3 * sizeof (float);
    buffer_ = (float*)malloc (buffer_size);
    memset (buffer_, 0xaf, buffer_size);

    xp_ = buffer_;
    yp_ = &xp_[rows_*cols_];
    zp_ = &yp_[rows_*cols_];

    return 0;

  }

  int SR3000::shutdown(){
    //close the camera
    SR_Close(sr_cam_);
    ROS_INFO("Closing connection to the SR3000 camera");
    
    //free the allocated memory buffer
    if(buffer_)
      delete buffer_;

    return 0;
  }

  void SR3000::run(){
    ros::NodeHandle n;
    ros::Publisher cloud_pub, dist_pub, int_pub;

    if(publish_point_cloud_)
      cloud_pub = n.advertise<robot_msgs::PointCloud>("sr3000_cloud", 1);

    if(publish_dist_img_)
      dist_pub = n.advertise<sensor_msgs::Image>("sr3000_distance", 1);
    
    if(publish_int_img_)
      int_pub = n.advertise<sensor_msgs::Image>("sr3000_intensity", 1);

    ros::Rate r(10);
    while(n.ok()){
      SR_Acquire(sr_cam_);
      ros::Time stamp = ros::Time::now();

      ROS_ASSERT(inr_ >= 2);

      uint16_t *distance_image  = (unsigned uint16_t*)SR_GetImage(sr_cam_, 0);
      uint16_t *intensity_image = (unsigned uint16_t*)SR_GetImage(sr_cam_, 1);

      // Points array
      SR_CoordTrfFlt (sr_cam_, xp_, yp_, zp_, sizeof (float), sizeof (float), sizeof (float));

      if(publish_dist_img_){
        fillImage(img_, "distance_image", rows_, cols_, 1,
            "mono", "uint16", distance_image);
        dist_pub.publish(img_);
      }

      if(publish_int_img_){
        fillImage(img_, "intensity_image", rows_, cols_, 1,
            "mono", "uint16", intensity_image);
        int_pub.publish(img_);
      }

      if(publish_point_cloud_){
        cam_cloud_.set_pts_size(rows_ * cols_);

        //add an intensity channel to the cloud
        cam_cloud_.set_chan_size(1);
        cam_cloud_.chan[0].name = "intensities";
        cam_cloud_.chan[0].set_vals_size(rows_ * cols_);

        for(unsigned int i = 0; i < cam_cloud_.get_pts_size(); ++i){
          cam_cloud_.pts[i].x = xp_[i];
          cam_cloud_.pts[i].y = yp_[i];
          cam_cloud_.pts[i].z = zp_[i];

          cam_cloud_.chan[0].vals[i] = intensity_image[i];
        }

        cam_cloud_.header.stamp = ros::Time::now();
        cam_cloud_.header.frame_id = sr_frame_;

        cloud_pub.publish(cam_cloud_);
      }

      r.sleep();
    }
    shutdown();

  }
};

using namespace sr3000;
int main(int argc, char** argv){
  ros::init(argc, argv, "SR3000_node");
  SR3000 sr;
  sr.setup();
  sr.run();
}
