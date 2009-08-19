/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 *
 * Author: Alexander Sorokin
 *********************************************************************/

//STD
#include <stdio.h>
#include <stdlib.h>
#include <string>


//ROS
#include "ros/ros.h"

//ROS-PKG


#include "image_server/SaveImage.h"
#include "image_server/LoadImage.h"

#include <opencv_latest/CvBridge.h>
#include <boost/format.hpp>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <boost/format.hpp>




class FilesystemImageStorageService{
protected:
  std::string db_path_;
  ros::NodeHandle n_;
  int last_id;

  ros::ServiceServer save_svc_;
  ros::ServiceServer load_svc_;
  sensor_msgs::CvBridge img_bridge_;

  boost::format filename_format_;
public:
  FilesystemImageStorageService(){
    if(!n_.getParam(std::string("~db_path"),db_path_,"NONE"))
    {
      ROS_ERROR("Database path (db_path) argument is required");
      throw "Database path (db_path) argument is required";
    }
    if(db_path_=="NONE")
    {
      ROS_ERROR("Database path (db_path) argument is required");
      throw "Database path (db_path) argument is required";
    }
    last_id=0;
    filename_format_.parse(db_path_+"/image%05d.jpg");


    save_svc_ = n_.advertiseService(std::string("save_image"), &FilesystemImageStorageService::save,this);
    load_svc_ = n_.advertiseService(std::string("load_image"), &FilesystemImageStorageService::load,this);
  }


  
  bool save(image_server::SaveImage::Request  &req,
            image_server::SaveImage::Response &res )
  {
    ROS_INFO("Save image call");
    last_id++;
    if (img_bridge_.fromImage(req.image, "bgr8"))
    {
    }
    else
    {
      ROS_ERROR("Unable to convert %s image to bgr8", req.image.encoding.c_str());
      return false;
    }

    IplImage *image = img_bridge_.toIpl();
    if (image) {
      std::string filename = (filename_format_ % last_id).str();
      cvSaveImage(filename.c_str(), image);


    res.id=float(last_id);
    ROS_INFO("Saved image: [%ld]", (long int)last_id);
    }else{
      ROS_INFO("CvBridge failed");
      return false;
    }
    return true;
  }


  bool load(image_server::LoadImage::Request  &req,
            image_server::LoadImage::Response &res )
  {
    /*
    uint32_t id = (uint32_t)req.id;
    //ros::Message msg;
    std::string s("/image");
    uint32_t node_id(0);
    if(db_->get(*(ros::Message*)&res.image,id,s,node_id))
      return true;
    
    ROS_ERROR("db_->get failed");
    */
    return false;
  }


};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_storage_server");
  ros::NodeHandle n;

  FilesystemImageStorageService iss;


  ros::spin();

  return 0;
}


