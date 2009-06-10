/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/*
 @mainpage
   Desc: RosProsilica plugin for simulating cameras in Gazebo
   Author: John Hsu
   Date: 24 Sept 2008
   SVN info: $Id$
 @htmlinclude manifest.html
 @b RosProsilica plugin mimics after prosilica_cam package
 */

#include <algorithm>
#include <assert.h>

#include <gazebo_plugin/ros_prosilica.h>

#include <gazebo/Sensor.hh>
#include <gazebo/Model.hh>
#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>
#include "gazebo/MonoCameraSensor.hh"


#include <image_msgs/Image.h>
#include <image_msgs/CamInfo.h>
#include <image_msgs/FillImage.h>
#include <opencv_latest/CvBridge.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include <cv.h>
#include <cvwimage.h>

#include <boost/scoped_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/tokenizer.hpp>
#include <boost/thread.hpp>
#include <string>

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("ros_prosilica", RosProsilica);

////////////////////////////////////////////////////////////////////////////////
// Constructor
RosProsilica::RosProsilica(Entity *parent)
    : Controller(parent)
{
  this->myParent = dynamic_cast<MonoCameraSensor*>(this->parent);

  if (!this->myParent)
    gzthrow("RosProsilica controller requires a Camera Sensor as its parent");

  Param::Begin(&this->parameters);
  this->imageTopicNameP = new ParamT<std::string>("imageTopicName","~image", 0);
  this->imageRectTopicNameP = new ParamT<std::string>("imageRectTopicName","~image_rect", 0);
  this->camInfoTopicNameP = new ParamT<std::string>("camInfoTopicName","~cam_info", 0);
  this->camInfoServiceNameP = new ParamT<std::string>("camInfoServiceName","~cam_info_service", 0);
  this->pollServiceNameP = new ParamT<std::string>("pollServiceName","~poll", 0);
  this->frameNameP = new ParamT<std::string>("frameName","prosilica_optical_frame", 0);
  Param::End();

  rosnode = ros::g_node; // comes from where?
  int argc = 0;
  char** argv = NULL;
  if (rosnode == NULL)
  {
    ros::init(argc,argv);
    rosnode = new ros::Node("ros_gazebo",ros::Node::DONT_HANDLE_SIGINT);
    ROS_DEBUG("Starting node in prosilica plugin");
  }

}

////////////////////////////////////////////////////////////////////////////////
// Destructor
RosProsilica::~RosProsilica()
{
  delete this->imageTopicNameP;
  delete this->imageRectTopicNameP;
  delete this->camInfoTopicNameP;
  delete this->camInfoServiceNameP;
  delete this->pollServiceNameP;
  delete this->frameNameP;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void RosProsilica::LoadChild(XMLConfigNode *node)
{
  this->imageTopicNameP->Load(node);
  this->imageRectTopicNameP->Load(node);
  this->camInfoTopicNameP->Load(node);
  this->camInfoServiceNameP->Load(node);
  this->pollServiceNameP->Load(node);
  this->frameNameP->Load(node);
  
  this->imageTopicName = this->imageTopicNameP->GetValue();
  this->imageRectTopicName = this->imageRectTopicNameP->GetValue();
  this->camInfoTopicName = this->camInfoTopicNameP->GetValue();
  this->camInfoServiceName = this->camInfoServiceNameP->GetValue();
  this->pollServiceName = this->pollServiceNameP->GetValue();
  this->frameName = this->frameNameP->GetValue();

  ROS_DEBUG("prosilica image topic name %s", this->imageTopicName.c_str());
  rosnode->advertise<image_msgs::Image>(this->imageTopicName,1);
  rosnode->advertise<image_msgs::Image>(this->imageRectTopicName,1);
  rosnode->advertise<image_msgs::CamInfo>(this->camInfoTopicName,1);
  rosnode->advertiseService(this->camInfoServiceName,&RosProsilica::camInfoService, this, 0);
  rosnode->advertiseService(this->pollServiceName,&RosProsilica::triggeredGrab, this, 0);
}

////////////////////////////////////////////////////////////////////////////////
// service call to return camera info
bool RosProsilica::camInfoService(prosilica_cam::CamInfo::Request &req,
                                  prosilica_cam::CamInfo::Response &res)
{
  return true;
}
bool RosProsilica::triggeredGrab(prosilica_cam::PolledImage::Request &req,
                                 prosilica_cam::PolledImage::Response &res)
{
  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void RosProsilica::InitChild()
{

  // set parent sensor to active automatically
  this->myParent->SetActive(true);

  // set buffer size
  this->width            = this->myParent->GetImageWidth();
  this->height           = this->myParent->GetImageHeight();
  this->depth            = 1;

}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void RosProsilica::UpdateChild()
{

  // do this first so there's chance for sensor to run 1 frame after activate
  if (this->myParent->IsActive())
    this->PutCameraData();
  else
    this->myParent->SetActive(true); // as long as this plugin is running, parent is active

}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void RosProsilica::FiniChild()
{
  rosnode->unadvertise(this->imageTopicName);
  rosnode->unadvertise(this->imageRectTopicName);
  rosnode->unadvertise(this->camInfoTopicName);
  rosnode->unadvertiseService(this->camInfoServiceName);
  rosnode->unadvertiseService(this->pollServiceName);
  this->myParent->SetActive(false);
}

////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void RosProsilica::PutCameraData()
{
  const unsigned char *src;

  // Get a pointer to image data
  src = this->myParent->GetImageData(0);

  //std::cout << " updating prosilica " << this->imageTopicName << " " << data->image_size << std::endl;
  if (src)
  {
    //double tmpT0 = Simulator::Instance()->GetWallTime();

    this->lock.lock();


    // copy data into image
    this->imageMsg->header.frame_id = this->frameName;
    this->imageMsg->header.stamp.sec = (unsigned long)floor(Simulator::Instance()->GetSimTime());
    this->imageMsg->header.stamp.nsec = (unsigned long)floor(  1e9 * (  Simulator::Instance()->GetSimTime() - this->imageMsg->header.stamp.sec) );

    //double tmpT1 = Simulator::Instance()->GetWallTime();
    //double tmpT2;
    // fill CamInfo left_info
    this->camInfoMsg->header = this->imageMsg->header;
    this->camInfoMsg->height = this->myParent->GetImageHeight();
    this->camInfoMsg->width  = this->myParent->GetImageWidth() ;
    // distortion
    this->camInfoMsg->D[0] = 0.0;
    this->camInfoMsg->D[1] = 0.0;
    this->camInfoMsg->D[2] = 0.0;
    this->camInfoMsg->D[3] = 0.0;
    this->camInfoMsg->D[4] = 0.0;
    // original camera matrix
    this->camInfoMsg->K[0] = 0;//this->focal_length;
    this->camInfoMsg->K[1] = 0.0;
    this->camInfoMsg->K[2] = 0;//this->Cx;
    this->camInfoMsg->K[3] = 0.0;
    this->camInfoMsg->K[4] = 0;//this->focal_length;
    this->camInfoMsg->K[5] = 0;//this->Cy;
    this->camInfoMsg->K[6] = 0.0;
    this->camInfoMsg->K[7] = 0.0;
    this->camInfoMsg->K[8] = 1.0;
    // rectification
    this->camInfoMsg->R[0] = 1.0;
    this->camInfoMsg->R[1] = 0.0;
    this->camInfoMsg->R[2] = 0.0;
    this->camInfoMsg->R[3] = 0.0;
    this->camInfoMsg->R[4] = 1.0;
    this->camInfoMsg->R[5] = 0.0;
    this->camInfoMsg->R[6] = 0.0;
    this->camInfoMsg->R[7] = 0.0;
    this->camInfoMsg->R[8] = 1.0;
    // camera projection matrix (same as camera matrix due to lack of distortion/rectification) (is this generated?)
    this->camInfoMsg->P[0] = 0;//this->focal_length;
    this->camInfoMsg->P[1] = 0.0;
    this->camInfoMsg->P[2] = 0;//this->Cx;
    this->camInfoMsg->P[3] = 0.0;
    this->camInfoMsg->P[4] = 0.0;
    this->camInfoMsg->P[5] = 0;//this->focal_length;
    this->camInfoMsg->P[6] = 0;//this->Cy;
    this->camInfoMsg->P[7] = 0.0;
    this->camInfoMsg->P[8] = 0.0;
    this->camInfoMsg->P[9] = 0.0;
    this->camInfoMsg->P[10] = 1.0;
    this->camInfoMsg->P[11] = 0.0;


    /// @todo: don't bother if there are no subscribers
    if (this->rosnode->numSubscribers(this->imageTopicName) > 0)
    {
      // copy from src to imageMsg
      fillImage(*this->imageMsg   ,"image_raw" ,
                this->height     ,this->width ,this->depth,
                "mono"            ,"uint8"     ,
                (void*)src );

      //tmpT2 = Simulator::Instance()->GetWallTime();

      // publish to ros
      rosnode->publish(this->imageTopicName,*this->imageMsg);
    }

    //double tmpT3 = Simulator::Instance()->GetWallTime();

    this->lock.unlock();
  }

}

