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
 * Desc: Stereo camera controller.
 * Author: Nathan Koenig
 * Date: 06 April 2008
 * SVN info: $Id: ros_stereo_camera.cpp 4436 2008-03-24 17:42:45Z robotos $
 */

#include <algorithm>
#include <assert.h>

#include <gazebo_plugin/ros_stereo_camera.h>

#include <gazebo/Sensor.hh>
#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>
#include <gazebo/Model.hh>

#include "image_msgs/Image.h"
#include "image_msgs/FillImage.h"



using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("ros_stereocamera", RosStereoCamera);

////////////////////////////////////////////////////////////////////////////////
// Constructor
RosStereoCamera::RosStereoCamera(Entity *parent)
    : Controller(parent)
{
  this->myParent = parent;

  if (!dynamic_cast<Model*>(this->myParent))
    gzthrow("RosStereoCamera controller requires a Model as its parent");

  Param::Begin(&this->parameters);
  // camera sensor names
  this->leftCameraNameP = new ParamT<std::string>("leftCamera","", 1);
  this->rightCameraNameP = new ParamT<std::string>("rightCamera","", 1);
  // raw_stereo topic name
  this->topicNameP = new ParamT<std::string>("topicName","stereo/raw_stereo", 0);
  // camera frame names
  this->leftFrameNameP = new ParamT<std::string>("leftFrameName","stereo_link", 0);
  this->rightFrameNameP = new ParamT<std::string>("rightFrameName","stereo_r_link", 0);
  // camera parameters 
  this->CxPrimeP = new ParamT<double>("CxPrime",320, 0); // for 640x480 image
  this->CxP  = new ParamT<double>("Cx" ,320, 0); // for 640x480 image
  this->CyP  = new ParamT<double>("Cy" ,240, 0); // for 640x480 image
  this->focal_lengthP  = new ParamT<double>("focal_length" ,554.256, 0); // == image_width(px) / (2*tan( hfov(radian) /2))
  this->distortion_k1P  = new ParamT<double>("distortion_k1" ,0, 0);
  this->distortion_k2P  = new ParamT<double>("distortion_k2" ,0, 0);
  this->distortion_k3P  = new ParamT<double>("distortion_k3" ,0, 0);
  this->distortion_t1P  = new ParamT<double>("distortion_t1" ,0, 0);
  this->distortion_t2P  = new ParamT<double>("distortion_t2" ,0, 0);
  this->baselineP  = new ParamT<double>("baseline" ,0.05, 0); // distance from left to right camera
  Param::End();

  // get ros node instance if it exists
  rosnode = ros::g_node;
  int argc = 0;
  char** argv = NULL;
  if (rosnode == NULL)
  {
    // this only works for a single camera.
    ros::init(argc,argv);
    rosnode = new ros::Node("ros_gazebo",ros::Node::DONT_HANDLE_SIGINT);
    ROS_DEBUG("Starting node in stereo camera");
  }
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
RosStereoCamera::~RosStereoCamera()
{
  delete this->leftCameraNameP;
  delete this->rightCameraNameP;
  delete this->leftFrameNameP;
  delete this->rightFrameNameP;
  delete this->topicNameP;
  delete this->CxPrimeP;
  delete this->CxP;
  delete this->CyP;
  delete this->focal_lengthP;
  delete this->distortion_k1P;
  delete this->distortion_k2P;
  delete this->distortion_k3P;
  delete this->distortion_t1P;
  delete this->distortion_t2P;
  delete this->baselineP;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void RosStereoCamera::LoadChild(XMLConfigNode *node)
{
  this->leftCameraNameP->Load(node);
  this->rightCameraNameP->Load(node);
  this->topicNameP->Load(node);
  this->leftFrameNameP->Load(node);
  this->rightFrameNameP->Load(node);
  this->CxPrimeP->Load(node);
  this->CxP->Load(node);
  this->CyP->Load(node);
  this->focal_lengthP->Load(node);
  this->distortion_k1P->Load(node);
  this->distortion_k2P->Load(node);
  this->distortion_k3P->Load(node);
  this->distortion_t1P->Load(node);
  this->distortion_t2P->Load(node);
  this->baselineP->Load(node);

  this->leftCameraName = this->leftCameraNameP->GetValue();
  this->rightCameraName = this->rightCameraNameP->GetValue();
  this->topicName = this->topicNameP->GetValue();
  this->leftFrameName = this->leftFrameNameP->GetValue();
  this->rightFrameName = this->rightFrameNameP->GetValue();
  this->CxPrime = this->CxPrimeP->GetValue();
  this->Cx = this->CxP->GetValue();
  this->Cy = this->CyP->GetValue();
  this->focal_length = this->focal_lengthP->GetValue();
  this->distortion_k1 = this->distortion_k1P->GetValue();
  this->distortion_k2 = this->distortion_k2P->GetValue();
  this->distortion_k3 = this->distortion_k3P->GetValue();
  this->distortion_t1 = this->distortion_t1P->GetValue();
  this->distortion_t2 = this->distortion_t2P->GetValue();
  this->baseline = this->baselineP->GetValue();


}

////////////////////////////////////////////////////////////////////////////////
/// Save the controller.
void RosStereoCamera::SaveChild(std::string &prefix, std::ostream &stream)
{
  stream << prefix << *(this->leftCameraNameP) << "\n";
  stream << prefix << *(this->rightCameraNameP) << "\n";
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void RosStereoCamera::InitChild()
{
  // iterate through children of the model parent to find left and right camera sensors
  std::vector<Entity*> children = this->myParent->GetChildren();
  std::vector<Entity*>::iterator iter;

  this->leftCamera = NULL;
  this->rightCamera = NULL;
  for (iter = children.begin(); iter != children.end(); iter++)
  {
    if (dynamic_cast<MonoCameraSensor*>(*iter))
      if ((*iter)->GetName() == this->leftCameraName)
        this->leftCamera = dynamic_cast<MonoCameraSensor*>(*iter);
      else if ((*iter)->GetName() == this->rightCameraName)
        this->rightCamera = dynamic_cast<MonoCameraSensor*>(*iter);
  }

  if (!this->leftCamera || !this->rightCamera)
    gzthrow("RosStereoCamera controller requires 2 MonoCameraSensor's");

  // set parent sensor to active automatically
  this->leftCamera->SetActive(true);
  this->rightCamera->SetActive(true);

  // advertise node topics
  rosnode->advertise<image_msgs::RawStereo>(this->topicName, 1);
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void RosStereoCamera::UpdateChild()
{

  if (!this->leftCamera->IsActive())
    this->leftCamera->SetActive(true);

  if (!this->rightCamera->IsActive())
    this->rightCamera->SetActive(true);

  this->PutCameraData();
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void RosStereoCamera::FiniChild()
{
  rosnode->unadvertise(this->topicName);
  this->leftCamera->SetActive(false);
  this->rightCamera->SetActive(false);
}

////////////////////////////////////////////////////////////////////////////////
// Put camera data to the interface
void RosStereoCamera::PutCameraData()
{
  //CameraData *camera_data = new CameraData();
  const unsigned char *left_src = NULL;
  const unsigned char *right_src = NULL;
  // Get a pointer to image data
  left_src = this->leftCamera->GetImageData(0);


  if (left_src && right_src)
  {
    this->lock.lock();
    rosnode->publish(this->topicName,this->rawStereoMsg);
    this->lock.unlock();
  }

}

