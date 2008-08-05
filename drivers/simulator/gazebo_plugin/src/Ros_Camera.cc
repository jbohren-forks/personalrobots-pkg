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
 * Desc: Actuator array controller for a Pr2 robot.
 * Author: Sachin Chitta and John Hsu
 * Date: 1 June 2008
 * SVN info: $Id$
 */


#include <algorithm>
#include <assert.h>

#include <gazebo/Sensor.hh>
#include <gazebo/Model.hh>
#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>
//#include "/u/johnhsu/projects/pr2/3rdparty/gazebo/gazebo-svn/server/sensors/camera/MonoCameraSensor.hh"
#include "MonoCameraSensor.hh"
#include <gazebo_plugin/Ros_Camera.hh>

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("ros_camera", Ros_Camera);

////////////////////////////////////////////////////////////////////////////////
// Constructor
Ros_Camera::Ros_Camera(Entity *parent)
    : Controller(parent)
{
  this->myParent = dynamic_cast<MonoCameraSensor*>(this->parent);

  if (!this->myParent)
    gzthrow("Ros_Camera controller requires a Camera Sensor as its parent");


  rosnode = ros::g_node; // comes from where?
  int argc = 0;
  char** argv = NULL;
  if (rosnode == NULL)
  {
    // this only works for a single camera.
    ros::init(argc,argv);
    rosnode = new ros::node("ros_gazebo");
    printf("-------------------- starting node in camera \n");
  }


}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Ros_Camera::~Ros_Camera()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void Ros_Camera::LoadChild(XMLConfigNode *node)
{
  this->cameraIface = dynamic_cast<CameraIface*>(this->ifaces[0]);

  if (!this->cameraIface)
    gzthrow("Ros_Camera controller requires a CameraIface");

  this->topicName = node->GetString("topicName","default_ros_camera",0); //read from xml file

  std::cout << "================= " << this->topicName << std::endl;
  rosnode->advertise<std_msgs::Image>(this->topicName);
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void Ros_Camera::InitChild()
{
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void Ros_Camera::UpdateChild()
{

  // do this first so there's chance for sensor to run 1 frame after activate
  if (this->myParent->IsActive())
    this->PutCameraData();

  // activate if iface open
  if (this->cameraIface->Lock(1))
  {
    if (this->cameraIface->GetOpenCount() > 0)
      this->myParent->SetActive(true);
    else
      this->myParent->SetActive(false);

    //std::cout << " camera open count " << this->cameraIface->GetOpenCount() << std::endl;
    this->cameraIface->Unlock();
  }
  //std::cout << " camera     active " << this->myParent->IsActive() << std::endl;

}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void Ros_Camera::FiniChild()
{
  // TODO: will be replaced by global ros node eventually
  delete rosnode;
  ros::fini();
}

////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void Ros_Camera::PutCameraData()
{
  CameraData *data = this->cameraIface->data;
  const unsigned char *src;
  unsigned char *dst;
  Pose3d cameraPose;

  this->cameraIface->Lock(1);

  // Data timestamp
  data->head.time = Simulator::Instance()->GetSimTime();

  data->width = this->myParent->GetImageWidth();
  data->height = this->myParent->GetImageHeight();
  data->image_size = data->width * data->height * 3;

  // GetFOV() returns radians
  data->hfov = this->myParent->GetHFOV();
  data->vfov = this->myParent->GetVFOV();

  // Set the pose of the camera
  cameraPose = this->myParent->GetWorldPose();
  data->camera_pose.pos.x = cameraPose.pos.x;
  data->camera_pose.pos.y = cameraPose.pos.y;
  data->camera_pose.pos.z = cameraPose.pos.z;
  data->camera_pose.roll = cameraPose.rot.GetRoll();
  data->camera_pose.pitch = cameraPose.rot.GetPitch();
  data->camera_pose.yaw = cameraPose.rot.GetYaw();

  // Make sure there is room to store the image
  assert (data->image_size <= sizeof(data->image));

  this->lock.lock();
  // Copy the pixel data to the interface
  src = this->myParent->GetImageData(0);
  dst = data->image;

  // TODO: can skip copy to Iface if Iface is not used
  memcpy(dst, src, data->image_size);

  this->cameraIface->Unlock();

  // New data is available
  this->cameraIface->Post();


  // copy data into image
  int    width            = this->myParent->GetImageWidth();
  int    height           = this->myParent->GetImageHeight();
  int    depth            = 3;

  this->image.width       = width;
  this->image.height      = height;
  this->image.compression = "jpeg";
  this->image.colorspace  = "bgr";

  // on first pass, the sensor does not update after cameraIface is opened.
  uint32_t       buf_size = (width) * (height) * (depth);

  this->image.set_data_size(buf_size);
  this->image.data        = (unsigned char*)src;

  // publish to ros
  rosnode->publish(this->topicName,this->image);
  this->lock.unlock();
}

