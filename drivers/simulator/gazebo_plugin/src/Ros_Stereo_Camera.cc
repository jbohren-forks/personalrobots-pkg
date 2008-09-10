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
 * SVN info: $Id: Ros_Stereo_Camera.cc 4436 2008-03-24 17:42:45Z robotos $
 */

#include <algorithm>
#include <assert.h>

#include <gazebo_plugin/Ros_Stereo_Camera.hh>
#include <gazebo/Sensor.hh>
#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("ros_stereocamera", Ros_Stereo_Camera);

////////////////////////////////////////////////////////////////////////////////
// Constructor
Ros_Stereo_Camera::Ros_Stereo_Camera(Entity *parent)
    : Controller(parent)
{
  this->myParent = dynamic_cast<StereoCameraSensor*>(this->parent);

  Param::Begin(&this->parameters);
  this->leftCameraNameP = new ParamT<std::string>("leftcamera","", 1);
  this->rightCameraNameP = new ParamT<std::string>("rightcamera","", 1);
  Param::End();

  if (!this->myParent)
    gzthrow("Ros_Stereo_Camera controller requires a Stereo Camera Sensor as its parent");

  // set parent sensor to active automatically
  this->myParent->SetActive(true);

  rosnode = ros::g_node; // comes from where?
  int argc = 0;
  char** argv = NULL;
  if (rosnode == NULL)
  {
    // this only works for a single camera.
    ros::init(argc,argv);
    rosnode = new ros::node("ros_gazebo",ros::node::DONT_HANDLE_SIGINT);
    printf("-------------------- starting node in stereo camera \n");
  }
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Ros_Stereo_Camera::~Ros_Stereo_Camera()
{
  delete this->leftCameraNameP;
  delete this->rightCameraNameP;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void Ros_Stereo_Camera::LoadChild(XMLConfigNode *node)
{
  std::vector<Iface*>::iterator iter;

  for (iter = this->ifaces.begin(); iter != this->ifaces.end(); iter++)
  {
    if ((*iter)->GetType() == "stereo")
      this->stereoIface = dynamic_cast<StereoCameraIface*>(*iter);
    else if ((*iter)->GetType() == "camera")
    {
      CameraIface *ciface = dynamic_cast<CameraIface*>(*iter);
      this->cameraIfaces[ciface->GetId()] = ciface;
    }
  }

  this->leftCameraNameP->Load(node);
  this->rightCameraNameP->Load(node);

  if (!this->stereoIface)
    gzthrow("Ros_Stereo_Camera controller requires a StereoCameraIface");

  this->topicName = node->GetString("topicName","default_ros_stereocamera",0); //read from xml file
  this->leftTopicName = node->GetString("leftTopicName","default_ros_stereocamera_left_image",0); //read from xml file
  this->rightTopicName = node->GetString("rightTopicName","default_ros_stereocamera_right_image",0); //read from xml file
  this->frameName = node->GetString("frameName","default_ros_stereocamera",0); //read from xml file

  std::cout << "================= " << this->topicName << std::endl;
  rosnode->advertise<std_msgs::PointCloud>(this->topicName);
  rosnode->advertise<std_msgs::Image>(this->leftTopicName);
  rosnode->advertise<std_msgs::Image>(this->rightTopicName);
}

////////////////////////////////////////////////////////////////////////////////
/// Save the controller.
void Ros_Stereo_Camera::SaveChild(std::string &prefix, std::ostream &stream)
{
  stream << prefix << *(this->leftCameraNameP) << "\n";
  stream << prefix << *(this->rightCameraNameP) << "\n";
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void Ros_Stereo_Camera::InitChild()
{
}

////////////////////////////////////////////////////////////////////////////////
/// True if a stereo iface is connected
bool Ros_Stereo_Camera::StereoIfaceConnected() const
{
  // always on
  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void Ros_Stereo_Camera::UpdateChild()
{

  this->PutCameraData( 0 );
  this->PutCameraData( 1 );

  this->PutStereoData();
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void Ros_Stereo_Camera::FiniChild()
{
}

////////////////////////////////////////////////////////////////////////////////
// Put stereo data to the interface
void Ros_Stereo_Camera::PutStereoData()
{
  StereoCameraData* stereo_data = new StereoCameraData();

  const float *disp_src;
  float *disp_dst;

  // Data timestamp
  stereo_data->head.time = Simulator::Instance()->GetSimTime();

  stereo_data->width = this->myParent->GetImageWidth();
  stereo_data->height = this->myParent->GetImageHeight();
  stereo_data->farClip = this->myParent->GetFarClip();
  stereo_data->nearClip = this->myParent->GetNearClip();

  stereo_data->hfov = *(this->myParent->GetHFOV());
  stereo_data->vfov = *(this->myParent->GetVFOV());

  stereo_data->right_depth_size = stereo_data->width * stereo_data->height * sizeof(float);
  stereo_data->left_depth_size = stereo_data->width * stereo_data->height * sizeof(float);

  assert (stereo_data->right_depth_size <= sizeof(stereo_data->right_depth));
  assert (stereo_data->left_depth_size <= sizeof(stereo_data->left_depth));

  // Copy the left depth data to the interface
  disp_src = this->myParent->GetDepthData(0);
  disp_dst = stereo_data->left_depth;
  memcpy(disp_dst, disp_src, stereo_data->left_depth_size);

  // Copy the right depth data to the interface
  disp_src = this->myParent->GetDepthData(1);
  disp_dst = stereo_data->right_depth;
  memcpy(disp_dst, disp_src, stereo_data->right_depth_size);

  //FIXME: the does nothing except get time for now
  if (disp_src)
  {
    this->lock.lock();
    // copy data into point cloud
    this->cloudMsg.header.frame_id = this->frameName;
    this->cloudMsg.header.stamp.sec = (unsigned long)floor(stereo_data->head.time);
    this->cloudMsg.header.stamp.nsec = (unsigned long)floor(  1e9 * (  stereo_data->head.time - this->cloudMsg.header.stamp.sec) );
    //this->cloudMsg.set_data_size(10000);
    //this->cloudMsg.data        = (unsigned char*)NULL;
    // publish to ros
    rosnode->publish(this->topicName,this->cloudMsg);
    this->lock.unlock();
  }
}

////////////////////////////////////////////////////////////////////////////////
// Put camera data to the interface
void Ros_Stereo_Camera::PutCameraData(unsigned int camera)
{
  CameraData *camera_data = new CameraData();
  const unsigned char *rgb_src = NULL;
  unsigned char *rgb_dst = NULL;
  Pose3d cameraPose;

  camera_data->head.time = Simulator::Instance()->GetSimTime();

  camera_data->width = this->myParent->GetImageWidth();
  camera_data->height = this->myParent->GetImageHeight();
  camera_data->image_size = camera_data->width * camera_data->height * 3;
  assert (camera_data->image_size <= sizeof(camera_data->image));

  camera_data->hfov = *(this->myParent->GetHFOV());
  camera_data->vfov = *(this->myParent->GetVFOV());

  // Set the pose of the camera
  cameraPose = this->myParent->GetWorldPose();
  camera_data->camera_pose.pos.x = cameraPose.pos.x;
  camera_data->camera_pose.pos.y = cameraPose.pos.y;
  camera_data->camera_pose.pos.z = cameraPose.pos.z;
  camera_data->camera_pose.roll = cameraPose.rot.GetRoll();
  camera_data->camera_pose.pitch = cameraPose.rot.GetPitch();
  camera_data->camera_pose.yaw = cameraPose.rot.GetYaw();

  // Copy the pixel data to the interface
  rgb_src = this->myParent->GetImageData(camera);
  rgb_dst = camera_data->image;

  memcpy(rgb_dst, rgb_src, camera_data->image_size);


  if (rgb_src)
  {
    this->lock.lock();
    // copy data into image
    this->imageMsg[camera].header.frame_id = this->frameName;
    this->imageMsg[camera].header.stamp.sec = (unsigned long)floor(camera_data->head.time);
    this->imageMsg[camera].header.stamp.nsec = (unsigned long)floor(  1e9 * (  camera_data->head.time - this->imageMsg[camera].header.stamp.sec) );

    int    width            = this->myParent->GetImageWidth();
    int    height           = this->myParent->GetImageHeight();
    int    depth            = 3;

    this->imageMsg[camera].width       = width;
    this->imageMsg[camera].height      = height;
    this->imageMsg[camera].compression = "raw";
    this->imageMsg[camera].colorspace  = "rgb24";

    // on first pass, the sensor does not update after cameraIface is opened.
    uint32_t       buf_size = (width) * (height) * (depth);

    this->imageMsg[camera].set_data_size(buf_size);
    this->imageMsg[camera].data        = (unsigned char*)rgb_src;

    // publish to ros
    if (camera==0)
      rosnode->publish(this->leftTopicName,this->imageMsg[camera]);
    else
      rosnode->publish(this->rightTopicName,this->imageMsg[camera]);

    this->lock.unlock();
  }



}

