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

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("ros_stereocamera", RosStereoCamera);

////////////////////////////////////////////////////////////////////////////////
// Constructor
RosStereoCamera::RosStereoCamera(Entity *parent)
    : Controller(parent)
{
  this->myParent = dynamic_cast<StereoCameraSensor*>(this->parent);

  Param::Begin(&this->parameters);
  this->leftCameraNameP = new ParamT<std::string>("leftcamera","", 1);
  this->rightCameraNameP = new ParamT<std::string>("rightcamera","", 1);
  Param::End();

  if (!this->myParent)
    gzthrow("RosStereoCamera controller requires a Stereo Camera Sensor as its parent");

  rosnode = ros::g_node; // comes from where?
  int argc = 0;
  char** argv = NULL;
  if (rosnode == NULL)
  {
    // this only works for a single camera.
    ros::init(argc,argv);
    rosnode = new ros::Node("ros_gazebo",ros::Node::DONT_HANDLE_SIGINT);
    printf("-------------------- starting node in stereo camera \n");
  }
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
RosStereoCamera::~RosStereoCamera()
{
  delete this->leftCameraNameP;
  delete this->rightCameraNameP;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void RosStereoCamera::LoadChild(XMLConfigNode *node)
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
    gzthrow("RosStereoCamera controller requires a StereoCameraIface");

  // set parent sensor to active automatically
  this->myParent->SetActive(true);

  this->leftCloudTopicName = node->GetString("leftCloudTopicName","default_ros_stereocamera_left_cloud",0); //read from xml file
  this->rightCloudTopicName = node->GetString("rightCloudTopicName","default_ros_stereocamera_right_cloud",0); //read from xml file
  this->leftTopicName = node->GetString("leftTopicName","default_ros_stereocamera_left_image",0); //read from xml file
  this->rightTopicName = node->GetString("rightTopicName","default_ros_stereocamera_right_image",0); //read from xml file
  this->leftFrameName = node->GetString("leftFrameName","default_ros_stereocamera_left_frame",0); //read from xml file
  this->rightFrameName = node->GetString("rightFrameName","default_ros_stereocamera_right_frame",0); //read from xml file

  std::cout << "================= " << this->leftCloudTopicName << std::endl;
  rosnode->advertise<std_msgs::PointCloud>(this->leftCloudTopicName);
  rosnode->advertise<std_msgs::PointCloud>(this->rightCloudTopicName);
  rosnode->advertise<std_msgs::Image>(this->leftTopicName);
  rosnode->advertise<std_msgs::Image>(this->rightTopicName);
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
}

////////////////////////////////////////////////////////////////////////////////
/// True if a stereo iface is connected
bool RosStereoCamera::StereoIfaceConnected() const
{
  // always on
  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void RosStereoCamera::UpdateChild()
{
  std::map< std::string, CameraIface*>::iterator iter;

  for (iter = this->cameraIfaces.begin(); 
       iter != this->cameraIfaces.end(); iter++)
  {
    iter->second->Lock(1);

    if ( true || iter->second->data->head.openCount > 0)
    {
      if (**(this->leftCameraNameP) == iter->first)
        this->PutCameraData( iter->second->data, 0 );
      else
        this->PutCameraData( iter->second->data, 1 );
    }

    iter->second->Unlock();
    iter->second->Post();
  }

  if (this->stereoIface)
  {
    this->stereoIface->Lock(1);
    if (true || this->stereoIface->data->head.openCount > 0)
      this->PutStereoData();
    this->stereoIface->Unlock();

    this->stereoIface->Post();
  }
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void RosStereoCamera::FiniChild()
{
  rosnode->unadvertise(this->leftCloudTopicName);
  rosnode->unadvertise(this->rightCloudTopicName);
  rosnode->unadvertise(this->leftTopicName);
  rosnode->unadvertise(this->rightTopicName);
}

////////////////////////////////////////////////////////////////////////////////
// Put stereo data to the interface
void RosStereoCamera::PutStereoData()
{
  //StereoCameraData* stereo_data = new StereoCameraData();
  StereoCameraData* stereo_data = this->stereoIface->data;
  int sizeFloat;

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

  sizeFloat  = stereo_data->width * stereo_data->height;

  // Copy the left depth data to the interface
  disp_src = this->myParent->GetDepthData(0);
  disp_dst = stereo_data->left_depth;
  memcpy(disp_dst, disp_src, stereo_data->left_depth_size);

  // Copy into ros
  if (disp_src)
  {
    this->lock.lock();
    // copy data into point cloud
    this->leftCloudMsg.header.frame_id = this->leftFrameName;
    this->leftCloudMsg.header.stamp.sec = (unsigned long)floor(stereo_data->head.time);
    this->leftCloudMsg.header.stamp.nsec = (unsigned long)floor(  1e9 * (  stereo_data->head.time - this->leftCloudMsg.header.stamp.sec) );
    this->leftCloudMsg.set_pts_size(sizeFloat);
    this->leftCloudMsg.set_chan_size(sizeFloat);
    std::cout << " stereo size " << sizeFloat << std::endl;

    // which way first?
    for (unsigned int c=0; c< stereo_data->width; c++)
    for (unsigned int r=0; r< stereo_data->height; r++)
    {
      // direct access of image element
      int n = stereo_data->width * r + c;
      // get angles in image frame, from [-hfov/2,-vfov/2] to [hfov/2,vfov/2]
      double a = stereo_data->hfov * ((double)c - (double)stereo_data->width/2.0)/((double)stereo_data->width/2.0);
      double b = stereo_data->vfov * ((double)r - (double)stereo_data->height/2.0)/((double)stereo_data->height/2.0);
      // analytical solution for 2 transforms
      this->leftCloudMsg.pts[n].x =  cos(b)*cos(a)*disp_src[n];
      this->leftCloudMsg.pts[n].y =         sin(a)*disp_src[n];
      this->leftCloudMsg.pts[n].z = -sin(b)*cos(a)*disp_src[n];
      // std::cout << " cloud (" << r << "," << c << ") "
      //           << " x:" << this->leftCloudMsg.pts[n].x
      //           << " y:" << this->leftCloudMsg.pts[n].y
      //           << " z:" << this->leftCloudMsg.pts[n].z
      //           << std::endl;
      this->leftCloudMsg.chan[n].set_vals_size(1);
      this->leftCloudMsg.chan[n].vals[0] = 255;
    }
    rosnode->publish(this->leftCloudTopicName,this->leftCloudMsg);
    this->lock.unlock();

  }

  // Copy the right depth data to the interface
  disp_src = this->myParent->GetDepthData(1);
  disp_dst = stereo_data->right_depth;
  memcpy(disp_dst, disp_src, stereo_data->right_depth_size);

  // Copy into ros
  if (disp_src)
  {
    this->lock.lock();
    // copy data into point cloud
    this->rightCloudMsg.header.frame_id = this->rightFrameName;
    this->rightCloudMsg.header.stamp.sec = (unsigned long)floor(stereo_data->head.time);
    this->rightCloudMsg.header.stamp.nsec = (unsigned long)floor(  1e9 * (  stereo_data->head.time - this->rightCloudMsg.header.stamp.sec) );
    this->rightCloudMsg.set_pts_size(sizeFloat);
    this->rightCloudMsg.set_chan_size(sizeFloat);

    // which way first?
    for (unsigned int c=0; c< stereo_data->width; c++)
    for (unsigned int r=0; r< stereo_data->height; r++)
    {
      // direct access of image element
      int n = stereo_data->width * r + c;
      // get angles in image frame, from [-hfov/2,-vfov/2] to [hfov/2,vfov/2]
      double a = stereo_data->hfov * ((double)c - (double)stereo_data->width/2.0)/((double)stereo_data->width/2.0);
      double b = stereo_data->vfov * ((double)r - (double)stereo_data->height/2.0)/((double)stereo_data->height/2.0);
      // analytical solution for 2 transforms
      this->rightCloudMsg.pts[n].x =  cos(b)*cos(a)*disp_src[n];
      this->rightCloudMsg.pts[n].y =         sin(a)*disp_src[n];
      this->rightCloudMsg.pts[n].z = -sin(b)*cos(a)*disp_src[n];
      // std::cout << " cloud (" << r << "," << c << ") "
      //           << " x:" << this->rightCloudMsg.pts[n].x
      //           << " y:" << this->rightCloudMsg.pts[n].y
      //           << " z:" << this->rightCloudMsg.pts[n].z
      //           << std::endl;
      this->rightCloudMsg.chan[n].set_vals_size(1);
      this->rightCloudMsg.chan[n].vals[0] = 255;
    }
    rosnode->publish(this->rightCloudTopicName,this->rightCloudMsg);
    this->lock.unlock();
  }
}

////////////////////////////////////////////////////////////////////////////////
// Put camera data to the interface
void RosStereoCamera::PutCameraData(CameraData *camera_data, unsigned int camera)
{
  //CameraData *camera_data = new CameraData();
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
    if (camera==0)
      this->imageMsg[camera].header.frame_id = this->leftFrameName;
    else
      this->imageMsg[camera].header.frame_id = this->rightFrameName;
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
    ///\todo FIXME checkme John
    memcpy(&(this->imageMsg[camera].data[0]), rgb_src, buf_size);
    //    this->imageMsg[camera].data        = (unsigned char*)rgb_src;

    // publish to ros
    if (camera==0)
      rosnode->publish(this->leftTopicName,this->imageMsg[camera]);
    else
      rosnode->publish(this->rightTopicName,this->imageMsg[camera]);

    this->lock.unlock();
  }

}

