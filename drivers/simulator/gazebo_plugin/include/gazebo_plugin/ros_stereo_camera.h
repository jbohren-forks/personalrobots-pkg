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
 * Desc: A stereo camera controller
 * Author: Nathan Koenig
 * Date: 06 April 2008
 * SVN: $Id:$
 */

#ifndef ROS_STEREO_CAMERA_HH
#define ROS_STEREO_CAMERA_HH

#include <map>

#include <ros/node.h>
#include "boost/thread/mutex.hpp"
#include <std_msgs/PointCloud.h>
#include <std_msgs/Image.h>

#include <Generic_Camera.hh>
#include <gazebo/gazebo.h>
#include <gazebo/Param.hh>
#include <gazebo/Controller.hh>
#include <StereoCameraSensor.hh>

namespace gazebo
{
  class CameraIface;
  class StereoCameraIface;

/// @addtogroup gazebo_dynamic_plugins Gazebo ROS Dynamic Plugins
/// @{
/** \defgroup RosStereoCamera ROS stereo camera controller plugin

  \brief Stereo camera controller.
  
  This is a controller that collects data from a Stereo Camera Sensor and populates a libgazebo stereo camera interfaace. This controller should only be used as a child of a stereo camera sensor 

  \verbatim
  <model:physical name="camera_model">
    <body:empty name="camera_body">
      <sensor:stereocamera name="stereo_camera_sensor">
        <imageSize>640 480</imageSize>
        <hfov>60</hfov>
        <nearClip>0.1</nearClip>
        <farClip>100</farClip>
        <saveFrames>false</saveFrames>
        <saveFramePath>frames</saveFramePath>
        <baseline>0.2</baseline>
        <updateRate>15.0</updateRate>
        <controller:ros_stereocamera name="stereo_camera_controller" plugin="libros_stereo_camera.so">
          <alwaysOn>true</alwaysOn>
          <updateRate>15.0</updateRate>
          <interface:stereocamera name="stereo_iface_0" />
          <interface:camera name="camera_iface_0" />
          <interface:camera name="camera_iface_1" />
          <leftcamera>camera_iface_0</leftcamera>
          <rightcamera>camera_iface_1</rightcamera>
          <leftCloudTopicName>stereo_left_cloud</leftCloudTopicName>
          <rightCloudTopicName>stereo_right_cloud</rightCloudTopicName>
          <leftTopicName>stereo_left_image</leftTopicName>
          <rightTopicName>stereo_right_image</rightTopicName>
          <leftFrameName>stereo_left</leftFrameName>
          <rightFrameName>stereo_right</rightFrameName>
        </controller:ros_stereocamera>
      </sensor:stereocamera>
    </body:empty>
  </model:phyiscal>
  \endverbatim
 
\{
*/

/// \brief Stereo camera controller.
/// 
/// This is a controller that simulates a stereo camera
class RosStereoCamera : public Controller
{
  /// \brief Constructor
  /// \param parent The parent entity, must be a Model or a Sensor
  public: RosStereoCamera(Entity *parent);

  /// \brief Destructor
  public: virtual ~RosStereoCamera();

  /// \brief True if a stereo iface is connected
  public: bool StereoIfaceConnected() const;

  /// \brief Load the controller
  /// \param node XML config node
  /// \return 0 on success
  protected: virtual void LoadChild(XMLConfigNode *node);

  /// \brief Save the controller.
  ///        stream Output stream
  protected: void SaveChild(std::string &prefix, std::ostream &stream);

  /// \brief Init the controller
  protected: virtual void InitChild();

  /// \brief Update the controller
  protected: virtual void UpdateChild();

  /// \brief Finalize the controller
  protected: virtual void FiniChild();

  /// \brief Put stereo data to the iface
  private: void PutStereoData();

  /// \brief Put camera data to the iface
  private: void PutCameraData(CameraData *camera_data, unsigned int camera);

  /// \brief The camera interface
  private: StereoCameraIface *stereoIface;
  private: std::map< std::string, CameraIface*> cameraIfaces;

  private: ParamT<std::string> *leftCameraNameP;
  private: ParamT<std::string> *rightCameraNameP;

  /// \brief The parent sensor
  private: StereoCameraSensor *myParent;

  /// \brief pointer to ros node
  private: ros::Node *rosnode;

  /// \brief ros message
  private: std_msgs::PointCloud leftCloudMsg;
  private: std_msgs::PointCloud rightCloudMsg;
  /// \brief ros message
  private: std_msgs::Image imageMsg[2];

  /// \brief topic name
  private: std::string leftCloudTopicName;
  private: std::string rightCloudTopicName;
  private: std::string leftTopicName;
  private: std::string rightTopicName;

  /// \brief frame transform name, should match link name
  /// \brief FIXME: extract link name directly?
  private: std::string leftFrameName;
  private: std::string rightFrameName;

  /// \brief A mutex to lock access to fields that are used in message callbacks
  private: boost::mutex lock;

};

/** \} */
/// @}

}

#endif

