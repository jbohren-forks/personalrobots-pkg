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
#include <robot_msgs/PointCloud.h>
#include <image_msgs/Image.h>

#include <Generic_Camera.hh>
#include <gazebo/gazebo.h>
#include <gazebo/Param.hh>
#include <gazebo/Controller.hh>
#include <StereoCameraSensor.hh>
#include <MonoCameraSensor.hh>


// raw_stereo components
#include <cstdio>

#include "ros/node.h"

#include "image_msgs/RawStereo.h"
#include "cam_bridge.h"

#include "diagnostic_updater/diagnostic_updater.h"

#include <boost/function.hpp>
#include <boost/bind.hpp>

#include "std_msgs/Empty.h"

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
          <stereoTopicName>stereo_right_image</stereoTopicName>
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
  /// \param parent The parent entity, must be a Model
  public: RosStereoCamera(Entity *parent);

  /// \brief Destructor
  public: virtual ~RosStereoCamera();

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

  /// \brief The parent sensor
  private: Entity *myParent;

  /// \brief parameters
  private: ParamT<std::string> *leftCameraNameP;
  private: ParamT<std::string> *rightCameraNameP;
  private: ParamT<std::string> *topicNameP;
  private: ParamT<std::string> *leftFrameNameP;
  private: ParamT<std::string> *rightFrameNameP;
  private: ParamT<double> *CxPrimeP;           // rectified optical center x, for sim, CxPrime == Cx
  private: ParamT<double> *CxP;            // optical center x
  private: ParamT<double> *CyP;            // optical center y
  private: ParamT<double> *focal_lengthP;  // also known as focal length
  private: ParamT<double> *distortion_k1P; // linear distortion
  private: ParamT<double> *distortion_k2P; // quadratic distortion
  private: ParamT<double> *distortion_k3P; // cubic distortion
  private: ParamT<double> *distortion_t1P; // tangential distortion
  private: ParamT<double> *distortion_t2P; // tangential distortion
  private: ParamT<double> *baselineP;      // shift from left camera to right camera.  we treat LEFT camera as origin

  /// \brief Pointer to Mono Left and Right Cameras
  private: MonoCameraSensor *leftCamera;
  private: MonoCameraSensor *rightCamera;
  /// \brief Stereo Node stuff
  private: std::string leftCameraName;
  private: std::string rightCameraName;
  private: std::string topicName;
  private: std::string leftFrameName;
  private: std::string rightFrameName;
  private: double CxPrime;
  private: double Cx;
  private: double Cy;
  private: double focal_length;
  private: double distortion_k1;
  private: double distortion_k2;
  private: double distortion_k3;
  private: double distortion_t1;
  private: double distortion_t2;
  private: double baseline;

  /// \brief pointer to ros node
  private: ros::Node *rosnode;

  /// \brief ros message
  /// \brief construct raw stereo message
  private: image_msgs::Image leftImageMsg, rightImageMsg;
  private: image_msgs::RawStereo rawStereoMsg;

  /// \brief A mutex to lock access to fields that are used in message callbacks
  private: boost::mutex lock;

  /// \brief Put camera data to the iface
  private: void PutCameraData();

  /// \brief The camera interface
  private: StereoCameraIface *stereoIface;
  private: std::map< std::string, CameraIface*> cameraIfaces;

};

/** \} */
/// @}

}

#endif

