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
 * Desc: A dynamic controller plugin that publishes ROS image topic for generic camera sensor.
 * Author: John Hsu
 * Date: 24 Sept 2008
 * SVN: $Id$
 */
#ifndef ROS_CAMERA_HH
#define ROS_CAMERA_HH

#include <ros/node.h>
#include <std_msgs/Image.h>
#include <gazebo/Controller.hh>

namespace gazebo
{
  class CameraIface;
  class MonoCameraSensor;

/// @addtogroup gazebo_dynamic_plugins Gazebo ROS Dynamic Plugins
/// @{
/** \defgroup roscamera Ros Camera Plugin

  \brief Ros camera controller.
  
  This is a controller that collects data from a Camera Sensor and populates a libgazebo camera interfaace as well as publish a ROS std_msgs::Image (under topicName). This controller should only be used as a child of a camera sensor 

  \verbatim
  <model:physical name="camera_model">
    <body:empty name="camera_body_name">
      <sensor:camera name="camera_sensor">
        <controller:ros_camera name="controller-name" plugin="libRos_Camera.so">
            <alwaysOn>true</alwaysOn>
            <updateRate>15.0</updateRate>
            <topicName>camera_name/image</topicName>
            <frameName>camera_body_name</frameName>
            <interface:camera name="ros_camera_iface" />
        </controller:ros_camera>
      </sensor:camera>
    </body:empty>
  </model:phyiscal>
  \endverbatim
 
\{
*/

/// \brief Ros camera controller.
/// 
/// This is a controller that simulates a generic camera
class Ros_Camera : public Controller
{
  /// \brief Constructor
  /// \param parent The parent entity, must be a Model or a Sensor
  public: Ros_Camera(Entity *parent);

  /// \brief Destructor
  public: virtual ~Ros_Camera();

  /// \brief Load the controller
  /// \param node XML config node
  /// \return 0 on success
  protected: virtual void LoadChild(XMLConfigNode *node);

  /// \brief Init the controller
  /// \return 0 on success
  protected: virtual void InitChild();

  /// \brief Update the controller
  /// \return 0 on success
  protected: virtual void UpdateChild();

  /// \brief Finalize the controller, unadvertise topics and shutdown ros node.
  /// \return 0 on success
  protected: virtual void FiniChild();

  /// \brief Put camera data to the iface
  private: void PutCameraData();

  /// The camera interface
  private: CameraIface *cameraIface;

  /// The parent sensor
  private: MonoCameraSensor *myParent;

  // pointer to ros node
  private: ros::node *rosnode;

  // ros message
  private: std_msgs::Image imageMsg;

  // topic name
  private: std::string topicName;

  // frame transform name, should match link name
  // FIXME: extract link name directly?
  private: std::string frameName;

  // A mutex to lock access to fields that are used in message callbacks
  private: ros::thread::mutex lock;

};

/** /} */
/// @}

}
#endif

