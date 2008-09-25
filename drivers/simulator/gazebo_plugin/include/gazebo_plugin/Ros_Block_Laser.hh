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
 * Desc: ros laser controller.
 * Author: Nathan Koenig
 * Date: 01 Feb 2007
 * SVN: $Id: Ros_Block_Laser.hh 6656 2008-06-20 22:52:19Z natepak $
 */

#ifndef ROS_BLOCK_LASER_HH
#define ROS_BLOCK_LASER_HH

#include <gazebo/Controller.hh>

#include <ros/node.h>
#include <std_msgs/LaserScan.h>
#include <std_msgs/PointCloudFloat32.h>

namespace gazebo
{
  class LaserIface;
  class FiducialIface;
  class RaySensor;

/// @addtogroup gazebo_dynamic_plugins Gazebo ROS Dynamic Plugins
/// @{
/** \defgroup Ros_Block_Laser ROS Block Laser Scanner Controller Plugin

  \brief ROS Block Laser Scanner Controller Plugin
  
  This is a controller that gathers range data from a ray sensor, and returns results via publishing ROS topic for point clouds and Iface.

  \verbatim
    <model:physical name="ray_model">
      <body:empty name="ray_body_name">
        <sensor:ray name="ray_sensor">
          <rayCount>30</rayCount>
          <rangeCount>30</rangeCount>
          <laserCount>1</laserCount>
          
          <origin>0.0 0.0 0.05</origin>
          <displayRays>false</displayRays>
          
          <minAngle>-15</minAngle>
          <maxAngle> 15</maxAngle>
          
          <minRange>0.05</minRange>
          <maxRange>100.0</maxRange>
          <updateRate>10.0</updateRate>

          <verticalRayCount>30</verticalRayCount>
          <verticalRangeCount>30</verticalRangeCount>
          <verticalMinAngle>-20</verticalMinAngle>
          <verticalMaxAngle>  0</verticalMaxAngle>

          <controller:ros_block_laser name="ray_block_controller" plugin="libRos_Block_Laser.so">
            <gaussianNoise>0.005</gaussianNoise>
            <alwaysOn>true</alwaysOn>
            <updateRate>10.0</updateRate>
            <topicName>full_cloud</topicName>
            <frameName>ray_model</frameName>
            <interface:laser name="ray_block_iface" />
          </controller:ros_block_laser>
        </sensor:ray>
      </body:empty>
    </model:phyiscal>
  \endverbatim
 
\{
*/

/// \brief ros laser controller.
/// 
/// This is a controller that simulates a ros laser
class Ros_Block_Laser : public Controller
{
  /// \brief Constructor
  /// \param parent The parent entity, must be a Model or a Sensor
  public: Ros_Block_Laser(Entity *parent);

  /// \brief Destructor
  public: virtual ~Ros_Block_Laser();

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

  /// \brief Finalize the controller
  /// \return 0 on success
  protected: virtual void FiniChild();

  /// \brief Put laser data to the iface
  private: void PutLaserData();

  /// \brief Put fiducial data to the iface
  private: void PutFiducialData();

  /// The laser interface
  private: LaserIface *laserIface;

  private: FiducialIface *fiducialIface;

  /// The parent sensor
  private: RaySensor *myParent;

  // pointer to ros node
  private: ros::node *rosnode;

  // ros message
  private: std_msgs::PointCloudFloat32 cloudMsg;
 
  // topic name
  private: std::string topicName;

  // frame transform name, should match link name
  // FIXME: extract link name directly?
  private: std::string frameName;

  private: double gaussianNoise;

  private: double GaussianKernel(double mu,double sigma);

  // A mutex to lock access to fields that are used in message callbacks
  private: ros::thread::mutex lock;

};

/** \} */
/// @}

}

#endif

