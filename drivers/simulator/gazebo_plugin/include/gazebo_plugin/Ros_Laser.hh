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
 * SVN: $Id: Ros_Laser.hh 6656 2008-06-20 22:52:19Z natepak $
 */

#ifndef ROS_LASER_HH
#define ROS_LASER_HH

#include <gazebo/Controller.hh>

#include <ros/node.h>
#include <std_msgs/PointCloudFloat32.h>

namespace gazebo
{
  class LaserIface;
  class FiducialIface;
  class RaySensor;

/// @addtogroup gazebo_controller
/// @{
/** \defgroup ros ros

  \brief ros laser controller.
   
   This is a controller that collects data from a ray sensor, and populates a libgazebo laser interface. 

  \verbatim
  <model:physical name="laser_model">
    <body:box name="laser_body">

      <sensor:ray name="laser">
        <controller:ros_laser name="controller-name">
          <interface:laser name="iface-name"/>
        </controller:ros_laser>
      </sensor:ray>

    </body:box>
  </model:physical>
  \endverbatim
 
\{
*/

/// \brief ros laser controller.
/// 
/// This is a controller that simulates a ros laser
class Ros_Laser : public Controller
{
  /// \brief Constructor
  /// \param parent The parent entity, must be a Model or a Sensor
  public: Ros_Laser(Entity *parent);

  /// \brief Destructor
  public: virtual ~Ros_Laser();

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

  private: double GaussianKernel(double mu,double sigma);
};

/** /} */
/// @}

}

#endif

