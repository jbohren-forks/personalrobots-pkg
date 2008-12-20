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
 * Desc: A ros ptz controller
 * Author: Nathan Koenig
 * Date: 26 Nov 2007
 * SVN: $Id$
 */

#ifndef ROS_PTZ_HH
#define ROS_PTZ_HH

#include <gazebo/Param.hh>
#include <gazebo/Controller.hh>

// ros messages
#include <ros/node.h>
#include <rosthread/mutex.h>

// messages for controlling ptz
#include <axis_cam/PTZActuatorState.h>
#include <axis_cam/PTZActuatorCmd.h>

namespace gazebo
{
  class HingeJoint;

/// @addtogroup gazebo_dynamic_plugins Gazebo ROS Dynamic Plugins
/// @{
/** \defgroup Ros_PTZ ROS PTZ Camera Controller Plugin

  \brief ROS pan-tilt-zoom controller.
  
  This is a controller that controls a pan, tilt, zoom unit 

  Example Usage:
  \verbatim
  <model:physical name="ptz_model">
    <controller:Ros_PTZ name="ptz_controller" plugin="libRos_PTZ.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>15.0</updateRate>
      <panJoint>ptz_pan_joint_name</panJoint>
      <tiltJoint>ptz_tilt_joint_name</tiltJoint>
      <commandTopicName>camera_name/ptz_cmd</commandTopicName>
      <stateTopicName>camera_name/ptz_state</stateTopicName>
      <interface:ptz name="ptz_iface" />
    </controller:Ros_PTZ>
  </model:phyiscal>
  \endverbatim
 
\{
*/

/**

      \brief ROS Pan/Tilt/Zoom Camera Controller
             \li Starts a ROS node if none exists.
             \li Simulates PTZ camera actuators.
                  - publish state information (PT angles) to ROS topic: \e camera_name/ptz_state
                  - subscribe to ROS topic: \e camera_name/ptz_cmd
             \li Example Usage:
  \verbatim
  <model:physical name="ptz_model">
    <controller:Ros_PTZ name="ptz_controller" plugin="libRos_PTZ.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>15.0</updateRate>
      <panJoint>ptz_pan_joint_name</panJoint>
      <tiltJoint>ptz_tilt_joint_name</tiltJoint>
      <commandTopicName>camera_name/ptz_cmd</commandTopicName>
      <stateTopicName>camera_name/ptz_state</stateTopicName>
      <interface:ptz name="ptz_iface" />
    </controller:Ros_PTZ>
  </model:phyiscal>
  \endverbatim
             .
*/
  class Ros_PTZ : public Controller
  {
    /// \brief Constructor
    /// \param parent The parent entity, must be a Model or a Sensor
    public: Ros_PTZ(Entity *parent);
  
    /// \brief Destructor
    public: virtual ~Ros_PTZ();
  
    /// \brief Load the controller
    /// \param node XML config node
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
              
    /// \brief Reset the controller
    protected: virtual void ResetChild();
  
    /// \brief Put camera data to the ROS topic
    private: void PutPTZData();
  
    /// \brief The parent sensor
    private: Model *myParent;

    /// \brief Pan joint
    private: HingeJoint *panJoint;

    /// \brief Tilt joint
    private: HingeJoint *tiltJoint;

    private: float cmdTilt;
    private: float cmdPan;

    private: ParamT<double> *motionGainP;
    private: ParamT<double> *forceP;

    private: ParamT<std::string> *panJointNameP;
    private: ParamT<std::string> *tiltJointNameP;
    private: ParamT<std::string> *commandTopicNameP;
    private: ParamT<std::string> *stateTopicNameP;

    /// \brief pointer to ros node
    private: ros::node *rosnode;
    /// \brief ros message
    private: axis_cam::PTZActuatorState PTZStateMessage;
    private: axis_cam::PTZActuatorCmd   PTZControlMessage;

    /// \brief receive message
    private: void PTZCommandReceived();

    /// \brief topic name
    private: std::string commandTopicName;
    private: std::string stateTopicName;

    /// \brief frame transform name, should match link name
    /// \brief FIXME: extract link name directly? currently using joint names
    private: std::string panFrameName;
    private: std::string tiltFrameName;

    /// \brief A mutex to lock access to fields that are used in message callbacks
    private: ros::thread::mutex lock;

  };
  
  /** \} */
  /// @}

}

#endif

