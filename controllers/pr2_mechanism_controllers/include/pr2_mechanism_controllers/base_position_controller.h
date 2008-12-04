/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef PR2_MECHANISM_CONTROLLERS_BASE_POSITION_CONTROLLER_H
#define PR2_MECHANISM_CONTROLLERS_BASE_POSITION_CONTROLLER_H

#include "control_toolbox/base_position_pid.h"
#include "pr2_mechanism_controllers/base_controller.h"
#include "tf/transform_listener.h"
#include "std_msgs/PoseStamped.h"
#include "std_msgs/Point.h"

#include "misc_utils/advertised_service_guard.h"

namespace pr2_mechanism_controllers
{

class BasePositionControllerNode : public controller::Controller
{
public :
  BasePositionControllerNode() ;
  ~BasePositionControllerNode() ;

  /**
   * Initializes the Controller.
   * A list of XML sections are necessary for this to work
   * 
   * 
   * 
   * 
   */
  bool initXml(mechanism::RobotState *robot_state, TiXmlElement *config) ;
  
  /**
   * Realtime safe update method called from the realtime loop. Compares the target position to the current position,
   * generates error terms, and then uses these to generate a Velocity command for BaseController
   */
  void update() ;

  /**
   * Sets the target pose of the controller. The cmd is transformed and stored into the odometric frame using the latest transform. The x,y position
   * of the pose define the target location of the robot (in the odometric frame). The theta rotation of the robot is defined by the pure z rotation that is
   * closest to the orientation of the pose in the odometric frame.
   * \param cmd The pose that we want to reach
   */
  void setPoseCommand(std_msgs::PoseStamped cmd) ;

  /**
   * Sets an x,y,theta position for the base to reach in wheel odometry frame.  This command doesn't do any transforms. It simply sets the PID targets
   * for the 3 different axes of the base.
   * \param x The x position that we want to reach [in the odometric frame]
   * \param y The y position that we want to reach [in the odometric frame]
   * \param w The theta angle that we want to reach [in the odometric frame]
   */
  void setPoseOdomFrameCommand(double x, double y, double w) ;
  
private :
  control_toolbox::BasePositionPid base_position_pid_ ;        // Does the math to compute a command velocity
  controller::BaseControllerNode base_controller_node_ ;       // Converts a commanded velocity into a wheel velocities and turret angles
  ros::node *node_ ;

  mechanism::RobotState *robot_state_ ;
  
  double last_time_ ;                                          // Store the last time that we called the update
  
  SubscriptionGuard guard_set_pose_command_ ;                  // Automatically unsubscribes set_pose_cmd
  void setPoseCommandCallback() ;                              // Ros callback for "set_pose_command" messages

  SubscriptionGuard guard_set_pose_odom_frame_command_ ;       // Automatically unsubscribes set_pose_odom_frame_cmd
  void setPoseOdomFrameCommandCallback() ;                     // Ros callback for "set_pose_command_odom_frame" messages
  
  tf::Vector3 xyt_target_ ;                                    // The current x,y,theta target in position space (NOT velocity space)
  
  tf::TransformListener tf_ ;                                  // Transformer used to convert commands from native frame into odometric frame
  std::string odom_frame_name_ ;                               // Stores the name of the odometric frame. This is the frame that we control in
  
  // Message Holders
  std_msgs::PoseStamped pose_cmd_ ;
  std_msgs::Point pose_odom_frame_cmd_ ;
} ;

}

#endif // PR2_MECHANISM_CONTROLLERS_BASE_POSITION_CONTROLLER_H
