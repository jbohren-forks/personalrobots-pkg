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

// Original version: Melonee Wise <mwise@willowgarage.com>

#pragma once

// ROS stuff
#include <ros/node.h>
#include <rosthread/mutex.h>

// Controllers
#include <mechanism_model/controller.h>
#include <mechanism_model/joint.h>
#include <robot_mechanism_controllers/joint_position_controller.h>

// Services
#include <robot_msgs/JointCmd.h>
#include <robot_srvs/GetJointCmd.h>
#include <std_msgs/PointStamped.h>

// Math utils
#include <math_utils/angles.h>
#include <tf/transform_listener.h>

namespace controller
{
// The maximum number of joints expected in a head.
static const int MAX_HEAD_JOINTS = 2;

/***************************************************/
/*! \class controller::HeadPanTiltController
    \brief Head Pan Tilt Controller

    This class closes the loop around the joint angles.

    Example config:<br>

    <controller name="head_controller" type="HeadPanTiltControllerNode"><br>
      <listen_topic name="head_commands" /><br>
    
      <controller name="head_pan_controller" topic="head_pan_controller" type="JointPositionController"><br>
        <joint name="head_pan_joint" ><br>
          <pid p="1.5" d="0.1" i="0.3" iClamp="0.2" /><br>
        </joint><br>
      </controller><br>
      
      <controller name="head_tilt_controller" topic="head_tilt_controller" type="JointPositionController"><br>
        <joint name="head_tilt_joint" ><br>
          <pid p="0.8" d="0.05" i="0.1" iClamp="0.1" /><br>
        </joint><br>
      </controller><br>
    </controller> <br> 
*/
/***************************************************/

class HeadPanTiltController : public Controller
{
public:

  /*!
   * \brief Default Constructor of the HeadPanTiltController class.
   *
   */
  HeadPanTiltController();

  /*!
   * \brief Destructor of the HeadPanTiltController class.
   */
  virtual ~HeadPanTiltController();

  /*!
   * \brief Functional way to initialize limits and gains.
   *
   */
  bool initXml(mechanism::RobotState *robot, TiXmlElement *config);

  /*!
   * \brief Sets goals for some joint, specified as a pair of (joint values, joint names).
   *
   * \param j_values the positions of the joints.
   * \param j_names the names of the joints.
   */
  void setJointCmd(const std::vector<double> &j_values, const std::vector<std::string> & j_names);

  /*!
   * \brief Returns the commanded joint poisitions, specified as a pair of (joint values, joint names).
   *
   * \param cmd names and positions.
   */
  void getJointCmd(robot_msgs::JointCmd & cmd) const;

  /*!
   * \brief Returns the position of the controller in the vector.
   *
   * \param name the name of the joint being controlled by the controller (i.e. head_pan_joint).
   */
  int getJointControllerByName(std::string name);

  /*!
   * \brief Issues commands to the joint.
   */
  virtual void update(void); // Real time safe.
  
  unsigned int num_joints_; /**< Number of joints. */
  controller::JointPositionController* getJointPositionControllerByName(std::string name); /**< Joints we're controlling. */
  std::vector<JointPositionController *> joint_position_controllers_; /**< Vector of the joint controllers. */ 


private:

  double last_time_;            /**< The last time. */
  std::vector<double> set_pts_; /**< The vector of joint set_pts. */
  mechanism::Robot* robot_;     /**< The robot we're controlling. */

  /*!
   * \brief Issues commands to the joint.
   */
  void updateJointControllers(void); 

};

/***************************************************/
/*! \class controller::HeadPanTiltControllerNode
    \brief Head Pan Tilt Controller Node

    This class sets the joint angles given a point or
    joint angles.

*/
/***************************************************/

class HeadPanTiltControllerNode : public Controller
{
  public:
    
    /*!
     * \brief Default Constructor
     */
    HeadPanTiltControllerNode();

    /*!
     * \brief Destructor
     */
    ~HeadPanTiltControllerNode();

    void update();

    bool initXml(mechanism::RobotState *robot, TiXmlElement *config);

    /*!
     * \brief Sets a command for all the joints managed by the controller at once.
     * 
     * \param joint_cmds_ (names, positions)
     */
    void setJointCmd();

    /*!
     * \brief Gets the commands for all the joints managed by the controller at once.
     * 
     * \param req 
     * \param resp (positions)
     */
    bool getJointCmd(robot_srvs::GetJointCmd::request &req,
                     robot_srvs::GetJointCmd::response &resp);
    /*!
     * \brief Tracks a point in a specified frame.
     * 
     * \param track_point_ (header, point)
     */
    void trackPoint();
  private:
    std_msgs::PointStamped track_point_; /**< The point from the subscription. */
    robot_msgs::JointCmd joint_cmds_;    /**< The joint commands from the subscription.*/
    HeadPanTiltController *c_;           /**< The controller. */
    std::string service_prefix;          /**< The service name. */
    ros::node *node;                     /**< The node. */
    tf::TransformListener TF;            /**< The transform for converting from point to head and tilt frames. */

};
}


