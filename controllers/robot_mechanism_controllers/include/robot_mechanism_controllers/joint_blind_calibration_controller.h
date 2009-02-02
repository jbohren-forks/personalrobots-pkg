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

#pragma once

/***************************************************/
/*! \class controller::JointCalibratonController
    \brief Joint Controller that finds zerop point
    \author Timothy Hunter <tjhunter@willowgarage.com>


    This class moves the joint and reads the value of the clibration_reading_ field to find the zero position of the joint. Once these are determined, these values
 * are passed to the joint and enable the joint for the other controllers.

 Example XML:
 <controller type="JointBlindCalibrationController">
   <calibrate joint="upperarm_roll_right_joint"
              actuator="upperarm_roll_right_act"
              transmission="upperarm_roll_right_trans"
              velocity="0.3" />
   <pid p="15" i="0" d="0" iClamp="0" />
 </controller>

*/
/***************************************************/


#include "mechanism_model/controller.h"
#include "robot_mechanism_controllers/joint_velocity_controller.h"
#include "misc_utils/advertised_service_guard.h"

// Services
#include <robot_mechanism_controllers/CalibrateJoint.h>


namespace controller
{

class JointBlindCalibrationController : public Controller
{
public:
  /*!
   * \brief Default Constructor.
   *
   */
  JointBlindCalibrationController();

  /*!
   * \brief Destructor.
   */
  virtual ~JointBlindCalibrationController();

  /*!
   * \brief Functional way to initialize limits and gains.
   *
   */
  virtual bool initXml(mechanism::RobotState *robot, TiXmlElement *config);


  /*!
   * \brief Issues commands to the joint. Should be called at regular intervals
   */
  virtual void update();

  bool calibrated() { return state_ == DONE; }
  void beginCalibration() {
    if (state_ == INITIALIZED)
      state_ = BEGINNING;
  }

protected:

  enum { INITIALIZED, BEGINNING, STARTING_UP, MOVING_UP,
         STARTING_DOWN, MOVING_DOWN, BACKING_OFF, DONE };
  int state_;

  double search_velocity_;
  Actuator *actuator_;
  mechanism::JointState *joint_;
  mechanism::Transmission *transmission_;

  double init_time;

  controller::JointVelocityController vc_; /** The joint velocity controller used to sweep the joint.*/
};


/***************************************************/
/*! \class controller::JointBlindCalibrationControllerNode
    \brief Joint Limit Controller ROS Node

    This class starts and stops the initialization sequence

*/
/***************************************************/

class JointBlindCalibrationControllerNode : public Controller
{
public:
  /*!
   * \brief Default Constructor
   *
   */
  JointBlindCalibrationControllerNode();

  /*!
   * \brief Destructor
   */
  ~JointBlindCalibrationControllerNode();

  void update();

  bool initXml(mechanism::RobotState *robot, TiXmlElement *config);

  // Services
  bool calibrateCommand(robot_mechanism_controllers::CalibrateJoint::Request &req,
                        robot_mechanism_controllers::CalibrateJoint::Response &resp);

private:
  JointBlindCalibrationController *c_;
  AdvertisedServiceGuard guard_calibrate_;
};
}


