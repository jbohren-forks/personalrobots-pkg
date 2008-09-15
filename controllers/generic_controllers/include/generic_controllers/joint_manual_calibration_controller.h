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

 */
/***************************************************/


#include <ros/node.h>
#include <generic_controllers/controller.h>
#include <generic_controllers/joint_velocity_controller.h>
#include <mechanism_model/robot.h>
#include <hardware_interface/hardware_interface.h>

// Services
#include <generic_controllers/CalibrateJoint.h>

//FIXME: the editor messed up the indentation
namespace controller
{

  class JointManualCalibrationController : public Controller
  {
    public:
  /*!
     * \brief Default Constructor.
     *
   */
      JointManualCalibrationController();

  /*!
       * \brief Destructor.
   */
      virtual ~JointManualCalibrationController();

  /*!
       * \brief Functional way to initialize limits and gains.
       *
   */
      virtual bool initXml(mechanism::RobotState *robot, TiXmlElement *config);

  /*!
       * \brief Sets the joint in motion to find the reference position.
       * 
       * There are two operating modes: manual and automatic. 
       * The following algorithm is currently used in automatic mode: the controller sets a search direction for the reference point: positive direction if the current calibration reading is low, negative otherwie. It uses a velocity controller to move the joint in this direction and moves the joint until the calibration reading changes value. It then sets the offset filed in the related transmission accordingly.
       * In manual mode, the joint velocity controller is used to find the min and max limits by exploring the space at low speed.
       * 
   */
      virtual void beginCalibration();
  
      virtual void endCalibration();
  
      bool getOffset(double & joint_angle);
  
  /** \brief Sets the offset of the joint
   */
      void setOffset(double joint_angle);

  /*!
       * \brief Issues commands to the joint. Should be called at regular intervals
   */
      virtual void update();
  
      bool calibrated() const { assert(joint_state_); return joint_state_->calibrated_; }

  
    protected:
  
      double offset(double act_pos, double joint_ref_pos);
  
      enum ControllerState {Stop,Search,Initialized,Begin,Idle} ; 
  
      mechanism::Joint* joint_; /**< Joint we're controlling. */
      Actuator* actuator_; /** The actuator corresponding to the joint */
      mechanism::Robot *robot_; /**< Pointer to robot structure. */
      mechanism::Transmission * transmission_;  /** The transmission associated to the actuator and the joint. */
      mechanism::JointState* joint_state_; /**< Joint we're controlling. */
  
      int state_; /** The current state of the controller*/
  
      double min_;  // in actuator position
      double max_;  // in actuator position
  
      ros::thread::mutex state_mutex_; /** Mutex locked during the calibration procedure to prevent lousy code from trying to find the offset while it is already searching. */
  };


  /***************************************************/
/*! \class controller::JointCalibrationControllerNode
  \brief Joint Limit Controller ROS Node
    
  This class starts and stops the initialization sequence

 */
  /***************************************************/

  class JointManualCalibrationControllerNode : public Controller
  {
    public:
  /*!
     * \brief Default Constructor
     *
   */
      JointManualCalibrationControllerNode();

  /*!
       * \brief Destructor
   */
      virtual ~JointManualCalibrationControllerNode();

      void update();

      bool initXml(mechanism::RobotState *robot, TiXmlElement *config);

  // Services
      bool beginCalibrationCommand(generic_controllers::CalibrateJoint::request &req,
                                   generic_controllers::CalibrateJoint::response &resp);
      bool endCalibrationCommand(generic_controllers::CalibrateJoint::request &req,
                                 generic_controllers::CalibrateJoint::response &resp);

    private:
      JointManualCalibrationController *c_;
  };
}


