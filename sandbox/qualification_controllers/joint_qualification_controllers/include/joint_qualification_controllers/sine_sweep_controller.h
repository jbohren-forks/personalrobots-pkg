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
/*! \class controller::SineSweepController
    \brief Sine Sweep Controller

    This class basically applies a sine sweep to the joint.
*/
/***************************************************/


#include <ros/ros.h>
#include <math.h>
#include <joint_qualification_controllers/TestData.h>
#include <realtime_tools/realtime_srv_call.h>
#include <pr2_controller_interface/controller.h>
#include <control_toolbox/sine_sweep.h>
#include <boost/scoped_ptr.hpp>


namespace controller
{

class SineSweepController : public Controller
{
public:
  /*!
   * \brief Default Constructor of the SineSweepController class.
   *
   */
  SineSweepController();

  /*!
   * \brief Destructor of the SineSweepController class.
   */
  ~SineSweepController();

  /*!
   * \brief Functional way to initialize.
   * \param *robot The robot that is being controlled.
   * \param &n NodeHandle of mechanism control
   */
  bool init(mechanism::RobotState *robot, const ros::NodeHandle &n);

  bool starting();

  void analysis();
  void update();

  bool sendData();
  
  bool done() { return done_ == 1; }

  joint_qualification_controllers::TestData::Request test_data_;

private:
  mechanism::JointState *joint_state_;      /**< Joint we're controlling. */
  mechanism::RobotState *robot_;            /**< Pointer to robot structure. */
  control_toolbox::SineSweep *sweep_;       /**< Sine sweep. */
  double duration_;                         /**< Duration of the sweep. */
  ros::Time initial_time_;                     /**< Start time of the sweep. */
  int count_;
  bool done_;
  
  bool data_sent_;
  
  boost::scoped_ptr<realtime_tools::RealtimeSrvCall<joint_qualification_controllers::TestData::Request, joint_qualification_controllers::TestData::Response>  >call_service_;

};
}


