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
/*! \class controller::HoldSetController
    \brief Hold Set Controller

    This holds a joint in a set of locations, while
    dithering the joint and recording position and cmd.

*/
/***************************************************/


#include <vector>
#include <ros/ros.h>
#include <math.h>
#include <joint_qualification_controllers/HoldSetData.h>
#include <realtime_tools/realtime_srv_call.h>
#include <controller_interface/controller.h>
#include <robot_mechanism_controllers/joint_position_controller.h>
#include <control_toolbox/dither.h>

#include <iostream>
#include <string>
#include <sstream>

namespace controller
{

class HoldSetController : public Controller
{

public:
  enum { STARTING, SETTLING, DITHERING, PAUSING, DONE };

  HoldSetController();
  ~HoldSetController();

  /*!
   * \brief Functional way to initialize.
   * \param *robot The robot that is being controlled.
   * \param &n Node handle for parameters and services
   */
  bool init( mechanism::RobotState *robot, const ros::NodeHandle &n);


  /*!
   * \brief Issues commands to the joint. Should be called at regular intervals
   */
  void update();

  bool starting();

  bool sendData();
    
  bool done() { return state_ == DONE; }
  
  joint_qualification_controllers::HoldSetData::Request hold_set_data_;

private:
  control_toolbox::Dither* lift_dither_;
  control_toolbox::Dither* flex_dither_;

  controller::JointPositionController* lift_controller_;
  controller::JointPositionController* flex_controller_;

  mechanism::JointState* flex_state_;    
  mechanism::JointState* lift_state_;    

  mechanism::RobotState *robot_;            /**< Pointer to robot structure. */

  int starting_count_;

  int state_;

  double lift_cmd_, flex_cmd_;


  double initial_time_;

  double settle_time_;
  double start_time_;
  double dither_time_;
  double timeout_;
  double lift_min_, lift_max_, lift_delta_;
  double flex_min_, flex_max_, flex_delta_;

  int dither_count_;

  uint lift_index_;
  uint flex_index_;

  bool data_sent_;

  boost::scoped_ptr<realtime_tools::RealtimeSrvCall<joint_qualification_controllers::HoldSetData::Request, joint_qualification_controllers::HoldSetData::Response> > call_service_;


};

}


/***************************************************/
/*! \class controller::HoldSetController
    \brief Hold Set Controller

    This holds a joint at a set of positions, measuring effort

<controller type="HoldSetControllerNode" name="cb_hold_set_controller">

      <controller name="shoulder_lift_controller" type="JointPositionController"><br>
      <dither dither_amp="0.25" />
        <joint name="r_shoulder_lift_joint" ><br>
          <pid p="1.5" d="0.1" i="0.3" iClamp="0.2" /><br>
        </joint><br>
      </controller><br>

      <controller name="elbow_flex_controller" type="JointPositionController"><br>
      <dither dither_amp="0.25" />
        <joint name="r_elbow_flex_joint" ><br>
          <pid p="0.8" d="0.05" i="0.1" iClamp="0.1" /><br>
        </joint><br>
      </controller><br>

  <controller_defaults settle_time="2.0" dither_time="1.0" timeout="60" />

  <hold_pt>
    <joint position="1.35" />
    <joint position="0.0" />
  </hold_pt>

  <hold_pt>
    <joint position="1.0" />
    <joint position="0.0" />
  </hold_pt>

  <hold_pt>
    <joint position="0.75" />
    <joint position="0.0" />
  </hold_pt>

  <hold_pt>
    <joint position="0.5" />
    <joint position="0.0" />
  </hold_pt>  

  <hold_pt>
    <joint position="0.25" />
    <joint position="0.0" />
  </hold_pt>

  <hold_pt>
    <joint position="0.0" />
    <joint position="0.0" />
  </hold_pt>

  <hold_pt>
    <joint position="-0.25" />
    <joint position="0.0" />
  </hold_pt>


</controller>

*/
/***************************************************/


