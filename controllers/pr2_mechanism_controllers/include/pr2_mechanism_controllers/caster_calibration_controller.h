/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Stuart Glaser
 */

#ifndef CASTER_CALIBRATION_CONTROLLER_H
#define CASTER_CALIBRATION_CONTROLLER_H

#include "pr2_mechanism_controllers/caster_controller.h"
#include <robot_mechanism_controllers/CalibrateJoint.h>

namespace controller {

class CasterCalibrationController : public Controller
{
public:
  CasterCalibrationController();
  ~CasterCalibrationController();

  virtual bool initXml(mechanism::RobotState *robot, TiXmlElement *config);


  /*!
   * \brief Issues commands to the joint. Should be called at regular intervals
   */
  virtual void update();

  bool calibrated() { return state_ == STOPPED; }
  void beginCalibration()
  {
    if (state_ == INITIALIZED)
      state_ = BEGINNING;
  }

protected:

  enum { INITIALIZED, BEGINNING, MOVING, STOPPED };
  int state_;

  double search_velocity_;
  bool original_switch_state_;

  Actuator *actuator_;
  mechanism::JointState *joint_;
  mechanism::Transmission *transmission_;

  controller::CasterController cc_;
};


class CasterCalibrationControllerNode : public Controller
{
public:
  CasterCalibrationControllerNode();
  ~CasterCalibrationControllerNode();

  void update();

  bool initXml(mechanism::RobotState *robot, TiXmlElement *config);

  bool calibrateCommand(robot_mechanism_controllers::CalibrateJoint::request &req,
                        robot_mechanism_controllers::CalibrateJoint::response &resp);

private:
  CasterCalibrationController c_;
  AdvertisedServiceGuard guard_calibrate_;
};

}

#endif
