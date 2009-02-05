/*********************************************************************
*
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
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#ifndef ESCAPE_CONTROLLER_ESCAPE_CONTROLLER_H_
#define ESCAPE_CONTROLLER_ESCAPE_CONTROLLER_H_

#include <string>
#include <highlevel_controllers/highlevel_controller.hh>
#include <escape_controllers/EscapeState.h>
#include <escape_controllers/EscapeGoal.h>

namespace escape_controller {
  class EscapeController : public HighlevelController<EscapeState, EscapeGoal> {
    public:
      EscapeController(const std::string& state_topic, const std::string& goal_topic);

      virtual ~EscapeController();

    private:
      //HighlevelController  Interface
      virtual void updateGoalMsg();
      virtual void updateStateMsg();
      virtual bool makePlan();
      virtual bool goalReached();
      virtual bool dispatchCommands();

      void baseStateCallback();

      pr2_msgs::BaseControllerState base_state_msg_;
  };

  EscapeController::EscapeController(const string& state_topic, const string& goal_topic)
    : HighlevelController<EscapeState, EscapeGoal>("escape_controller", state_topic, goal_topic)
  {
    subscribe("/base_controller/state", base_state_msg_, &EscapeController::baseStateCallback, 1);
  }
};
#endif
