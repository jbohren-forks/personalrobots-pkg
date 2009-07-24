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

#ifndef ACTION_TOOLS_CLIENT_GOAL_STATUS_H_
#define ACTION_TOOLS_CLIENT_GOAL_STATUS_H_

#include <string>

#include "action_tools/GoalStatus.h"

namespace action_tools
{

class ClientGoalStatus
{
public:
  enum StateEnum  { PENDING, ACTIVE, PREEMPTED, SUCCEEDED, ABORTED, REJECTED, LOST } ;

  ClientGoalStatus(StateEnum state)
  {
    state_ = state;
  }

  ClientGoalStatus(const GoalStatus& goal_status)
  {
    fromGoalStatus(goal_status);
  }

  inline const StateEnum& operator=(const StateEnum& state)
  {
    state_ = state;
    return state;
  }

  inline bool operator==(const ClientGoalStatus& rhs)
  {
    return state_ == rhs.state_;
  }

  inline bool operator!=(const ClientGoalStatus& rhs)
  {
    return !(state_ == rhs.state_);
  }

  void fromGoalStatus(const GoalStatus& goal_status)
  {
    switch(goal_status.status)
    {
      case GoalStatus::PREEMPTED:
        state_ = ClientGoalStatus::PREEMPTED; break;
      case GoalStatus::SUCCEEDED:
        state_ = ClientGoalStatus::SUCCEEDED; break;
      case GoalStatus::ABORTED:
        state_ = ClientGoalStatus::ABORTED; break;
      case GoalStatus::REJECTED:
        state_ = ClientGoalStatus::REJECTED; break;
      default:
        state_ = ClientGoalStatus::LOST;
        ROS_ERROR("Cannot convert GoalStatus %u to ClientGoalState", goal_status.status); break;
    }
  }

  std::string toString() const
  {
    switch(state_)
    {
      case PENDING:
        return "PENDING";
      case ACTIVE:
        return "ACTIVE";
      case PREEMPTED:
        return "PREEMPTED";
      case SUCCEEDED:
        return "SUCCEEDED";
      case ABORTED:
        return "ABORTED";
      case REJECTED:
        return "REJECTED";
      case LOST:
        return "LOST";
      default:
        ROS_ERROR("BUG: Unhandled ClientGoalStatus");
        break;
    }
    return "BUG-UNKNOWN";
  }

private:
  StateEnum state_;
  ClientGoalStatus(); //!< Need to always specific an initial state. Thus, no empty constructor
};

}

#endif // ACTION_TOOLS_CLIENT_GOAL_STATE_H_
