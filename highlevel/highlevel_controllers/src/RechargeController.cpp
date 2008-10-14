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

/**
 * @mainpage
 *
 * @htmlinclude manifest.html
 *
 * @b recharge is a node which will handle plugging in and unpluggin the robot, assuming the plug location
 * is within the immediate vicinity.
 *
 * <hr>
 *
 *  @section usage Usage
 *  @verbatim
 *  $ recharge
 *  @endverbatim
 *
 * <hr>
 *
 * @section topic ROS topics
 *
 * Subscribes to (name/type):
 * - @b 
 *
 * Publishes to (name / type):
 * - @b 
 *
 *  <hr>
 *
 * @section parameters ROS parameters
 *
 * - None
 **/

#include <HighlevelController.hh>
#include <highlevel_controllers/RechargeGoal.h>
#include <highlevel_controllers/RechargeState.h>

namespace highlevel_controllers {

  class RechargeController : public HighlevelController<RechargeState, RechargeGoal> {

  public:

    /**
     * @brief Constructor
     */
    RechargeController(const std::string& stateTopic, const std::string& goalTopic);

    virtual ~RechargeController();

  private:
    virtual void updateStateMsg();
    virtual void updateGoalMsg();
    virtual bool makePlan();
    virtual bool goalReached();
    virtual bool dispatchCommands();
  };

  RechargeController::RechargeController(const std::string& stateTopic, const std::string& goalTopic)
    : HighlevelController<RechargeState, RechargeGoal>("recharge_controller", stateTopic, goalTopic){
    initialize();

    lock();
    stateMsg.plugIn = false;
    stateMsg.desPlugIn = false;
    unlock();
  }

  RechargeController::~RechargeController(){}

  void RechargeController::updateStateMsg(){
    lock();
    // HACK
    if(goalReached())
      stateMsg.plugIn = goalMsg.plugIn;

    unlock();
  }

  void RechargeController::updateGoalMsg(){
    lock();
    stateMsg.desPlugIn = goalMsg.plugIn;
    stateMsg.goal = goalMsg.goal;
    unlock();
  }

  bool RechargeController::makePlan(){
    return true;
  }

  /**
   * @brief
   */
  bool RechargeController::goalReached(){
    return true;
  }

  /**
   * @brief 
   */
  bool RechargeController::dispatchCommands(){
    return true;
  }

}

int
main(int argc, char** argv)
{

  if(argc != 2){
    ROS_INFO("Invalid arg count of %d. Usage: ./recharge_controller", argc);
    return -1;
  }

  ros::init(argc,argv);
  try{
    highlevel_controllers::RechargeController node("recharge_state", "recharge_goal");
    node.run();
  }
  catch(...){
    ROS_DEBUG("Caught expection running node. Cleaning up.\n");
  }

  ros::fini();

  return(0);
}
