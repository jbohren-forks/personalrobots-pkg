/*
 * TREX Process
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
 *     * Neither the name of the <ORGANIZATION> nor the names of its
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

/**
 * @file This File contains the declaration and definition of action adapters to bind TREX behavior timeline tokens
 * to ROS message constructs, and to handle appropriate dispatching and observation synchronization
 *
 * @author Conor McGann
 */


#include <executive_trex_pr2/adapter_utilities.h>
#include <executive_trex_pr2/adapters.h>
#include "ros_action_adapter.h"
#include "ROSStateAdapter.hh"

using namespace executive_trex_pr2;

namespace TREX {

  /***********************************************************************
   * @brief Door actions operate with a door message for goal and feedback
   ***********************************************************************/
  class DoorActionAdapter: public ROSActionAdapter<robot_msgs::Door, robot_actions::DoorActionState, robot_msgs::Door> {
  public:

    DoorActionAdapter(const LabelStr& agentName, const TiXmlElement& configData)
      : ROSActionAdapter<robot_msgs::Door, robot_actions::DoorActionState, robot_msgs::Door>(agentName, configData){
    }
    
    virtual void fillActiveObservationParameters(const robot_msgs::Door& msg, ObservationByValue* obs){
      AdapterUtilities::read(*obs, msg);
    }

    virtual void fillInactiveObservationParameters(const robot_msgs::Door& msg, ObservationByValue* obs){ 
      AdapterUtilities::read(*obs, msg);
    }

    void fillDispatchParameters(robot_msgs::Door& msg, const TokenId& goalToken){
      AdapterUtilities::write(goalToken, msg);
    }
  }; 

  // Allocate Factory
  TeleoReactor::ConcreteFactory<DoorActionAdapter> DoorActionAdapter_Factory("DoorActionAdapter");

  /***********************************************************************
   * @brief MoveBase actions with a pose message for goal and feedback
   **********************************************************************/
  class MoveBaseAdapter: public ROSActionAdapter<robot_actions::Pose2D, robot_actions::MoveBaseState, robot_actions::Pose2D> {
  public:

    MoveBaseAdapter(const LabelStr& agentName, const TiXmlElement& configData)
      : ROSActionAdapter<robot_actions::Pose2D, robot_actions::MoveBaseState,  robot_actions::Pose2D>(agentName, configData){
    }

    virtual void fillActiveObservationParameters(const robot_actions::Pose2D& msg, ObservationByValue* obs){
      AdapterUtilities::setFrame(msg.header.frame_id, *obs);
      AdapterUtilities::readPose(*obs, msg.x, msg.y, msg.th);
    }

    virtual void fillInactiveObservationParameters(const robot_actions::Pose2D& msg, ObservationByValue* obs){ 
      AdapterUtilities::setFrame(msg.header.frame_id, *obs);
      AdapterUtilities::readPose(*obs, msg.x, msg.y, msg.th);
    }

    void fillDispatchParameters(robot_actions::Pose2D& msg, const TokenId& goalToken){
      msg.header.frame_id = AdapterUtilities::getFrame(goalToken);
      AdapterUtilities::writePose(goalToken, msg.x, msg.y, msg.th);
    }
  };

  // Allocate Factory
  TeleoReactor::ConcreteFactory<MoveBaseAdapter> MoveBaseAdapter_Factory("NewMoveBaseAdapter");

  /***********************************************************************
   * @brief CheckDoorway actions with a pose message for goal and feedback
   **********************************************************************/
  class CheckDoorwayAdapter: public ROSActionAdapter<robot_actions::Pose2D, robot_actions::CheckDoorwayState, robot_actions::Pose2D> {
  public:

    CheckDoorwayAdapter(const LabelStr& agentName, const TiXmlElement& configData)
      : ROSActionAdapter<robot_actions::Pose2D, robot_actions::CheckDoorwayState,  robot_actions::Pose2D>(agentName, configData){
    }

    void fillDispatchParameters(robot_actions::Pose2D& msg, const TokenId& goalToken){
      msg.header.frame_id = AdapterUtilities::getFrame(goalToken);
      AdapterUtilities::writePose(goalToken, msg.x, msg.y, msg.th);
    }
  };

  // Allocate Factory
  TeleoReactor::ConcreteFactory<CheckDoorwayAdapter> CheckDoorwayAdapter_Factory("CheckDoorwayAdapter");

  /***********************************************************************
   * @brief NotifyDoorBlocked action
   **********************************************************************/
  class NotifyDoorBlockedAdapter: public ROSActionAdapter<robot_actions::Pose2D, robot_actions::NotifyDoorBlockedState, robot_actions::Pose2D> {
  public:

    NotifyDoorBlockedAdapter(const LabelStr& agentName, const TiXmlElement& configData)
      : ROSActionAdapter<robot_actions::Pose2D, robot_actions::NotifyDoorBlockedState,  robot_actions::Pose2D>(agentName, configData){
    }

    /**
     * @todo Should take a door message and an ID?
     */
    void fillDispatchParameters(robot_actions::Pose2D& msg, const TokenId& goalToken){
    }
  };

  // Allocate Factory
  TeleoReactor::ConcreteFactory<NotifyDoorBlockedAdapter> NotifyDoorBlockedAdapter_Factory("NotifyDoorBlockedAdapter");

  /***********************************************************************
   * @brief NoArgumentsAction action
   **********************************************************************/
  class NoArgumentsActionAdapter: public ROSActionAdapter<std_msgs::Empty, robot_actions::NoArgumentsActionState, std_msgs::Empty> {
  public:

    NoArgumentsActionAdapter(const LabelStr& agentName, const TiXmlElement& configData)
      : ROSActionAdapter<std_msgs::Empty, robot_actions::NoArgumentsActionState,  std_msgs::Empty>(agentName, configData){
    }
  };

  // Allocate Factory
  TeleoReactor::ConcreteFactory<NoArgumentsActionAdapter> NoArgumentsActionAdapter_Factory("NoArgumentsActionAdapter");


  /***********************************************************************
   * @@brief Recharge action
   **********************************************************************/
  class RechargeAdapter: public ROSActionAdapter<std_msgs::Float32, robot_actions::RechargeState, std_msgs::Float32> {
  public:

    RechargeAdapter(const LabelStr& agentName, const TiXmlElement& configData)
      : ROSActionAdapter<std_msgs::Float32, robot_actions::RechargeState,  std_msgs::Float32>(agentName, configData){}
    
    virtual void fillActiveObservationParameters(const std_msgs::Float32& msg, ObservationByValue* obs){
      AdapterUtilities::read<float>("recharge_level", *obs, msg.data);
    }

    virtual void fillInactiveObservationParameters(const std_msgs::Float32& msg, ObservationByValue* obs){
      AdapterUtilities::read<float>("recharge_level", *obs, msg.data);
    }

    virtual void fillDispatchParameters(std_msgs::Float32& msg, const TokenId& goalToken){
      AdapterUtilities::write<float>("recharge_level", goalToken, msg.data);
    }
  };

  // Allocate Factory
  TeleoReactor::ConcreteFactory<RechargeAdapter> RechargeAdapter_Factory("RechargeAdapter");


  /***********************************************************************
   * @brief ShellCommand action will take system commands in strings and 
   * ship them for execution.
   **********************************************************************/
  class ShellCommandAdapter: public ROSActionAdapter<std_msgs::String, robot_actions::ShellCommandState, std_msgs::String> {
  public:

    ShellCommandAdapter(const LabelStr& agentName, const TiXmlElement& configData)
      : ROSActionAdapter<std_msgs::String, robot_actions::ShellCommandState, std_msgs::String>(agentName, configData){
    }
    
    virtual void fillActiveObservationParameters(const std_msgs::String& msg, ObservationByValue* obs){
      obs->push_back("request", AdapterUtilities::toStringDomain(msg));
    }

    virtual void fillInactiveObservationParameters(const std_msgs::String& msg, ObservationByValue* obs){ 
      obs->push_back("response", AdapterUtilities::toStringDomain(msg));
    }

    void fillDispatchParameters(std_msgs::String& msg, const TokenId& goalToken){
      const StringDomain& dom = static_cast<const StringDomain&>(goalToken->getVariable("request")->lastDomain());
      AdapterUtilities::write(dom, msg);
    }
  };  

  // Allocate Factory
  TeleoReactor::ConcreteFactory<ShellCommandAdapter> ShellCommandAdapter_Factory("ShellCommandAdapter");


  /***********************************************************************
   * @brief ShellCommand action will take system commands in strings and 
   * ship them for execution.
   **********************************************************************/
  class BaseStateAdapter: public ROSStateAdapter<deprecated_msgs::RobotBase2DOdom> {
  public:
    BaseStateAdapter(const LabelStr& agentName, const TiXmlElement& configData)
      : ROSStateAdapter<deprecated_msgs::RobotBase2DOdom> ( agentName, configData) {
    }

  private:
    void fillObservationParameters(ObservationByValue* obs){
      AdapterUtilities::setFrame(stateMsg.header.frame_id, *obs);
      AdapterUtilities::readPose(*obs, stateMsg.pos.x, stateMsg.pos.y, stateMsg.pos.th);
    }
  };

  // Allocate a Factory
  TeleoReactor::ConcreteFactory<BaseStateAdapter> l_NewBaseStateAdapter_Factory("NewBaseStateAdapter");

  /***********************************************************************
   * @brief DetectPlugOnBase 
   **********************************************************************/
  class DetectPlugOnBaseAdapter: public ROSActionAdapter<std_msgs::Empty, robot_actions::DetectPlugOnBaseActionState, robot_msgs::PlugStow> {
  public:

    DetectPlugOnBaseAdapter(const LabelStr& agentName, const TiXmlElement& configData)
      : ROSActionAdapter<std_msgs::Empty, robot_actions::DetectPlugOnBaseActionState, robot_msgs::PlugStow>(agentName, configData){
    }

    virtual void fillInactiveObservationParameters(const robot_msgs::PlugStow& msg, ObservationByValue* obs){
      AdapterUtilities::read(*obs, msg);
    }
  };

  // Allocate Factory
  TeleoReactor::ConcreteFactory<DetectPlugOnBaseAdapter> DetectPlugOnBaseAdapter_Factory("DetectPlugOnBaseAdapter");

  /***********************************************************************
   * @brief MoveAndGraspPlug 
   **********************************************************************/
  class MoveAndGraspPlugAdapter: public ROSActionAdapter<robot_msgs::PlugStow, robot_actions::MoveAndGraspPlugState, std_msgs::Empty> {
  public:

    MoveAndGraspPlugAdapter(const LabelStr& agentName, const TiXmlElement& configData)
      : ROSActionAdapter<robot_msgs::PlugStow, robot_actions::MoveAndGraspPlugState, std_msgs::Empty>(agentName, configData){
    }

    void fillDispatchParameters(robot_msgs::PlugStow& msg, const TokenId& goalToken){
      AdapterUtilities::write(goalToken, msg);
    }
  };

  // Allocate Factory
  TeleoReactor::ConcreteFactory<MoveAndGraspPlugAdapter> MoveAndGraspPlugAdapter_Factory("MoveAndGraspPlugAdapter");

  /***********************************************************************
   * @brief StowPlug 
   **********************************************************************/
  class StowPlugAdapter: public ROSActionAdapter<robot_msgs::PlugStow, robot_actions::StowPlugState, std_msgs::Empty> {
  public:

    StowPlugAdapter(const LabelStr& agentName, const TiXmlElement& configData)
      : ROSActionAdapter<robot_msgs::PlugStow, robot_actions::StowPlugState, std_msgs::Empty>(agentName, configData){
    }

    void fillDispatchParameters(robot_msgs::PlugStow& msg, const TokenId& goalToken){
      AdapterUtilities::write(goalToken, msg);
    }
  };

  // Allocate Factory
  TeleoReactor::ConcreteFactory<StowPlugAdapter> StowPlugAdapter_Factory("StowPlugAdapter");


  /***********************************************************************
   * @brief SwtichControllers action
   **********************************************************************/
  class SwitchControllersAdapter: public ROSActionAdapter<robot_actions::SwitchControllers, robot_actions::SwitchControllersState, std_msgs::Empty> {
  public:

    SwitchControllersAdapter(const LabelStr& agentName, const TiXmlElement& configData)
      : ROSActionAdapter<robot_actions::SwitchControllers, robot_actions::SwitchControllersState, std_msgs::Empty>(agentName, configData){
    }

    void fillDispatchParameters(robot_actions::SwitchControllers& msg, const TokenId& goalToken){
      // The token will have a set of merged tokens on it. These merged tokens all are derived from a master of type
      // 'MechanismController' which will be in a state of up or down. If it is up, we will append to the stop list, and if down, we
      // will append to the start list

      // Apply for immediate master token
      handleMasterToken(goalToken->master(), msg);

      // Apply over merged tokens
      const TokenSet& merged_tokens = goalToken->getMergedTokens();
      for(TokenSet::const_iterator it = merged_tokens.begin(); it != merged_tokens.end(); ++it){
	TokenId merged_token = *it;
	handleMasterToken(merged_token->master(), msg);
      }
    }

    /**
     * The Master Token is an instance of a MechanismController. It has a paramater indicating if it is up or down. The timeline name
     * should correspond to the actual controller name.
     */
    void handleMasterToken(const TokenId& master_token, robot_actions::SwitchControllers& msg){
      if(master_token.isId() && master_token->getPlanDatabase()->getSchema()->isA(master_token->getPredicateName(), "MechanismController.Holds")){
	ConstrainedVariableId param = master_token->getVariable("is_up");
	checkError(param.isValid(), "Trying to dispatch controller switch but could find no variable named 'is_up' in token " << master_token->toString() << 
		   ". This indicates that the model is out of synch with the adapter code.");
	ROS_ASSERT(param.isId());
	checkError(param->lastDomain().isSingleton(), "Cannot tell if controller is up or down:" << param->toString() << ". Model should ensure value is bound.");

	bool is_up = (bool) param->lastDomain().getSingletonValue();
	std::string controller_name = Observation::getTimelineName(master_token).toString();

	if(is_up){
	  TREX_INFO("ros:debug:dispatching", "Adding " << controller_name << " to the stop list. Current state is:" << param->lastDomain().toString());
	  msg.stop_controllers.push_back(controller_name);
	}
	else {
	  TREX_INFO("ros:debug:dispatching", "Adding " << controller_name << " to the start list. Current state is:" << param->lastDomain().toString());
	  msg.start_controllers.push_back(controller_name);
	}
      }
    }
  };

  // Allocate Factory
  TeleoReactor::ConcreteFactory<SwitchControllersAdapter> SwitchControllersAdapter_Factory("SwitchControllersAdapter");

  /***********************************************************************
   * @brief ServoToOutlet 
   **********************************************************************/
  class ServoToOutletAdapter: public ROSActionAdapter<robot_actions::ServoToOutlet, robot_actions::ServoToOutletState, std_msgs::Empty> {
  public:

    ServoToOutletAdapter(const LabelStr& agentName, const TiXmlElement& configData)
      : ROSActionAdapter<robot_actions::ServoToOutlet, robot_actions::ServoToOutletState, std_msgs::Empty>(agentName, configData){
    }

    void fillDispatchParameters(robot_msgs::PlugStow& msg, const TokenId& goalToken){
      AdapterUtilities::write(goalToken, msg);
    }
  };

  // Allocate Factory
  TeleoReactor::ConcreteFactory<ServoToOutletAdapter> ServoToOutletAdapter_Factory("ServoToOutletAdapter");

  /***********************************************************************
   * @brief DetectOutletAdapter
   **********************************************************************/
  class DetectOutletAdapter: public ROSActionAdapter<robot_msgs::PointStamped, robot_actions::DetectOutletState, robot_msgs::PoseStamped> {
  public:

    DetectOutletAdapter(const LabelStr& agentName, const TiXmlElement& configData)
      : ROSActionAdapter<robot_msgs::PointStamped, robot_actions::DetectOutletState, robot_msgs::PoseStamped>(agentName, configData){
    }

    void fillDispatchParameters(robot_msgs::PointStamped& msg, const TokenId& goalToken){
      AdapterUtilities::write(goalToken, msg);
    }
    virtual void fillActiveObservationParameters(const robot_msgs::PointStamped& msg, ObservationByValue* obs){
      AdapterUtilities::read(*obs, msg);
    }
    virtual void fillInactiveObservationParameters(const robot_msgs::PoseStamped& msg, ObservationByValue* obs){
      AdapterUtilities::read(*obs, msg);
    }
  };

  // Allocate Factory
  TeleoReactor::ConcreteFactory<DetectOutletAdapter> DetectOutletAdapter_Factory("DetectOutletAdapter");
}
