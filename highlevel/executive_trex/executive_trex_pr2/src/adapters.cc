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


#include "ros_action_adapter.h"
#include "ROSStateAdapter.hh"
#include <std_msgs/Empty.h>
#include <deprecated_msgs/RobotBase2DOdom.h>
#include <robot_msgs/Door.h>
#include <robot_msgs/PlugStow.h>
#include <robot_actions/ShellCommandState.h>
#include <robot_actions/DoorActionState.h>
#include <robot_actions/MoveBaseState.h>
#include <robot_actions/Pose2D.h>
#include <robot_actions/RechargeState.h>
#include <robot_actions/DetectPlugOnBaseActionState.h>
#include <robot_actions/SwitchControllers.h>
#include <robot_actions/SwitchControllersState.h>

namespace TREX {

  /***********************************************************************
   * @brief Door actions operate with a door message for goal and feedback
   ***********************************************************************/
  class DoorActionAdapter: public ROSActionAdapter<robot_msgs::Door, robot_actions::DoorActionState, robot_msgs::Door> {
  public:

    DoorActionAdapter(const LabelStr& agentName, const TiXmlElement& configData)
      : ROSActionAdapter<robot_msgs::Door, robot_actions::DoorActionState, robot_msgs::Door>(agentName, configData){
    }

    virtual ~DoorActionAdapter(){}

  protected:

    /**
     * @brief bind a door message based on the token
     *
     * // Points for the door frame
     * float frame_p1_x;
     * float frame_p1_y;
     * float frame_p1_z;
     * float frame_p2_x;
     * float frame_p2_y;
     * float frame_p2_z;
     *
     * // Handle data
     * float height;
     * HINGE_FRAME hinge;
     * ROTATION_DIRECTION rot_dir;
     *
     * // Points for the door
     * float door_p1_x;
     * float door_p1_y;
     * float door_p1_z;
     * float door_p2_x;
     * float door_p2_y;
     * float door_p2_z;
     *
     * // Handle Data
     * float handle_x
     * float handle_y
     * float handle_z
     */
    void write(const TokenId& token, robot_msgs::Door& msg){
      // Set the frame we are in
      msg.header.frame_id = getFrame(token);

      // Frame Data
      ROSAdapter::write<float>("frame_p1_x", token, msg.frame_p1.x);
      ROSAdapter::write<float>("frame_p1_y", token, msg.frame_p1.y);
      ROSAdapter::write<float>("frame_p1_z", token, msg.frame_p1.z);
      ROSAdapter::write<float>("frame_p2_x", token, msg.frame_p2.x);
      ROSAdapter::write<float>("frame_p2_y", token, msg.frame_p2.y);
      ROSAdapter::write<float>("frame_p2_z", token, msg.frame_p2.z);
      ROSAdapter::write<float>("height", token, msg.height);
      ROSAdapter::write<int32_t>("hinge", token, msg.hinge);
      ROSAdapter::write<int32_t>("rot_dir", token, msg.rot_dir);

      // Door Data
      ROSAdapter::write<float>("door_p1_x", token, msg.door_p1.x);
      ROSAdapter::write<float>("door_p1_y", token, msg.door_p1.y);
      ROSAdapter::write<float>("door_p1_z", token, msg.door_p1.z);
      ROSAdapter::write<float>("door_p2_x", token, msg.door_p2.x);
      ROSAdapter::write<float>("door_p2_y", token, msg.door_p2.y);
      ROSAdapter::write<float>("door_p2_z", token, msg.door_p2.z);

      // Handle Data
      ROSAdapter::write<float>("handle_x", token, msg.handle.x);
      ROSAdapter::write<float>("handle_y", token, msg.handle.y);
      ROSAdapter::write<float>("handle_z", token, msg.handle.z);
    }
    // Read Observation from Door Message
    void read(ObservationByValue& obs, const robot_msgs::Door& msg){
      setFrame(msg.header.frame_id, &obs);

      // Frame Data
      ROSAdapter::read<float>("frame_p1_x", obs, msg.frame_p1.x);
      ROSAdapter::read<float>("frame_p1_y", obs, msg.frame_p1.y);
      ROSAdapter::read<float>("frame_p1_z", obs, msg.frame_p1.z);
      ROSAdapter::read<float>("frame_p2_x", obs, msg.frame_p2.x);
      ROSAdapter::read<float>("frame_p2_y", obs, msg.frame_p2.y);
      ROSAdapter::read<float>("frame_p2_z", obs, msg.frame_p2.z);
      ROSAdapter::read<float>("height", obs, msg.height);
      ROSAdapter::read<int32_t>("hinge", obs, msg.hinge);
      ROSAdapter::read<int32_t>("rot_dir", obs, msg.rot_dir);

      // Door Data
      ROSAdapter::read<float>("door_p1_x", obs, msg.door_p1.x);
      ROSAdapter::read<float>("door_p1_y", obs, msg.door_p1.y);
      ROSAdapter::read<float>("door_p1_z", obs, msg.door_p1.z);
      ROSAdapter::read<float>("door_p2_x", obs, msg.door_p2.x);
      ROSAdapter::read<float>("door_p2_y", obs, msg.door_p2.y);
      ROSAdapter::read<float>("door_p2_z", obs, msg.door_p2.z);

      // Handle Data
      ROSAdapter::read<float>("handle_x", obs, msg.handle.x);
      ROSAdapter::read<float>("handle_y", obs, msg.handle.y);
      ROSAdapter::read<float>("handle_z", obs, msg.handle.z);
    }
    
    virtual void fillActiveObservationParameters(const robot_msgs::Door& msg, ObservationByValue* obs){
      read(*obs, msg);
    }

    virtual void fillInactiveObservationParameters(const robot_msgs::Door& msg, ObservationByValue* obs){ 
      read(*obs, msg);
    }

    void fillDispatchParameters(robot_msgs::Door& msg, const TokenId& goalToken){
      write(goalToken, msg);
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

    virtual ~MoveBaseAdapter(){}

  protected:

    virtual void fillActiveObservationParameters(const robot_actions::Pose2D& msg, ObservationByValue* obs){
      setFrame(msg.header.frame_id, obs);
      readPose(*obs, msg.x, msg.y, msg.th);
    }

    virtual void fillInactiveObservationParameters(const robot_actions::Pose2D& msg, ObservationByValue* obs){ 
      setFrame(msg.header.frame_id, obs);
      readPose(*obs, msg.x, msg.y, msg.th);
    }

    void fillDispatchParameters(robot_actions::Pose2D& msg, const TokenId& goalToken){
      msg.header.frame_id = getFrame(goalToken);
      writePose(goalToken, msg.x, msg.y, msg.th);
    }
  };

  // Allocate Factory
  TeleoReactor::ConcreteFactory<MoveBaseAdapter> MoveBaseAdapter_Factory("NewMoveBaseAdapter");


  /***********************************************************************
   * @@brief Recharge action with no goal or feedback
   **********************************************************************/
  class RechargeAdapter: public ROSActionAdapter<std_msgs::Float32, robot_actions::RechargeState, std_msgs::Float32> {
  public:

    RechargeAdapter(const LabelStr& agentName, const TiXmlElement& configData)
      : ROSActionAdapter<std_msgs::Float32, robot_actions::RechargeState,  std_msgs::Float32>(agentName, configData){}

    virtual ~RechargeAdapter(){}

  protected:
    
    virtual void fillActiveObservationParameters(const std_msgs::Float32& msg, ObservationByValue* obs){
      ROSAdapter::read<float>("recharge_level", *obs, msg.data);
    }

    virtual void fillInactiveObservationParameters(const std_msgs::Float32& msg, ObservationByValue* obs){
      ROSAdapter::read<float>("recharge_level", *obs, msg.data);
    }

    virtual void fillDispatchParameters(std_msgs::Float32& msg, const TokenId& goalToken){
      ROSAdapter::write<float>("recharge_level", goalToken, msg.data);
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

    virtual ~ShellCommandAdapter(){}

  protected:
    
    virtual void fillActiveObservationParameters(const std_msgs::String& msg, ObservationByValue* obs){
      obs->push_back("request", toStringDomain(msg));
    }

    virtual void fillInactiveObservationParameters(const std_msgs::String& msg, ObservationByValue* obs){ 
      obs->push_back("response", toStringDomain(msg));
    }

    void fillDispatchParameters(std_msgs::String& msg, const TokenId& goalToken){
      const StringDomain& dom = static_cast<const StringDomain&>(goalToken->getVariable("request")->lastDomain());
      write(dom, msg);
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

    virtual ~BaseStateAdapter(){}

  private:
    void fillObservationParameters(ObservationByValue* obs){
      setFrame(stateMsg.header.frame_id, obs);
      readPose(*obs, stateMsg.pos.x, stateMsg.pos.y, stateMsg.pos.th);
    }
  };

  // Allocate a Factory
  TeleoReactor::ConcreteFactory<BaseStateAdapter> l_NewBaseStateAdapter_Factory("NewBaseStateAdapter");

  /***********************************************************************
   * @brief DetectPlugOnBase actions with a pose message for goal and feedback
   **********************************************************************/
  class DetectPlugOnBaseAdapter: public ROSActionAdapter<std_msgs::Empty, robot_actions::DetectPlugOnBaseActionState, robot_msgs::PlugStow> {
  public:

    DetectPlugOnBaseAdapter(const LabelStr& agentName, const TiXmlElement& configData)
      : ROSActionAdapter<std_msgs::Empty, robot_actions::DetectPlugOnBaseActionState, robot_msgs::PlugStow>(agentName, configData){
    }

    virtual ~DetectPlugOnBaseAdapter(){}

  protected:

    virtual void fillActiveObservationParameters(const std_msgs::Empty& msg, ObservationByValue* obs){}

    virtual void fillInactiveObservationParameters(const robot_msgs::PlugStow& msg, ObservationByValue* obs){
      setFrame(msg.header.frame_id, obs);
      ROSAdapter::read<double>("x", *obs, msg.plug_centroid.x);
      ROSAdapter::read<double>("y", *obs, msg.plug_centroid.y);
      ROSAdapter::read<double>("z", *obs, msg.plug_centroid.z);
    }

    void fillDispatchParameters(std_msgs::Empty& msg, const TokenId& goalToken){}
  };

  // Allocate Factory
  TeleoReactor::ConcreteFactory<DetectPlugOnBaseAdapter> DetectPlugOnBaseAdapter_Factory("DetectPlugOnBaseAdapter");


  /***********************************************************************
   * @brief SwtichControllers action
   **********************************************************************/
  class SwitchControllersAdapter: public ROSActionAdapter<robot_actions::SwitchControllers, robot_actions::SwitchControllersState, std_msgs::Empty> {
  public:

    SwitchControllersAdapter(const LabelStr& agentName, const TiXmlElement& configData)
      : ROSActionAdapter<robot_actions::SwitchControllers, robot_actions::SwitchControllersState, std_msgs::Empty>(agentName, configData){
    }

    virtual ~SwitchControllersAdapter(){}

  protected:

    virtual void fillActiveObservationParameters(const robot_actions::SwitchControllers& msg, ObservationByValue* obs){}

    virtual void fillInactiveObservationParameters(const std_msgs::Empty& msg, ObservationByValue* obs){}

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

    void handleMasterToken(const TokenId& master_token, robot_actions::SwitchControllers& msg){
      if(master_token.isId() && master_token->getPlanDatabase()->getSchema()->isA(master_token->getPredicateName(), "MechanismControllerState.Holds")){
	ConstrainedVariableId param = master_token->getVariable("is_up");
	checkError(param.isValid(), "Trying to dispatch controller switch but could find no variable named 'is_up' in token " << master_token->toString() << 
		   ". This indicates that the model is out of synch with the adapter code.");
	ROS_ASSERT(param.isId());
	checkError(param->lastDomain().isSingleton(), "Cannot tell if controller is up or down:" << param->toString() << ". Model should ensure value is bound.");

	bool is_up = (bool) param->lastDomain().getSingletonValue();

	// The controller name is a parameter of the object on which the master is placed
	checkError(master_token->getObject()->lastDomain().isSingleton(), master_token->getObject()->lastDomain().toString());
	ObjectId master_object = (ObjectId) master_token->getObject()->lastDomain().getSingletonValue();
	checkError(master_token->getObject()->lastDomain().isSingleton(), master_token->getObject()->lastDomain().toString());

	std::string master_name = Observation::getTimelineName(master_token).toString();
	std::string controller_param_name = master_name + ".controller";
	ConstrainedVariableId controller_sv_id = master_object->getVariable(controller_param_name); // This is ugly
	checkError(controller_sv_id.isValid(), "No variable named '" << controller_param_name << "' in " << master_object->toLongString());
	checkError(controller_sv_id->lastDomain().isSingleton(), controller_sv_id->lastDomain().toString());
	ObjectId controller_id = (ObjectId) controller_sv_id->lastDomain().getSingletonValue();
	std::string controller_name = controller_id->getName().toString();

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
}
