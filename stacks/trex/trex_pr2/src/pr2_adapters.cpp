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


#include <trex_ros/adapter_utilities.h>
#include <trex_pr2/pr2_adapter_utilities.h>
#include <trex_pr2/pr2_adapters.h>
#include "trex_ros/ros_action_adapter.h"
#include "trex_ros/ros_state_adapter.h"

using namespace trex_ros;
using namespace trex_pr2;
namespace TREX {

  /***********************************************************************
   * @brief Door actions operate with a door message for goal and feedback
   ***********************************************************************/
  class DoorActionAdapter: public ROSActionAdapter<door_msgs::Door, pr2_robot_actions::DoorActionState, door_msgs::Door> {
  public:

    DoorActionAdapter(const LabelStr& agentName, const TiXmlElement& configData)
      : ROSActionAdapter<door_msgs::Door, pr2_robot_actions::DoorActionState, door_msgs::Door>(agentName, configData){
    }
    
    virtual void fillActiveObservationParameters(const door_msgs::Door& msg, ObservationByValue* obs){
      Pr2AdapterUtilities::read(*obs, msg);
    }

    virtual void fillInactiveObservationParameters(const door_msgs::Door& msg, ObservationByValue* obs){ 
      Pr2AdapterUtilities::read(*obs, msg);
    }

    void fillDispatchParameters(door_msgs::Door& msg, const TokenId& goalToken){
      Pr2AdapterUtilities::write(goalToken, msg);
    }
  }; 

  // Allocate Factory
  TeleoReactor::ConcreteFactory<DoorActionAdapter>  DoorActionAdapter_Factory("DoorActionAdapter");

  /***********************************************************************
   * @brief MoveBase actions with a pose message for goal and feedback
   **********************************************************************/
  class MoveBaseAdapter: public ROSActionAdapter<geometry_msgs::PoseStamped, nav_robot_actions::MoveBaseState, geometry_msgs::PoseStamped> {
  public:

    MoveBaseAdapter(const LabelStr& agentName, const TiXmlElement& configData)
      : ROSActionAdapter<geometry_msgs::PoseStamped, nav_robot_actions::MoveBaseState,  geometry_msgs::PoseStamped>(agentName, configData){
    }

    virtual void fillDispatchParameters(geometry_msgs::PoseStamped& msg, const TokenId& goalToken){
      AdapterUtilities::write(goalToken, msg);
    }

    virtual void fillActiveObservationParameters(const geometry_msgs::PoseStamped& msg, ObservationByValue* obs){
      AdapterUtilities::read(*obs, msg);
    }

    virtual void fillInactiveObservationParameters(const geometry_msgs::PoseStamped& msg, ObservationByValue* obs){
      AdapterUtilities::read(*obs, msg);
    }

  };

  // Allocate Factory
  TeleoReactor::ConcreteFactory<MoveBaseAdapter> MoveBaseAdapter_Factory("MoveBaseAdapter");

  /***********************************************************************
   * @brief CheckPath
   **********************************************************************/
  class CheckPathAdapter: public ROSActionAdapter<geometry_msgs::PoseStamped, pr2_robot_actions::CheckPathState, int8_t> {
  public:

    CheckPathAdapter(const LabelStr& agentName, const TiXmlElement& configData)
      : ROSActionAdapter<geometry_msgs::PoseStamped, pr2_robot_actions::CheckPathState, int8_t>(agentName, configData){
    }

    virtual void fillDispatchParameters(geometry_msgs::PoseStamped& msg, const TokenId& goalToken){
      AdapterUtilities::write(goalToken, msg);
    }

    virtual void fillActiveObservationParameters(const geometry_msgs::PoseStamped& msg, ObservationByValue* obs){
      AdapterUtilities::read(*obs, msg);
    }

    virtual void fillInactiveObservationParameters(const int8_t& msg, ObservationByValue* obs){
      bool bool_value (msg < 1 ? false : true);
      AdapterUtilities::read<bool>("is_clear", *obs, bool_value);
    }

  };

  // Allocate Factory
  TeleoReactor::ConcreteFactory<CheckPathAdapter> CheckPathAdapter_Factory("CheckPathAdapter");


  /***********************************************************************
   * @@brief Recharge action
   **********************************************************************/
  class RechargeAdapter: public ROSActionAdapter<std_msgs::Float32, pr2_robot_actions::RechargeState, std_msgs::Float32> {
  public:

    RechargeAdapter(const LabelStr& agentName, const TiXmlElement& configData)
      : ROSActionAdapter<std_msgs::Float32, pr2_robot_actions::RechargeState,  std_msgs::Float32>(agentName, configData){}
    
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
   * @brief PlugIn
   **********************************************************************/
  class PlugInAdapter: public ROSActionAdapter<std_msgs::Int32, pr2_robot_actions::PlugInState, std_msgs::Empty> {
  public:

    PlugInAdapter(const LabelStr& agentName, const TiXmlElement& configData)
      : ROSActionAdapter<std_msgs::Int32, pr2_robot_actions::PlugInState, std_msgs::Empty>(agentName, configData){
    }

    virtual void fillDispatchParameters(std_msgs::Int32& msg, const TokenId& goalToken){
      AdapterUtilities::write("outlet_id" , goalToken, msg.data);
    }

  };

  // Allocate Factory
  TeleoReactor::ConcreteFactory<PlugInAdapter> PlugInAdapter_Factory("PlugInAdapter");


  /***********************************************************************
   * @brief 
   **********************************************************************/
  class BaseStateAdapter: public ROSStateAdapter<geometry_msgs::PoseStamped> {
  public:
    BaseStateAdapter(const LabelStr& agentName, const TiXmlElement& configData)
      : ROSStateAdapter<geometry_msgs::PoseStamped> ( agentName, configData) {
    }

  private:
    void fillObservationParameters(ObservationByValue* obs){
      AdapterUtilities::setHeader(stateMsg, *obs);
      AdapterUtilities::read(*obs, stateMsg);
    }
  };

  // Allocate a Factory
  TeleoReactor::ConcreteFactory<BaseStateAdapter> l_BaseStateAdapter_Factory("BaseStateAdapter");

  /***********************************************************************
   * @brief DetectPlugOnBase 
   **********************************************************************/
  class DetectPlugOnBaseAdapter: public ROSActionAdapter<std_msgs::Empty, pr2_robot_actions::DetectPlugOnBaseState, plugs_msgs::PlugStow> {
  public:

    DetectPlugOnBaseAdapter(const LabelStr& agentName, const TiXmlElement& configData)
      : ROSActionAdapter<std_msgs::Empty, pr2_robot_actions::DetectPlugOnBaseState, plugs_msgs::PlugStow>(agentName, configData){
    }

    virtual void fillInactiveObservationParameters(const plugs_msgs::PlugStow& msg, ObservationByValue* obs){
      Pr2AdapterUtilities::read(*obs, msg);
    }
  };

  // Allocate Factory
  TeleoReactor::ConcreteFactory<DetectPlugOnBaseAdapter> DetectPlugOnBaseAdapter_Factory("DetectPlugOnBaseAdapter");

  /***********************************************************************
   * @brief MoveAndGraspPlug 
   **********************************************************************/
  class MoveAndGraspPlugAdapter: public ROSActionAdapter<plugs_msgs::PlugStow, pr2_robot_actions::MoveAndGraspPlugState, std_msgs::Empty> {
  public:

    MoveAndGraspPlugAdapter(const LabelStr& agentName, const TiXmlElement& configData)
      : ROSActionAdapter<plugs_msgs::PlugStow, pr2_robot_actions::MoveAndGraspPlugState, std_msgs::Empty>(agentName, configData){
    }

    virtual void fillActiveObservationParameters(const plugs_msgs::PlugStow& msg, ObservationByValue* obs){
      Pr2AdapterUtilities::read(*obs, msg);
    }

    virtual void fillDispatchParameters(plugs_msgs::PlugStow& msg, const TokenId& goalToken){
      Pr2AdapterUtilities::write(goalToken, msg);
    }
  };

  // Allocate Factory
  TeleoReactor::ConcreteFactory<MoveAndGraspPlugAdapter> MoveAndGraspPlugAdapter_Factory("MoveAndGraspPlugAdapter");


  /***********************************************************************
   * @brief StowPlug 
   **********************************************************************/
  class StowPlugAdapter: public ROSActionAdapter<plugs_msgs::PlugStow, pr2_robot_actions::StowPlugState, std_msgs::Empty> {
  public:

    StowPlugAdapter(const LabelStr& agentName, const TiXmlElement& configData)
      : ROSActionAdapter<plugs_msgs::PlugStow, pr2_robot_actions::StowPlugState, std_msgs::Empty>(agentName, configData){
    }

    virtual void fillDispatchParameters(plugs_msgs::PlugStow& msg, const TokenId& goalToken){
      Pr2AdapterUtilities::write(goalToken, msg);
    }

    virtual void fillActiveObservationParameters(const plugs_msgs::PlugStow& msg, ObservationByValue* obs){
      Pr2AdapterUtilities::read(*obs, msg);
    }
  };

  // Allocate Factory
  TeleoReactor::ConcreteFactory<StowPlugAdapter> StowPlugAdapter_Factory("StowPlugAdapter");


  /***********************************************************************
   * @brief SwtichControllers action
   **********************************************************************/
  class SwitchControllersAdapter: public ROSActionAdapter<pr2_robot_actions::SwitchControllers, pr2_robot_actions::SwitchControllersState, std_msgs::Empty> {
  public:

    SwitchControllersAdapter(const LabelStr& agentName, const TiXmlElement& configData)
      : ROSActionAdapter<pr2_robot_actions::SwitchControllers, pr2_robot_actions::SwitchControllersState, std_msgs::Empty>(agentName, configData){
    }

    void fillDispatchParameters(pr2_robot_actions::SwitchControllers& msg, const TokenId& goalToken){
      static const std::string NO_CONTROLLER("no_controller");
      static const LabelStr UP_LIST("up_list");
      static const LabelStr DOWN_LIST("down_list");
      checkError(goalToken->getVariable(UP_LIST).isValid(), goalToken->toLongString());
      checkError(goalToken->getVariable(DOWN_LIST).isValid(), goalToken->toLongString());
      // Get domains
      ObjectDomain up_list_domain(static_cast<const ObjectDomain&>(goalToken->getVariable(UP_LIST)->lastDomain()));
      std::list<ObjectId> down_list = (static_cast<const ObjectDomain&>(goalToken->getVariable(DOWN_LIST)->lastDomain())).makeObjectList();
      // Process down list first.
      for(std::list<ObjectId>::const_iterator it = down_list.begin(); it != down_list.end(); ++it){
	ObjectId object = *it;
	std::string controller_name = object->getName().toString();
	if(controller_name.find(NO_CONTROLLER) == std::string::npos){
	  TREX_INFO("ros:debug:dispatching", "Adding " << controller_name << " to the down list.");
	  msg.stop_controllers.push_back(controller_name);
	}

	up_list_domain.remove(object);
      }
      // Now process up list
      std::list<ObjectId> up_list = up_list_domain.makeObjectList();
      for(std::list<ObjectId>::const_iterator it = up_list.begin(); it != up_list.end(); ++it){
	ObjectId object = *it;
	std::string controller_name = object->getName().toString();
	if(controller_name.find(NO_CONTROLLER) == std::string::npos){
	  TREX_INFO("ros:debug:dispatching", "Adding " << controller_name << " to the up list.");
	  msg.start_controllers.push_back(controller_name);
	}
      }

      // Finally, restrict the up_list paramater
      goalToken->getVariable(UP_LIST)->restrictBaseDomain(up_list_domain);
    }
  };

  // Allocate Factory
  TeleoReactor::ConcreteFactory<SwitchControllersAdapter> SwitchControllersAdapter_Factory("SwitchControllersAdapter");

  /***********************************************************************
   * @brief DetectOutletAdapter
   **********************************************************************/
  class DetectOutletAdapter: public ROSActionAdapter<geometry_msgs::PointStamped, pr2_robot_actions::DetectOutletState, geometry_msgs::PoseStamped> {
  public:

    DetectOutletAdapter(const LabelStr& agentName, const TiXmlElement& configData)
      : ROSActionAdapter<geometry_msgs::PointStamped, pr2_robot_actions::DetectOutletState, geometry_msgs::PoseStamped>(agentName, configData){
    }

    virtual void fillDispatchParameters(geometry_msgs::PointStamped& msg, const TokenId& goalToken){
      AdapterUtilities::write(goalToken, msg);
    }

    virtual void fillActiveObservationParameters(const geometry_msgs::PointStamped& msg, ObservationByValue* obs){
      AdapterUtilities::read(*obs, msg);
    }

    virtual void fillInactiveObservationParameters(const geometry_msgs::PoseStamped& msg, ObservationByValue* obs){
      AdapterUtilities::read(*obs, msg);
    }
  };

  // Allocate Factory
  TeleoReactor::ConcreteFactory<DetectOutletAdapter> DetectOutletAdapter_Factory("DetectOutletAdapter");
}
