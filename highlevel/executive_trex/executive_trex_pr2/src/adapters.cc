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
#include "StringDomain.hh"
#include "Token.hh"
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <robot_actions/ShellCommandState.h>
#include <robot_actions/DoorActionState.h>
#include <robot_actions/MoveBaseState.h>
#include <robot_actions/RechargeState.h>
#include <robot_msgs/Door.h>
#include <robot_msgs/Pose.h>

namespace TREX {

  /********************************************************
   * CORE TOKEN / ROS TYPE CONVERSIONS
   *******************************************************/
  StringDomain* toStringDomain(const std_msgs::String& msg){
    return new StringDomain(LabelStr(msg.data), "string");
  }

  // bind a string
  void write(const StringDomain& dom, std_msgs::String& msg){
      msg.data = (dom.isSingleton() ? LabelStr(dom.getSingletonValue()).toString() : "");
  }

  // Bind intervals to the singleton, or the domain midpoint
  template <class T>
  void write(const AbstractDomain& dom, T& target){
    double value = (dom.isSingleton() ? dom.getSingletonValue() : (dom.getLowerBound() + dom.getUpperBound() / 2) );
    target = static_cast<T>(value);
  }

  // Bind names paramater to value
  template <class T>
  void write(const char* param_name, const TokenId& token, T& target){
    ConstrainedVariableId var = token->getVariable(param_name);
    ROS_ASSERT(var.isValid());
    const AbstractDomain& dom = var->lastDomain();
    ROS_ASSERT(dom.isNumeric());
    double value;
    write(dom, value);
    target = value;
  }

  
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
   * // Points for the door
   * float door_p1_x;
   * float door_p1_y;
   * float door_p1_z;
   * float door_p2_x;
   * float door_p2_y;
   * float door_p2_z;
   *
   * // Handle data
   * float height;
   * HINGE_FRAME hinge;
   * ROTATION_DIRECTION rot_dir;
   */

  // Write Token to Door message
  void write(const TokenId& token, robot_msgs::Door& msg){
    // Frame Data
    write("frame_p1_x", token, msg.frame_p1.x);
    write("frame_p1_y", token, msg.frame_p1.y);
    write("frame_p1_z", token, msg.frame_p1.z);
    write("frame_p2_x", token, msg.frame_p2.x);
    write("frame_p2_y", token, msg.frame_p2.y);
    write("frame_p2_z", token, msg.frame_p2.z);

    // Door Data
    write("door_p1_x", token, msg.door_p1.x);
    write("door_p1_y", token, msg.door_p1.y);
    write("door_p1_z", token, msg.door_p1.z);
    write("door_p2_x", token, msg.door_p2.x);
    write("door_p2_y", token, msg.door_p2.y);
    write("door_p2_z", token, msg.door_p2.z);

    // Handle Data
    write("height", token, msg.height);
    write("hinge", token, msg.hinge);
    write("rot_dir", token, msg.rot_dir);
  }

  // Bind intervals to the singleton, or the domain midpoint
  template <class T>
  void read(const char* name, ObservationByValue& obs, const T& source){
    obs.push_back(name, new IntervalDomain(source));
  }

  // Read Observation from Door Message
  void read(ObservationByValue& obs, const robot_msgs::Door& msg){
    // Frame Data
    read("frame_p1_x", obs, msg.frame_p1.x);
    read("frame_p1_y", obs, msg.frame_p1.y);
    read("frame_p1_z", obs, msg.frame_p1.z);
    read("frame_p2_x", obs, msg.frame_p2.x);
    read("frame_p2_y", obs, msg.frame_p2.y);
    read("frame_p2_z", obs, msg.frame_p2.z);

    // Door Data
    read("door_p1_x", obs, msg.door_p1.x);
    read("door_p1_y", obs, msg.door_p1.y);
    read("door_p1_z", obs, msg.door_p1.z);
    read("door_p2_x", obs, msg.door_p2.x);
    read("door_p2_y", obs, msg.door_p2.y);
    read("door_p2_z", obs, msg.door_p2.z);

    // Handle Data
    read("height", obs, msg.height);
    read("hinge", obs, msg.hinge);
    read("rot_dir", obs, msg.rot_dir);
  }


  void read(ObservationByValue& obs, const robot_msgs::Pose& msg){
    read("x", obs, msg.position.x);
    read("y", obs, msg.position.y);
    read("th", obs, msg.orientation.w);
  }

  void write(const TokenId& token, robot_msgs::Pose& msg){
    write("x", token, msg.position.x);
    write("y", token, msg.position.y);
    write("th", token, msg.orientation.w);
  }

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
  class MoveBaseAdapter: public ROSActionAdapter<robot_msgs::Pose, robot_actions::MoveBaseState, robot_msgs::Pose> {
  public:

    MoveBaseAdapter(const LabelStr& agentName, const TiXmlElement& configData)
      : ROSActionAdapter<robot_msgs::Pose, robot_actions::MoveBaseState,  robot_msgs::Pose>(agentName, configData){
    }

    virtual ~MoveBaseAdapter(){}

  protected:
    
    virtual void fillActiveObservationParameters(const robot_msgs::Pose& msg, ObservationByValue* obs){
      read(*obs, msg);
    }

    virtual void fillInactiveObservationParameters(const robot_msgs::Pose& msg, ObservationByValue* obs){ 
      read(*obs, msg);
    }

    void fillDispatchParameters(robot_msgs::Pose& msg, const TokenId& goalToken){
      write(goalToken, msg);
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
      read("recharge_level", *obs, msg.data);
    }

    virtual void fillInactiveObservationParameters(const std_msgs::Float32& msg, ObservationByValue* obs){
      read("recharge_level", *obs, msg.data);
    }

    virtual void fillDispatchParameters(std_msgs::Float32& msg, const TokenId& goalToken){
      write("recharge_level", goalToken, msg.data);
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
  class BaseStateAdapter: public ROSStateAdapter<robot_msgs::Pose> {
  public:
    BaseStateAdapter(const LabelStr& agentName, const TiXmlElement& configData)
      : ROSStateAdapter<robot_msgs::Pose> ( agentName, configData) {
    }

    virtual ~BaseStateAdapter(){}

  private:
    void fillObservationParameters(ObservationByValue* obs){
      read(*obs, stateMsg);
    }
  };

  // Allocate a Factory
  TeleoReactor::ConcreteFactory<BaseStateAdapter> l_NewBaseStateAdapter_Factory("NewBaseStateAdapter");
}
