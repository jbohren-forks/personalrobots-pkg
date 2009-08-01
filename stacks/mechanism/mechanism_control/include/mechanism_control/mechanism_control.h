///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2008-2009, Willow Garage, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Stanford University nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/*
 * Author: Stuart Glaser, Wim Meeussen
 */
#ifndef MECHANISM_CONTROL_H
#define MECHANISM_CONTROL_H

#include <pthread.h>
#include <map>
#include <string>
#include <vector>
#include "ros/ros.h"
#include "roslib/Time.h"
#include "mechanism_control/controller_spec.h"
#include "mechanism_control/controller_handle.h"
#include <tinyxml/tinyxml.h>
#include <hardware_interface/hardware_interface.h>
#include <mechanism_model/robot.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node.h>
//#include <misc_utils/advertised_service_guard.h>

#include <mechanism_msgs/ListControllerTypes.h>
#include <mechanism_msgs/ListControllers.h>
#include <mechanism_msgs/SpawnController.h>
#include <mechanism_msgs/KillController.h>
#include <mechanism_msgs/SwitchController.h>
#include <mechanism_msgs/MechanismState.h>
#include <mechanism_msgs/JointStates.h>
#include <diagnostic_msgs/DiagnosticMessage.h>

typedef controller::ControllerHandle* (*ControllerAllocator)();

namespace controller{

class MechanismControl {

public:
  MechanismControl(HardwareInterface *hw);
  virtual ~MechanismControl();

  // Real-time functions
  void update();

  // Non real-time functions
  bool initXml(TiXmlElement* config);
  bool spawnController(const std::string& name);
  bool killController(const std::string &name);
  bool switchController(const std::vector<std::string>& start_controllers,
                        const std::vector<std::string>& stop_controllers,
                        const int strictness);

  // controllers_lock_ must be locked before calling
  controller::ControllerHandle* getControllerByName(const std::string& name);
  template<class ControllerType> bool getControllerByName(const std::string& name, ControllerType*& c)
  {
    // get controller
    controller::ControllerHandle* controller = getControllerByName(name);
    if (controller == NULL) return false;

    // cast controller to ControllerType
    ControllerType* controller_type = dynamic_cast< ControllerType* >(controller);
    if (controller_type == NULL)  return false;

    // copy result
    c = controller_type;
    return true;
  };

  mechanism::Robot model_;
  mechanism::RobotState *state_;
  HardwareInterface *hw_;

private:
  void getControllerNames(std::vector<std::string> &v);
  void getControllerSchedule(std::vector<size_t> &schedule);

  ros::NodeHandle node_;

  // for controller switching
  std::vector<controller::ControllerHandle*> start_request_, stop_request_;
  bool please_switch_, switch_success_;
  int switch_strictness_;

  // controller lists
  boost::mutex controllers_lock_;
  std::vector<ControllerSpec> controllers_lists_[2];
  std::vector<size_t>   controllers_scheduling_[2];
  int current_controllers_list_, used_by_realtime_;

  // for controller statistics
  Statistics pre_update_stats_;
  Statistics update_stats_;
  Statistics post_update_stats_;

  // for publishing constroller state/diagnostics
  void publishDiagnostics();
  void publishState();
  realtime_tools::RealtimePublisher<diagnostic_msgs::DiagnosticMessage> pub_diagnostics_;
  realtime_tools::RealtimePublisher<mechanism_msgs::JointStates> pub_joints_;
  realtime_tools::RealtimePublisher<mechanism_msgs::MechanismState> pub_mech_state_;
  double publish_period_state_, last_published_state_;
  double publish_period_diagnostics_, last_published_diagnostics_;

  // services to work with controllers
  bool listControllerTypesSrv(mechanism_msgs::ListControllerTypes::Request &req,
                              mechanism_msgs::ListControllerTypes::Response &resp);
  bool listControllersSrv(mechanism_msgs::ListControllers::Request &req,
                          mechanism_msgs::ListControllers::Response &resp);
  bool switchControllerSrv(mechanism_msgs::SwitchController::Request &req,
                           mechanism_msgs::SwitchController::Response &resp);
  bool spawnControllerSrv(mechanism_msgs::SpawnController::Request &req,
                          mechanism_msgs::SpawnController::Response &resp);
  bool killControllerSrv(mechanism_msgs::KillController::Request &req,
                         mechanism_msgs::KillController::Response &resp);
  boost::mutex services_lock_; 
  ros::ServiceServer srv_list_controllers_, srv_list_controller_types_, srv_spawn_controller_;
  ros::ServiceServer srv_kill_controller_, srv_switch_controller_;
};

}
#endif /* MECHANISM_CONTROL_H */
