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
#ifndef GENERIC_LQR_CONTROLLER_H
#define GENERIC_LQR_CONTROLLER_H

#include <ros/node.h>
#include <mechanism_model/controller.h>
#include <mechanism_model/robot.h>

#include <robot_mechanism_controllers/SetLQRCommand.h> //<- should we depend on the robot services for that?
#include <robot_msgs/JointCmd.h>
#include <robot_srvs/SetJointCmd.h>

#include "control_toolbox/system_model.h"
#include <control_toolbox/control_test.h>
#include <misc_utils/job_queue.h>
using misc_utils::JobQueue;
using misc_utils::JobQueueItem;
/** @class LQRController
  * @brief Implements a LQR controller (code for the realtime loop)
  * 
  */
  
namespace controller
{

class LQRControllerNode;

class LQRController : public Controller
{
  friend class LQRControllerNode;
  
public:
  typedef DynamicsModel<double, Eigen::Dynamic, Eigen::Dynamic> ModelType;
  typedef ModelType::StateMatrix StateMatrix;
  typedef ModelType::StateVector StateVector;
  typedef ModelType::InputMatrix InputMatrix;
  typedef ModelType::InputVector InputVector;
  typedef Eigen::MatrixXd GainsMatrix;
  typedef Eigen::MatrixXd InputCostMatrix;
  typedef Eigen::MatrixXd StateCostMatrix;
  
  // Queue Jobs
  
  class UpdateGainsJob : public JobQueueItem
  {
  private:
    LQRController * c_;
    GainsMatrix gains_;
    StateVector target_;
    InputVector input_offset_;
  public:
  
    UpdateGainsJob(LQRController * c, const GainsMatrix & g, const StateVector & target, const InputVector & input_off) : c_(c),gains_(g),target_(target),input_offset_(input_off) {}
    
    UpdateGainsJob(LQRController * c, const StateCostMatrix & s_cost, const InputCostMatrix &i_cost, const StateVector & target);
    
    void process()
    {
      std::cout<<"NEW GAINS\n"<<gains_<<std::endl;
      if(c_)
      {
        c_->gains_=gains_;
        c_->target_=target_;
        c_->input_offset_=input_offset_;
      }
    }
  };

  class UpdateTargetJob : public JobQueueItem
  {
  private:
    LQRController * c_;
    StateVector target_;
  public:
    UpdateTargetJob(LQRController * c, const StateVector & t) : c_(c),target_(t) {}
    void process()
    {
      c_->target_=target_;
    }
  };
  
  LQRController();
  
  ~LQRController();
  
  //Can be safely called from non real-time code to update the state of the controller
  void add(JobQueueItem * job) { queue_.push(job); }

  void update();
  
  bool initXml(mechanism::RobotState *robot, TiXmlElement *config);
  
  void setModel(ModelType *model);
  ModelType * model() { return model_; }
  
private:

  JobQueue queue_; // Job queue to do I/O with non-RT part
  
  GainsMatrix gains_;   // The control matrix
  
//   StateVector state_offset_;
  InputVector input_offset_;
  
  StateVector target_; // Target state
  
  // Model of the arm
  ModelType * model_;
  
  StateVector state_; 
//   previous_state_;
  
  InputVector commands_;
  
  mechanism::RobotState * robot_;

};

/** @class LQRControllerNode
* @brief Provides a thin wrapper for ROS communicaition with the LQR controller
* Most of the input/output is done through the LQRController job queue
*/
class LQRControllerNode : public Controller
{
public:
  LQRControllerNode();
  
  virtual ~LQRControllerNode();

  bool setLQRParamsSrv(robot_mechanism_controllers::SetLQRCommand::request &req,
                robot_mechanism_controllers::SetLQRCommand::response &resp);
                
  bool setTargetSrv(robot_srvs::SetJointCmd::request &req,
                robot_srvs::SetJointCmd::response &resp);
                
  bool setTargetAsynchronous(const robot_msgs::JointCmd &cmd);
                
  void update();
  
  bool initXml(mechanism::RobotState *robot, TiXmlElement *config);
  
private:
  
  // Cached cost matrices so that the user can use default parameters when updating the controller
  // Initially set to the identity
  LQRController::InputCostMatrix input_cost_;
  LQRController::StateCostMatrix state_cost_;
  LQRController::StateVector target_;
  
  LQRController * c_;
};

};

#endif

