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
  
  virtual ~LQRController();
  
  //Can be safely called from non real-time code to update the state of the controller
  void add(JobQueueItem * job) { queue_.push(job); }

  void update();
  
  bool initXml(mechanism::RobotState *robot, TiXmlElement *config);
  
  void setModel(ModelType *model);
  ModelType * model() { return model_; }

  // FIXME: the API to convert model state  <-> robot parameters is really bad
  // Need to think about it
  // Code also used in LQRControllerNode
  
  /** @brief fills a state vector from information supplied by n-uplets of (name,position,velocity)
    */
  bool toStateVector(const std::vector<std::string> & names, const std::vector<double> & positions, const std::vector<double> & velocities, StateVector & v) const;
  
protected:

  virtual void getValues();
  virtual void setCommands();
  
private:

  /** @brief Given a joint name, return the index of this joint in joint_states_, or -1 if it not present
    */
  //FIXME: should be removed by using a more efficient storage structure for joint_states_ (hash map)
  int jointIndex(const std::string & name) const;

  JobQueue queue_; // Job queue to do I/O with non-RT part
  
  GainsMatrix gains_;   // The control matrix
  
  StateVector state_offset_;
  InputVector input_offset_;
  
  StateVector target_; // Target state
  
  // Model of the arm
  ModelType * model_;
  
  std::vector<mechanism::JointState *> joint_states_; // Lets a mapping from the joints (name) to the vector (indexes)
  
  StateVector state_, previous_state_;
  
  InputVector commands_;
  
  mechanism::RobotState * robot_;
  
  double previous_time_;
  
  bool reset_previous_; //Reset the value of previous_state_ to the current state

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


// template<typename Scalar = double, int StateSize = Eigen::Dynamic, int InputSize = Eigen::Dynamic>
// class LQRController
// {
//   typedef DynamicsModel<Scalar, StateSize, InputSize> ModelType;
//   typedef LQRController<Scalar, StateSize, InputSize> _LQRController;
//   
// public:
//   typedef typename ModelType::StateMatrix StateMatrix;
//   typedef typename ModelType::StateVector StateVector;
//   typedef typename ModelType::InputMatrix InputMatrix;
//   typedef typename ModelType::InputVector InputVector;
//   typedef Eigen::Matrix<Scalar, InputSize, StateSize> GainsMatrix;
//   
//   // Queue Jobs
//   
//   class UpdateGainsJob : public JobQueueItem
//   {
//   private:
//     _LQRController * c_;
//     GainsMatrix gains_;
//     StateVector off_;
//   public:
//     UpdateGainsJob(_LQRController * c, const GainsMatrix & g, const StateVector & off) : c_(c),gains_(g),off_(off) {}
//     void process()
//     {
//       std::cout<<"NEW GAINS\n"<<gains_<<std::endl;
//       c_->gains_=gains_;
//       c_->offset_=off_;
//     }
//     
//   };
// 
//   class UpdateTargetJob : public JobQueueItem
//   {
//   private:
//     _LQRController * c_;
//     StateVector target_;
//   public:
//     UpdateTargetJob(_LQRController * c, const StateVector & t) : c_(c),target_(t) {}
//     void process()
//     {
//       c_->target_=target_;
//     }
//     
//   };
//   
//   LQRController()
//   {
//     target_.setZero();
//     cmatrix_.setZero();
//   }
//   
//   ~LQRController()
//   {
//     queue_.clear();
//   }
//   
//   //Can be safely called from non real-time code to update the state of the controller
// //   JobQueue & queue() { return queue_; }
//   void add(JobQueueItem * job) { queue_.push(job); }
// 
//   // To be called in the real time loop
//   // Processes the queue and updates the commands vector
//   void updateCommands(const StateVector & current, double dt, InputVector &commands)
//   {
//     queue_.processAll();
//     commands=dt*cmatrix_*(current-target_+offset_);
// //     std::cout<<"$$$$$ GAINS=\n"<<cmatrix_<<std::endl;
// //     std::cout<<"$$$$$ current=\n"<<current<<std::endl;
// //     std::cout<<"$$$$$ COM="<<commands.transpose()<<std::endl;
//   }
//   
// private:
// 
//   JobQueue queue_; // Job queue to do I/O with non-RT part
//   
//   GainsMatrix cmatrix_;   // The control matrix
//   
//   StateVector offset_;
//   
//   StateVector target_; // Target state
// };
// 
// 
// /** @brief non real time interface to the LQR controller
//   */
// template<typename Scalar, int StateSize, int InputSize>
// class LQRControllerInterface
// {
// private:
//   typedef DynamicsModel<Scalar, StateSize, InputSize> ModelType;
//   typedef LQRController<Scalar,StateSize,InputSize> _LQRController;
//   
// public:
//   typedef typename ModelType::StateMatrix StateMatrix;
//   typedef typename ModelType::StateVector StateVector;
//   typedef typename ModelType::InputMatrix InputMatrix;
//   typedef typename ModelType::InputVector InputVector;
//   typedef Eigen::Matrix<Scalar, InputSize, StateSize> GainsMatrix;
//   typedef Eigen::Matrix<Scalar, InputSize, InputSize> InputWeightsMatrix;
//   typedef int OutputMatrix;
// private:  
//   
// public:
// 
//   LQRControllerInterface(_LQRController * controller) : c_(controller)
//   {
//   }
//   
//   bool init(const InputMatrix & cmatrix, ModelType * model)
//   {
//     model_=model;
//     c_->add(new typename _LQRController::UpdateGainsJob(c_,cmatrix));
//     return true;
//   }
//   
//   void updateTarget(const StateVector & target)
//   {
//     c_->add(new typename _LQRController::UpdateTargetJob(c_,target));
//   }
//   
//   void updateInputMatrix(const StateVector & target_state, const StateMatrix & lqrWeights)
//   {
//     // Get new linearization
//     StateMatrix A;
//     InputMatrix B;
//     StateVector c;
//     model_->getLinearization(target_state, A, B, c);
//     
//     // Compute new gains matrix
//     StateMatrix K;
//     InputWeightsMatrix R=InutWeightsMatrix::Identity();
//     LQR::LQRDP<StateMatrix,InputMatrix,StateMatrix,InputWeightsMatrix>::run(A, B, lqrWeights, lqrWeights, R, 0.1, K);
// 
//     // Check validity
//     bool test=SystemTest<StateMatrix,InputMatrix,OutputMatrix>::isConverging(A,B,K);
//     assert(test);
//     
//     // Sends gains matrix to the controller
//     c_->add(new typename _LQRController::UpdateGainsJob(c_,K,A.inverse()*c));
//   }
// 
// private:
//   _LQRController * c_;
//   
//   ModelType * model_;
// };

};

#endif