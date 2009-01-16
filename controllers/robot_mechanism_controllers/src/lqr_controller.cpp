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

#include <robot_mechanism_controllers/lqr_controller.h>
#include <control_toolbox/serialchain_model.h>
#include <mechanism_model/joint.h>
#include <control_toolbox/LQRDP.h>
#include <Eigen/Array>
#include <rosconsole/rosconsole.h>

#include <robot_mechanism_controllers/ros_serialchain_model.h>

using std::string;
using namespace controller;

ROS_REGISTER_CONTROLLER(LQRController);

LQRController::LQRController()
{
  ROS_DEBUG("creating controller...");
//   target_.setZero();
//   gains_.setZero();
//   input_offset_.setZero();
  ROS_DEBUG("creating controller...");
  model_ = NULL;
//   reset_previous_=true;
}

LQRController::~LQRController()
{
  queue_.clear();
  delete model_;
}

void LQRController::update()
{
  queue_.processAll();
  
  //Fills the current state
  model_->toState(robot_,state_);

  //Computes update from controller
  commands_=-gains_*(state_-target_); //+input_offset_;
  
  //Updates joints
  model_->setEffort(robot_,commands_);
}

bool LQRController::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  robot_=robot;
  return true;
} 

void LQRController::setModel(ModelType *model)
{
  model_=model;
  assert(model_);
  const int n=model_->states();
  const int m=model_->inputs();
  //Using the set method ensures the matrices are resized if necessary
  target_.set(StateVector::Zero(n,1));
  gains_.set(GainsMatrix::Zero(m,n));
  state_.set(StateVector::Zero(n,1));
  input_offset_.set(InputVector::Zero(m,1));
  commands_.set(InputVector::Zero(m,1));
}

//------------------ Jobs --------------

LQRController::UpdateGainsJob::UpdateGainsJob(LQRController * c, const StateMatrix & weights, const InputMatrix &input_weights, const StateVector & target)
: c_(c), target_(target)
{
  if(!c)
    return;
  ROS_DEBUG_STREAM("target is "<<target.transpose());
  const int n=weights.rows();
  const int m=input_weights.rows();
  
  input_offset_.resize(m,1);
  gains_.resize(m,n);
  
  //Computes a linearization around the target point
  StateMatrix A(n,n);
  InputMatrix B(n,m);
  if(!c_->model_->getLinearization(target, A, B, input_offset_))
  {
    ROS_ERROR("Failed to get linearization");
    c_=NULL;
    return;
  }
  ROS_DEBUG_STREAM("************ Weights **********\n[Q]\n"<<weights<<std::endl<<"[R]"<<std::endl<<input_weights<<std::endl<<"[input offset]"<<std::endl<<input_offset_<<"\n*************");
  ROS_DEBUG_STREAM("************ Obtained linearization **********\n[A]\n"<<A<<std::endl<<"[B]"<<std::endl<<B<<std::endl<<"[input offset]"<<std::endl<<input_offset_<<"\n*************");
  ROS_DEBUG("Sanity checks");
  ROS_ASSERT(LQR::isCommandable(A,B));
  // Compute new gains matrix
  ROS_DEBUG("Computing LQR gains matrix. Hold on to your seatbelts...");
  LQR::LQRDP<StateMatrix,InputMatrix,StateCostMatrix,InputCostMatrix>::runContinuous(A, B, weights,input_weights, gains_);
  ROS_DEBUG_STREAM("Done. Computed gains:\n***********\n"<<gains_<<"\n**********\n");
  // Check validity
  ROS_DEBUG("Checking gains matrix...");
  bool test=LQR::isConverging(A,B,gains_);
  ROS_ASSERT(test);
  ROS_DEBUG("Valid gains matrix");
//   assert(test);
}

//------------------- NODE -------------

ROS_REGISTER_CONTROLLER(LQRControllerNode);

LQRControllerNode::LQRControllerNode()
{
  ROS_DEBUG("creating controller node...");
  c_ = new LQRController();
  ROS_DEBUG("done");
}

LQRControllerNode::~LQRControllerNode()
{
  delete c_;
}

void LQRControllerNode::update()
{
  c_->update();
}

//FIXME: this should be moved elsewhere and integrated in some API
LQRController::ModelType * createModel(mechanism::RobotState * robot, TiXmlElement *config)
{
  if(!config)
    return NULL;
  LQRController::ModelType *model=NULL;
  std::string model_name=static_cast<std::string>(config->Attribute("name"));
  ROS_DEBUG_STREAM("Found model name: "<<model_name);
  if(model_name=="serial_chain")
  {
    ROS_DEBUG("Attempting to load serial chain model");
    model=new SerialChainModelWrapper();
  }
  if(model&&!model->initXml(robot,config))
  {
    ROS_DEBUG("Some error occurred... Clearing model");
    delete model;
    return NULL;
  }
  return model;
}

bool LQRControllerNode::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  //Init the model.
  
  ROS_DEBUG("LOADING LQR CONTROLLER NODE");
  ros::Node * const node = ros::Node::instance();
  string prefix = config->Attribute("name");
  ROS_DEBUG_STREAM("the prefix is "<<prefix);

  
  // Parses subcontroller configuration
  if(c_->initXml(robot, config))
  {
    node->advertiseService(prefix + "/set_target", &LQRControllerNode::setTargetSrv, this);
    node->advertiseService(prefix + "/set_command", &LQRControllerNode::setLQRParamsSrv, this);

    TiXmlElement *model_xml = config->FirstChildElement("model");
    if(!model_xml)
    {
      ROS_ERROR("Missing model description");
      return false;
    }
    LQRController::ModelType * model=createModel(robot,model_xml);

    if(!model)
    {
      ROS_DEBUG("LQR controller failed to load a model");
      return false;
    }
    ROS_DEBUG("Setting controller model");
    c_->setModel(model);
    // Default parameters
    const int n = model->states();
    const int m = model->inputs();
    state_cost_.set(LQRController::StateCostMatrix::Identity(n,n));
    input_cost_.set(LQRController::InputCostMatrix::Identity(m,m));
    target_.set(LQRController::StateVector::Zero(n,1));
    
    ROS_DEBUG("DONE LOADING LQR CONTROLLER NODE\n");
    return model;
  }
  ROS_DEBUG("ERROR LOADING LQR CONTROLLER NODE");
  return false;
}

bool LQRControllerNode::setLQRParamsSrv(robot_mechanism_controllers::SetLQRCommand::request &req,
              robot_mechanism_controllers::SetLQRCommand::response &resp)
{
  const int n = c_->model()->states();
  const int m = c_->model()->inputs();
  
  //Decodes state cost
  const std::vector<double> & s_c = req.state_cost;
  if((int)s_c.size()!=n && (int)s_c.size()!=n*n && (int)s_c.size()!=0)
  {
    ROS_ERROR_STREAM("Bad length for state cost vector: # of states is "<<n<<", vector size is "<<s_c.size());
    return false;
  }
  ROS_DEBUG("0");
  if((int)s_c.size() == n)
  {
    state_cost_.setZero();
    for(int i=0;i<n;i++)
      state_cost_(i,i)=s_c.at(i);
  }
  ROS_DEBUG("1");
  if((int)s_c.size() == n*n)
  {
    for(int i=0;i<n;i++)
      for(int j=0;j<n;j++)
        state_cost_(i,j)=s_c.at(i*n+j);
  }
  
//   ROS_DEBUG("1bis");
  const std::vector<double> & i_c = req.input_cost;
  if((int)i_c.size()!=m && (int)i_c.size()!=m*m && i_c.size()!=0)
  {
    ROS_ERROR_STREAM("Bad length for state cost vector: # of states is "<<m<<", vector size is "<<i_c.size());
    return false;
  }
//   ROS_DEBUG("2");
  if((int)i_c.size() == m)
  {
    input_cost_.setZero();
    for(int i=0;i<m;i++)
      input_cost_(i,i)=i_c.at(i);
  }
//   ROS_DEBUG("3");

  if((int)i_c.size() == m*m)
  {
    for(int i=0;i<m;i++)
      for(int j=0;j<m;j++)
        input_cost_(i,j)=i_c.at(i*m+j);
  }
  
//   ROS_DEBUG("4");
  if(!c_->model()->toState(&(req.target),target_))
    return false;
//   ROS_DEBUG("5");
  
  c_->add(new LQRController::UpdateGainsJob(c_,state_cost_,input_cost_,target_));
//   ROS_DEBUG("6");
  
  return true;
}

bool LQRControllerNode::setTargetAsynchronous(const robot_msgs::JointCmd &cmd)
{
  if(!c_->model()->toState(&cmd,target_))
    return false;
  c_->add(new LQRController::UpdateTargetJob(c_,target_));
  return true;
}

//FIXME robot_srvs::SetJointCmd should use robot_msgs::JointCmd  
bool LQRControllerNode::setTargetSrv(robot_srvs::SetJointCmd::request &req,
              robot_srvs::SetJointCmd::response &resp)
{
  
  robot_msgs::JointCmd cmd;
  cmd.names=req.names;
  cmd.positions=req.positions;
  cmd.velocity=req.velocity;
  return setTargetAsynchronous(cmd);
/*  if(!c_->model_->toState(req.names,req.positions,req.velocity,target_))
    return false;
  
  return true;*/
}

