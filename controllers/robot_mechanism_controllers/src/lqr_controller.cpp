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

using std::string;
using namespace controller;

ROS_REGISTER_CONTROLLER(LQRController);

LQRController::LQRController()
{
  target_.setZero();
  gains_.setZero();
  state_offset_.setZero();
  input_offset_.setZero();
  model_ = NULL;
  reset_previous_=true;
}

LQRController::~LQRController()
{
  queue_.clear();
  delete model_;
}

void LQRController::update()
{
  queue_.processAll();
  
  const double t=robot_->hw_->current_time_;
  const double dt=t-previous_time_;
  
  //Fills the current state
  getValues();

  //Updates the model if needed
  model_->observation(state_, previous_state_, commands_, dt);
  
  //Computes update from controller
  commands_=dt*gains_*(state_-target_+state_offset_)+input_offset_;
  
  //Updates joints
  setCommands();
  
  previous_state_=state_;
  previous_time_=t;
}

void LQRController::getValues()
{
  const int n=joint_states_.size();
  assert(n*2 == state_.rows());
  previous_state_=state_;
  
  for(int i=0;i<n;i++)
  {
    state_(i,0)=joint_states_.at(i)->position_;  
    state_(i+n,0)=joint_states_.at(i)->velocity_;  
    commands_(i,0)=joint_states_.at(i)->applied_effort_;
  }
  
  if(reset_previous_)
  {
    previous_state_=state_;
    reset_previous_=false;
  }
}

void LQRController::setCommands()
{
  for(unsigned int i=0;i<joint_states_.size();++i)
  {
    joint_states_.at(i)->commanded_effort_=commands_(i,0);  
//     std::cout<<joint_states_.at(i)->commanded_effort_<<std::endl;
  }
}

bool LQRController::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  robot_=robot;
  previous_time_=robot_->hw_->current_time_;

  TiXmlElement *elt = config->FirstChildElement("joints");
  if(!elt)
  {
    ROS_ERROR("No list of joints\nYou have to specify a list of joints to the LQR controller");
    return false;
  }
  elt=elt->FirstChildElement("joint");
  if(!elt)
  {
    ROS_ERROR("No list of joints\nYou have to specify a list of joints to the LQR controller");
    return false;
  }
  while (elt)
  {
    const std::string & name=static_cast<std::string>(elt->Attribute("name"));
    mechanism::JointState * state=robot_->getJointState(name);
    if(!state)
    {
      ROS_ERROR_STREAM("Could not find joint called "<<name);
      return false;
    }
    
    joint_states_.push_back(state);
    elt = elt->NextSiblingElement("joint");
  }
  ROS_DEBUG("State Vector:");
  for(uint i=0;i<joint_states_.size();++i)
    ROS_DEBUG_STREAM('\t'<<joint_states_[i]->joint_->name_);
  
  return true;
} 

void LQRController::setModel(ModelType *model)
{
  model_=model;
  assert(model_);
  const int n=model_->states();
  const int m=model_->inputs();
  target_=StateVector::Zero(n,1);
  gains_=GainsMatrix::Zero(m,n);
  state_offset_=StateVector::Zero(n,1);
  state_=StateVector::Zero(n,1);
  previous_state_=StateVector::Zero(n,1);
  input_offset_=InputVector::Zero(m,1);
  commands_=InputVector::Zero(m,1);
}

bool LQRController::toStateVector(const std::vector<std::string> & names, const std::vector<double> & positions, const std::vector<double> & velocities, StateVector & v) const
{
  const int N = names.size();
  assert(model_);
  const int n = model_->states();
  if(v.rows() != n)
  {
    ROS_ERROR("Supplied state vector has wrong size");
    return false;
  }

  if((int)positions.size()!=N or (int)velocities.size()!=N)
  {
    ROS_ERROR("Incoherent data supplied");
    return false;
  }
  
  int count=1;
  for(int i=0;i<N;i++)
  {
    const int index=jointIndex(names.at(i));
    if(index>=0)
    {
      v(index,0)=positions.at(index);
      v(index+n/2,0)=velocities.at(index);
      count++;
    }
  }
  
  if(count!=n)
    ROS_INFO("The state was only partly specified");
  return true;
}

int LQRController::jointIndex(const std::string &name) const 
{
  for(uint i=0; i<joint_states_.size();i++)
    if(joint_states_.at(i)->joint_->name_==name)
      return i;
  return -1;
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
//   const int m=input_weights.rows();
  input_offset_.resize(n,1);
  gains_.resize(n,n);
  
  //Computes a linearization around the target point
  StateMatrix A(n,n);
  InputMatrix B(n,m);
  if(!c_->model_->getLinearization(target, A, B, input_offset_))
  {
    ROS_ERROR("Failed to get linearization");
    c_=NULL;
    return;
  }
  ROS_DEBUG_STREAM("************ Obtained linearization **********\n"<<A<<std::endl<<std::endl<<B<<std::endl<<std::endl<<input_offset_<<"\n*************");
  // Compute new gains matrix
  ROS_DEBUG("Computing LQR gains matrix. Hold on to your seatbelts...");
  LQR::LQRDP<StateMatrix,InputMatrix,StateMatrix,InputMatrix>::runContinuous(A, B, weights, weights, input_weights, 0.1, 0.01, gains_);
  ROS_DEBUG_STREAM("Done:\n***********\n"<<gains_<<"\n**********\n");
  // Check validity
  typedef int OutputMatrix;
  //FIXME: the test is not working for now
//   bool test=LQR::isConverging(A,B,gains_);
//   assert(test);
}

//------------------- NODE -------------

ROS_REGISTER_CONTROLLER(LQRControllerNode);

LQRControllerNode::LQRControllerNode()
{
  c_ = new LQRController();
}

LQRControllerNode::~LQRControllerNode()
{
  delete c_;
}

void LQRControllerNode::update()
{
  c_->update();
}

bool LQRControllerNode::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  //Init the model.
  
  ROS_DEBUG("LOADING LQR CONTROLLER NODE");
  ros::node * const node = ros::node::instance();
  string prefix = config->Attribute("name");
  ROS_DEBUG_STREAM("the prefix is "<<prefix);

  // Parses controller configuration.
  std::string kdl_chain_name="";
  std::string model_type="";
  
  TiXmlElement *j = config->FirstChildElement("map");
  if(!j)
  {
    ROS_ERROR("Missing configuration options");
    return false;
  }
  
  j = j->FirstChildElement("elem");
  
  if(!j)
  {
    ROS_ERROR("Missing element field");
    return false;
  }
  
  while(j!=NULL)
  {
    if(j->Attribute("key")==std::string("kdl_chain_name"))
      kdl_chain_name = j->GetText();

    if(j->Attribute("key")==std::string("model_type"))
      model_type = j->GetText();
      
    j = j->NextSiblingElement("elem");
  }
  
  ROS_DEBUG_STREAM("the model is "<<model_type);

  
  // Parses subcontroller configuration
  if(c_->initXml(robot, config))
  {
    node->advertise_service(prefix + "/set_target", &LQRControllerNode::setTargetSrv, this);
    node->advertise_service(prefix + "/set_command", &LQRControllerNode::setLQRParamsSrv, this);

    LQRController::ModelType * model=NULL;
    
    // Parses kinematics description
    if(model_type=="serialchain")
    {
      ROS_DEBUG("Creating serial chain");
      std::string pr2Contents;
      node->get_param("robotdesc/pr2", pr2Contents);
      SerialChainModel * model1 = new SerialChainModel();
      
      if(!model1->init(pr2Contents, kdl_chain_name))
      {
        delete model1;
        ROS_ERROR("Could not load model parameters");
        return false;
      }
      model=model1;
    }
    ROS_DEBUG_STREAM("Model has "<<model->states()<<" states and "<<model->inputs()<<" inputs, controller loaded "<< c_->joint_states_.size()<<" joints");
    
    if(model && model->states()!=2*model->inputs() && model->inputs() != (int)(c_->joint_states_.size()))
    {
      ROS_ERROR("Size mismatch between the model loaded and the joints provided\nFor info:\n");
      ROS_ERROR_STREAM("Model has "<<model->states()<<" states and "<<model->inputs()<<" inputs, controller loaded "<< c_->joint_states_.size()<<" joints");
      delete model;
      return false;
    }

    c_->setModel(model);
    if(!model)
    {
      ROS_DEBUG("LQR controller failed to load a model");
      return false;
    }
    // Default parameters
    const int n = model->states();
    const int m = model->inputs();
    state_cost_ = LQRController::StateCostMatrix::Identity(n,n);
    input_cost_ = LQRController::InputCostMatrix::Identity(m,m);
    target_=LQRController::StateVector::Zero(n,1);
    
    ROS_DEBUG("DONE LOADING LQR CONTROLLER NODE\nYou have to load parameters now.");
    return model;
  }
  ROS_DEBUG("ERROR LOADING LQR CONTROLLER NODE");
  return false;
}

bool LQRControllerNode::setLQRParamsSrv(robot_mechanism_controllers::SetLQRCommand::request &req,
              robot_mechanism_controllers::SetLQRCommand::response &resp)
{
  // FIXME: factor conversion of state <-> robot parameters for more genericity
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
  
  ROS_DEBUG("1bis");
  const std::vector<double> & i_c = req.input_cost;
  if((int)i_c.size()!=m && (int)i_c.size()!=m*m && i_c.size()!=0)
  {
    ROS_ERROR_STREAM("Bad length for state cost vector: # of states is "<<m<<", vector size is "<<i_c.size());
    return false;
  }
  ROS_DEBUG("2");
  if((int)i_c.size() == m)
  {
    input_cost_.setZero();
    for(int i=0;i<m;i++)
      input_cost_(i,i)=i_c.at(i);
  }
  ROS_DEBUG("3");

  if((int)i_c.size() == m*m)
  {
    for(int i=0;i<m;i++)
      for(int j=0;j<m;j++)
        input_cost_(i,j)=i_c.at(i*m+j);
  }
  
  ROS_DEBUG("4");
  if(!c_->toStateVector(req.target.names,req.target.positions,req.target.velocity,target_))
    return false;
  ROS_DEBUG("5");
  
  c_->add(new LQRController::UpdateGainsJob(c_,state_cost_,input_cost_,target_));
  ROS_DEBUG("6");
  
  return true;
}
              
bool LQRControllerNode::setTargetSrv(robot_srvs::SetJointCmd::request &req,
              robot_srvs::SetJointCmd::response &resp)
{
  if(!c_->toStateVector(req.names,req.positions,req.velocity,target_))
    return false;
  c_->add(new LQRController::UpdateTargetJob(c_,target_));
  return true;
}

