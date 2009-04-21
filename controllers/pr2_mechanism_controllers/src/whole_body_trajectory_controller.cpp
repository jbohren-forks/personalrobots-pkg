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

// Original version: Sachin Chitta <sachinc@willowgarage.com>

#include "pr2_mechanism_controllers/whole_body_trajectory_controller.h"

using namespace controller;
using namespace std;

ROS_REGISTER_CONTROLLER(WholeBodyTrajectoryController);


WholeBodyTrajectoryController::WholeBodyTrajectoryController() :
  refresh_rt_vals_(false),trajectory_type_("linear"),trajectory_wait_time_(0.0), max_update_time_(0.0)
{
  controller_state_publisher_ = NULL;
  alpha_filter_ = 0.5;
}

WholeBodyTrajectoryController::~WholeBodyTrajectoryController()
{
  // Assumes the joint controllers are owned by this class.
  for(unsigned int i=0; i<joint_pv_controllers_.size();++i)
    delete joint_pv_controllers_[i];
}

bool WholeBodyTrajectoryController::initXml(mechanism::RobotState * robot, TiXmlElement * config)
{
  ROS_INFO("Initializing trajectory controller");
  int num_joints = 0;

  robot_ = robot->model_;
  mechanism::Joint *joint;
  std::vector<trajectory::Trajectory::TPoint> trajectory_points_vector;

  TiXmlElement *elt = config->FirstChildElement("controller");
  while (elt)
  {
    if(static_cast<std::string>(elt->Attribute("type")) == std::string("JointPVController"))
    {
      JointPDController * jpc = new JointPDController();
      ROS_INFO("Joint PD Controller: %s , %s",(elt->Attribute("type")),(elt->Attribute("name")));

      joint_pv_controllers_.push_back(jpc);
      if(!jpc->initXml(robot, elt))
        return false;

      joint = (robot->getJointState(jpc->getJointName()))->joint_;

      if(joint)
      {
        joint_velocity_limits_.push_back(joint->velocity_limit_*velocity_scaling_factor_);
        joint_type_.push_back(joint->type_);
      }
      else
      {
        ROS_ERROR("Joint %s not found in class robot",jpc->getJointName().c_str());
      }
      joint_name_.push_back(std::string(jpc->getJointName()));
      current_joint_position_.push_back(0.0);
      current_joint_velocity_.push_back(0.0);
      joint_errors_.push_back(0.0);
      num_joints++;
    }
    else if(static_cast<std::string>(elt->Attribute("type")) == std::string("BasePIDController"))
    {

      TiXmlElement *pj = elt->FirstChildElement("joint");
      if(pj)
      {
        std::string joint_name = static_cast<std::string>(pj->Attribute("name"));
        joint_name_.push_back(joint_name);        

        double velocity_limit = atof(pj->Attribute("limit"));
        joint_velocity_limits_.push_back(velocity_limit*velocity_scaling_factor_);

        if(static_cast<std::string>(pj->Attribute("type")) == std::string("ROTARY_CONTINUOUS"))
        {
          ROS_INFO("Adding joint %s with type: ROTARY_CONTINUOUS",joint_name.c_str());
          joint_type_.push_back(mechanism::JOINT_CONTINUOUS);
        }
        else if(static_cast<std::string>(pj->Attribute("type")) == std::string("PRISMATIC_CONTINUOUS"))
        {
          ROS_INFO("Adding joint %s with type: PRISMATIC_CONTINUOUS",joint_name.c_str());
          joint_type_.push_back(mechanism::JOINT_PRISMATIC);
        }
        else
        {
          ROS_ERROR("Unknown type %s for base joint %s",(static_cast<std::string>(pj->Attribute("type"))).c_str(),joint_name.c_str());
        }

        TiXmlElement *p = pj->FirstChildElement("pid");
        control_toolbox::Pid pid;
        if (p)
          pid.initXml(p);
        else
          ROS_ERROR("BasePIDController's config did not specify the default pid parameters.\n");
      
        base_pid_controller_.push_back(pid);

        base_joint_index_.push_back(num_joints);

      }
      current_joint_position_.push_back(0.0);
      current_joint_velocity_.push_back(0.0);
      joint_errors_.push_back(0.0);
      num_joints++;
    }
    else if(static_cast<std::string>(elt->Attribute("type")) == std::string("BaseControllerNode"))
    {        
      base_controller_node_.initXml(robot,elt);
    }
    else
    {
      ROS_ERROR("Unrecognized joint controller type: %s",(static_cast<std::string>(elt->Attribute("type"))).c_str());
    }
    elt = elt->NextSiblingElement("controller");
  }

  ROS_INFO("WholeBodyTrajectoryController:: Done loading controllers");

  elt = config->FirstChildElement("trajectory");

  if(!elt)
    ROS_WARN("No trajectory information in xml file. ");
  else
  {
    trajectory_type_ = std::string(elt->Attribute("interpolation"));
    ROS_INFO("WholeBodyTrajectoryController:: interpolation type:: %s",trajectory_type_.c_str());
  }
  joint_cmd_rt_.resize(num_joints);
  joint_cmd_dot_rt_.resize(num_joints);

  joint_trajectory_ = new trajectory::Trajectory((int) num_joints);

  joint_trajectory_->setMaxRates(joint_velocity_limits_);
  joint_trajectory_->setInterpolationMethod(trajectory_type_);
  joint_trajectory_->setJointWraps(base_joint_index_[2]);

  trajectory_point_.setDimension(num_joints);
  dimension_ = num_joints;

  updateJointValues();

  for(int i=0; i < dimension_; i++)
    trajectory_point_.q_[i] = current_joint_position_[i];

  trajectory_point_.time_ = 0.0;

  trajectory_points_vector.push_back(trajectory_point_);
  trajectory_points_vector.push_back(trajectory_point_);

  joint_trajectory_->autocalc_timing_ = true;

  ROS_INFO("Size of trajectory points vector : %d",trajectory_points_vector.size());
  if(!joint_trajectory_->setTrajectory(trajectory_points_vector))
    ROS_WARN("Trajectory not set correctly");
  else
    ROS_INFO("Trajectory set correctly");

  trajectory_start_time_ = robot_->hw_->current_time_;

  last_time_ = robot_->hw_->current_time_;

  watch_dog_active_ = false;
  last_update_time_ = robot_->hw_->current_time_;

  ROS_INFO("Initialized controllers");

  return true;
}

void WholeBodyTrajectoryController::setTrajectoryCmd(const std::vector<trajectory::Trajectory::TPoint>& joint_trajectory)
{
  if(joint_trajectory.size() < 1)
  {
    //   ROS_WARN("WholeBodyTrajectoryController:: No points in trajectory");
    return;
  }

  joint_trajectory_->setTrajectory(joint_trajectory);
//  joint_trajectory_->write("foo.txt",0.01);
  arm_controller_lock_.lock();
  refresh_rt_vals_ = true;
  arm_controller_lock_.unlock();
}

void WholeBodyTrajectoryController::getJointPosCmd(pr2_mechanism_controllers::JointPosCmd & cmd) const
{
}

controller::JointPDController* WholeBodyTrajectoryController::getJointControllerByName(std::string name)
{
  for(int i=0; i< (int) joint_pv_controllers_.size(); i++)
  {
    if(joint_pv_controllers_[i]->getJointName() == name)
    {
      return joint_pv_controllers_[i];
    }
  }
  return NULL;
}

int WholeBodyTrajectoryController::getJointControllerPosByName(std::string name)
{
  for(int i=0; i< (int) joint_pv_controllers_.size(); i++)
  {
    if(joint_pv_controllers_[i]->getJointName() == name)
    {
      return i;
    }
  }
  return -1;
}

bool WholeBodyTrajectoryController::errorsWithinThreshold()
{
  for(int i=0; i < dimension_; i++)
  {
    if(fabs(joint_errors_[i]) > max_allowable_joint_errors_[i])
    {
      return false;
    }
  }
  return true;
}

void WholeBodyTrajectoryController::computeJointErrors()
{
  for(unsigned int i=0; i < joint_pv_controllers_.size(); i++)
  {
    if(joint_type_[i] == mechanism::JOINT_CONTINUOUS || joint_type_[i] == mechanism::JOINT_ROTARY)
    {
      joint_errors_[i] = angles::shortest_angular_distance(joint_cmd_rt_[i], current_joint_position_[i]);
    }
    else //prismatic
    {
      joint_errors_[i] = current_joint_position_[i] - joint_cmd_rt_[i];
    }
  }

  for(unsigned int i=0; i < base_pid_controller_.size(); i++)
  {
    int joint_index = base_joint_index_[i];
    joint_errors_[joint_index] = joint_cmd_rt_[joint_index] - current_joint_position_[joint_index];
  }
}


void WholeBodyTrajectoryController::checkWatchDog(double current_time)
{
  if(current_time - last_update_time_ < max_allowed_update_time_ && errorsWithinThreshold())
  {
    if(watch_dog_active_)
    {
      watch_dog_active_ = false;
    }
  }
  else
  {
    if(!watch_dog_active_)
    {
      watch_dog_active_ = true;
      stopMotion();
    }
  }
}

void WholeBodyTrajectoryController::stopMotion()
{
  std::vector<trajectory::Trajectory::TPoint> tp;
  int msg_size = 1;

  tp.resize(msg_size+1);

  //set first point in trajectory to current position of the arm
  tp[0].setDimension((int) dimension_);

  for(int j=0; j < dimension_; j++)
  {
    tp[0].q_[j] = current_joint_position_[j];
    tp[0].time_ = 0.0;
  }
//  ROS_WARN("Trajectory message has no way points");
  //set second point in trajectory to current position of the arm
  tp[1].setDimension((int) dimension_);

  for(int j=0; j < dimension_; j++)
  {
    tp[1].q_[j] = current_joint_position_[j];
    tp[1].time_ = 0.0;
  }
  setTrajectoryCmd(tp);
  base_controller_node_.setCommand(0.0,0.0,0.0);

}

void WholeBodyTrajectoryController::update(void)
{

#ifdef PUBLISH_MAX_TIME
  double start_time = realtime_gettime();
#endif

  double sample_time(0.0);

  current_time_ = robot_->hw_->current_time_;

  checkWatchDog(current_time_);

  if(refresh_rt_vals_)
  {
    trajectory_start_time_ = robot_->hw_->current_time_;
    trajectory_wait_time_ = 0.0;
    trajectory_end_time_ = joint_trajectory_->getTotalTime()+trajectory_start_time_;
    refresh_rt_vals_ = false;
    trajectory_done_ = false;
  }
  if(arm_controller_lock_.try_lock())
  {
    sample_time = robot_->hw_->current_time_ - trajectory_start_time_;
    joint_trajectory_->sample(trajectory_point_,sample_time);

//    ROS_INFO("sample_time: %f", sample_time);
    for(unsigned int i=0; i < joint_cmd_rt_.size(); ++i)
    {
      joint_cmd_rt_[i] = trajectory_point_.q_[i];
      joint_cmd_dot_rt_[i] = trajectory_point_.qdot_[i];
//      ROS_INFO("Cmd: %f %f",joint_cmd_rt_[i],joint_cmd_dot_rt_[i]);
    }
//    cout << endl;
    arm_controller_lock_.unlock();

    if(robot_->hw_->current_time_ >= trajectory_end_time_ && trajectory_done_ == false)
    {
      trajectory_wait_time_ = robot_->hw_->current_time_ - trajectory_end_time_;
      if(reachedGoalPosition(joint_cmd_rt_))
      {
        trajectory_wait_time_ = 0.0;
        trajectory_done_= true;
      }
    }
  }

  for(unsigned int i=0;i<joint_pv_controllers_.size();++i)
    joint_pv_controllers_[i]->setCommand(joint_cmd_rt_[i],joint_cmd_dot_rt_[i]);

  updateBaseController(current_time_);

  updateJointControllers();

#ifdef PUBLISH_MAX_TIME
  double end_time = realtime_gettime();
  max_update_time_ = std::max(max_update_time_,end_time-start_time);

  if (controller_state_publisher_->trylock())
  {
    controller_state_publisher_->msg_.update_time = end_time - start_time; 
    controller_state_publisher_->msg_.max_update_time = max_update_time_; 
    controller_state_publisher_->unlockAndPublish();      
  }
#endif
  last_time_ = current_time_;
}

void WholeBodyTrajectoryController::updateBaseController(double time)
{
  double cmd[3] = {0.0,0.0,0.0};
  double theta  = 0.0;

  if(base_pid_controller_.size() < 3)
    return;

  for(unsigned int i=0; i< base_pid_controller_.size(); i++)
  {
    int joint_index = base_joint_index_[i];
    double error(0.0), error_dot(0.0);
    if(joint_type_[joint_index] == mechanism::JOINT_CONTINUOUS)
    {
      error = angles::shortest_angular_distance(joint_cmd_rt_[joint_index], current_joint_position_[joint_index]);
    }
    else //prismatic
    {
      error = current_joint_position_[joint_index] - joint_cmd_rt_[joint_index];
    }
    error_dot = current_joint_velocity_[joint_index] - joint_cmd_dot_rt_[joint_index];      
    cmd[i] = base_pid_controller_[i].updatePid(error, time - last_time_);
    cmd[i] += joint_cmd_dot_rt_[joint_index];
    if(i == 2)
    {
      theta = current_joint_position_[joint_index];
    }
  }

  //Transform the cmd back into the base frame
  double vx = cmd[0]*cos(theta) + cmd[1]*sin(theta);
  double vy = -cmd[0]*sin(theta) + cmd[1]*cos(theta);
  double vw = cmd[2];

  if(active_)
    base_controller_node_.setCommand(vx,vy,vw);
//  base_controller_node_.setCommand(0.0,0.0,0.0);

}


bool WholeBodyTrajectoryController::reachedGoalPosition(std::vector<double> joint_cmd)
{
  bool return_val = true;
  double error(0.0);
  for(unsigned int i=0;i< joint_pv_controllers_.size();++i)
  {
    if(joint_type_[i] == mechanism::JOINT_CONTINUOUS || joint_type_[i] == mechanism::JOINT_ROTARY)
    {
      error = fabs(angles::shortest_angular_distance(joint_cmd_rt_[i], current_joint_position_[i]));
    }
    else //prismatic
    {
      error = fabs(current_joint_position_[i] - joint_cmd_rt_[i]);
    }
    return_val = return_val && (error <= goal_reached_threshold_[i]);
  }
  for(unsigned int i=0; i< base_pid_controller_.size(); i++)
  {
    int joint_index = base_joint_index_[i];
    double error(0.0);
    if(joint_type_[joint_index] == mechanism::JOINT_CONTINUOUS)
    {
      error = fabs(angles::shortest_angular_distance(joint_cmd_rt_[joint_index], current_joint_position_[joint_index]));
    }
    else //prismatic
    {
      error = fabs(current_joint_position_[joint_index] - joint_cmd_rt_[joint_index]);
    }
    return_val = return_val && (error <= goal_reached_threshold_[i]);
  }
  return return_val;
}

void WholeBodyTrajectoryController::updateJointControllers(void)
{

  for(unsigned int i=0;i<joint_pv_controllers_.size();++i)
    joint_pv_controllers_[i]->update();

  base_controller_node_.update();
}


void WholeBodyTrajectoryController::updateJointValues()
{
  // Grab the current odometric position
  double q[3], qdot[3];

  for(unsigned int i=0; i < joint_pv_controllers_.size();++i){
    current_joint_position_[i] = joint_pv_controllers_[i]->joint_state_->position_;
    current_joint_velocity_[i] = joint_pv_controllers_[i]->joint_state_->velocity_;
  }

  base_controller_node_.getOdometry(q[0], q[1], q[2], qdot[0], qdot[1], qdot[2]) ;
  for(unsigned int i=0;i< base_pid_controller_.size();++i){
    int joint_index = base_joint_index_[i];
    current_joint_position_[joint_index] = (1-alpha_filter_)*q[i] + alpha_filter_*current_joint_position_[joint_index];
    current_joint_velocity_[joint_index] = qdot[i];
  }
}

//------ Arm controller node --------

ROS_REGISTER_CONTROLLER(WholeBodyTrajectoryControllerNode)

  WholeBodyTrajectoryControllerNode::WholeBodyTrajectoryControllerNode()
    : Controller(), node_(ros::Node::instance()), request_trajectory_id_(1), current_trajectory_id_(0), trajectory_wait_timeout_(10.0)
{
  std::cout<<"Controller node created"<<endl;
  c_ = new WholeBodyTrajectoryController();
  diagnostics_publisher_ = NULL;

  c_->active_ = true;
}

WholeBodyTrajectoryControllerNode::~WholeBodyTrajectoryControllerNode()
{
  /* node_->unadvertiseService(service_prefix_ + "/set_command");
     node_->unadvertiseService(service_prefix_ + "/set_command_array");
     node_->unadvertiseService(service_prefix_ + "/get_command");
     node_->unadvertiseService(service_prefix_ + "/set_target");
  */
  node_->unadvertiseService(service_prefix_ + "/TrajectoryStart");
  node_->unadvertiseService(service_prefix_ + "/TrajectoryQuery");

  c_->controller_state_publisher_->stop();
  delete c_->controller_state_publisher_;

  diagnostics_publisher_->stop();
  delete diagnostics_publisher_;

  if(topic_name_ptr_ && topic_name_.c_str())
  {
    std::cout << "unsub arm controller" << topic_name_ << std::endl;
    node_->unsubscribe(topic_name_);
  }

  node_->unsubscribe("whole_body_trajectory_controller/activate");
  delete c_;
}

void WholeBodyTrajectoryControllerNode::update()
{

  c_->updateJointValues();

  if(c_->trajectory_done_)
  {
    updateTrajectoryQueue( WholeBodyTrajectoryControllerNode::DONE);
  }
  if(c_->trajectory_wait_time_ >= trajectory_wait_timeout_)
  {
    updateTrajectoryQueue( WholeBodyTrajectoryControllerNode::FAILED);
  }

  c_->update();

  if((c_->current_time_ - last_diagnostics_publish_time_) > diagnostics_publish_delta_time_)
  {
    publishDiagnostics();
    last_diagnostics_publish_time_ = c_->current_time_;
  }
}

void WholeBodyTrajectoryControllerNode::activate()
{
  c_->active_ = true;
  c_->stopMotion(); //sets all the desired joint values to their current values - prevents PID gains from building up on activation
}

void WholeBodyTrajectoryControllerNode::updateTrajectoryQueue(int last_trajectory_finish_status)
{
  if(joint_trajectory_vector_.size() > 0)
  {
    if(current_trajectory_id_ == joint_trajectory_id_.front())
    {
      joint_trajectory_status_[current_trajectory_id_] = last_trajectory_finish_status;
      joint_trajectory_time_[current_trajectory_id_] = c_->trajectory_end_time_ - c_->trajectory_start_time_;
      joint_trajectory_vector_.erase(joint_trajectory_vector_.begin());
      joint_trajectory_id_.erase(joint_trajectory_id_.begin());
    }
    if(joint_trajectory_vector_.size() > 0)
    {
      setTrajectoryCmdFromMsg(joint_trajectory_vector_.front());
      current_trajectory_id_ = joint_trajectory_id_.front();
      joint_trajectory_status_[current_trajectory_id_] = WholeBodyTrajectoryControllerNode::ACTIVE;
    }
    else
    {
      current_trajectory_id_ = 0;
    }
  }
}


bool WholeBodyTrajectoryControllerNode::initXml(mechanism::RobotState * robot, TiXmlElement * config)
{
  ROS_INFO("Loading WholeBodyTrajectoryControllerNode.");
  service_prefix_ = config->Attribute("name");
  ROS_INFO("The service_prefix_ is %s",service_prefix_.c_str());

  double scale;
  node_->param<double>(service_prefix_ + "/velocity_scaling_factor",scale,0.25);
  node_->param<double>(service_prefix_ + "/trajectory_wait_timeout",trajectory_wait_timeout_,10.0);
  node_->param<double>(service_prefix_ + "/trajectory_update_timeout",c_->max_allowed_update_time_,0.2);

  c_->velocity_scaling_factor_ = std::min(1.0,std::max(0.0,scale));

  if(c_->initXml(robot, config))  // Parses subcontroller configuration
  {
/*    node_->advertiseService(service_prefix_ + "/set_command", &WholeBodyTrajectoryControllerNode::setJointPosHeadless, this);
      node_->advertiseService(service_prefix_ + "/set_command_array", &WholeBodyTrajectoryControllerNode::setJointPosSrv, this);
      node_->advertiseService(service_prefix_ + "/get_command", &WholeBodyTrajectoryControllerNode::getJointPosCmd, this);
      node_->advertiseService(service_prefix_ + "/set_target", &WholeBodyTrajectoryControllerNode::setJointPosTarget, this);
*/

    node_->advertiseService(service_prefix_ + "/TrajectoryStart", &WholeBodyTrajectoryControllerNode::setJointTrajSrv, this);
    node_->advertiseService(service_prefix_ + "/TrajectoryQuery", &WholeBodyTrajectoryControllerNode::queryJointTrajSrv, this);
   node_->advertiseService(service_prefix_ + "/TrajectoryCancel", &WholeBodyTrajectoryControllerNode::cancelJointTrajSrv, this);

    topic_name_ptr_ = config->FirstChildElement("listen_topic");
    if(topic_name_ptr_)
    {
      topic_name_= topic_name_ptr_->Attribute("name");
      if(!topic_name_.c_str())
      {
        std::cout<<" A listen _topic is present in the xml file but no name is specified\n";
        return false;
      }
      node_->subscribe(topic_name_, traj_msg_, &WholeBodyTrajectoryControllerNode::CmdTrajectoryReceived, this, 1);
      ROS_INFO("Listening to topic: %s",topic_name_.c_str());
    }

    node_->subscribe("whole_body_trajectory_controller/activate",activate_msg_,&WholeBodyTrajectoryControllerNode::activate,this,1);

    getJointTrajectoryThresholds();

    if (c_->controller_state_publisher_ != NULL)// Make sure that we don't memory leak if initXml gets called twice
      delete c_->controller_state_publisher_ ;
    c_->controller_state_publisher_ = new realtime_tools::RealtimePublisher <robot_msgs::ControllerState> (service_prefix_+"/controller_state", 1) ;

  if (diagnostics_publisher_ != NULL)// Make sure that we don't memory leak if initXml gets called twice
    delete diagnostics_publisher_ ;
  diagnostics_publisher_ = new realtime_tools::RealtimePublisher <robot_msgs::DiagnosticMessage> ("/diagnostics", 2) ;

  last_diagnostics_publish_time_ = c_->robot_->hw_->current_time_;
  node_->param<double>(service_prefix_ + "/diagnostics_publish_delta_time",diagnostics_publish_delta_time_,0.05);

    ROS_INFO("Initialize publisher");

    c_->controller_state_publisher_->msg_.name = std::string(service_prefix_); 

    ROS_INFO("Initialized controller");
    return true;
  }
  ROS_INFO("Could not initialize controller");
  return false;
}

void WholeBodyTrajectoryControllerNode::getJointTrajectoryThresholds()
{
  c_->goal_reached_threshold_.resize(c_->dimension_);
  c_->max_allowable_joint_errors_.resize(c_->dimension_);
  for(int i=0; i< c_->dimension_;i++)
  {
    node_->param<double>(service_prefix_ + "/" + c_->joint_name_[i] + "/goal_reached_threshold",c_->goal_reached_threshold_[i],GOAL_REACHED_THRESHOLD);
    node_->param<double>(service_prefix_ + "/" + c_->joint_name_[i] + "/joint_error_threshold",c_->max_allowable_joint_errors_[i],MAX_ALLOWABLE_JOINT_ERROR_THRESHOLD);
  }
}

void WholeBodyTrajectoryControllerNode::setTrajectoryCmdFromMsg(robot_msgs::JointTraj traj_msg)
{
  std::vector<trajectory::Trajectory::TPoint> tp;
  int msg_size = std::max<int>((int)traj_msg.get_points_size(),1);

  tp.resize(msg_size+1);

  //set first point in trajectory to current position of the arm
  tp[0].setDimension((int) c_->dimension_);

  for(int j=0; j < c_->dimension_; j++)
  {
    tp[0].q_[j] = c_->current_joint_position_[j];
    tp[0].time_ = 0.0;
  }

  if((int)traj_msg.get_points_size() > 0)
  {
    if((int) traj_msg.points[0].get_positions_size() != (int) c_->dimension_)
    {
      ROS_WARN("Dimension of input trajectory = %d does not match number of controlled joints = %d",(int) traj_msg.points[0].get_positions_size(), (int) c_->dimension_);
      return;
    }
    else
    {
      for(int i=0; i < (int) traj_msg.get_points_size(); i++)
      {
        tp[i+1].setDimension((int) c_->dimension_);
        for(int j=0; j < (int) c_->dimension_; j++)
        {
          tp[i+1].q_[j] = traj_msg.points[i].positions[j];
          tp[i+1].time_ = traj_msg.points[i].time;
        }
      }
    }
  }
  else
  {
    ROS_WARN("Trajectory message in command has no way points");
    //set second point in trajectory to current position of the arm
    tp[1].setDimension((int) c_->dimension_);

    for(int j=0; j < c_->dimension_; j++)
    {
      tp[1].q_[j] = c_->current_joint_position_[j];
      tp[1].time_ = 0.0;
    }
  }

  this->c_->setTrajectoryCmd(tp);
}

void WholeBodyTrajectoryControllerNode::CmdTrajectoryReceived()
{

  this->ros_lock_.lock();
  c_->last_update_time_ = c_->current_time_;
  setTrajectoryCmdFromMsg(traj_msg_);
  this->ros_lock_.unlock();
}


bool WholeBodyTrajectoryControllerNode::getJointPosCmd(pr2_mechanism_controllers::GetJointPosCmd::Request &req,
                                                 pr2_mechanism_controllers::GetJointPosCmd::Response &resp)
{
  pr2_mechanism_controllers::JointPosCmd cmd;
  c_->getJointPosCmd(cmd);
  resp.command = cmd;
  return true;
}


bool WholeBodyTrajectoryControllerNode::setJointTrajSrv(pr2_mechanism_controllers::TrajectoryStart::Request &req,
                                                  pr2_mechanism_controllers::TrajectoryStart::Response &resp)
{
  addTrajectoryToQueue(req.traj, request_trajectory_id_);
  resp.trajectoryid = request_trajectory_id_;

  if(req.requesttiming)
  {
    trajectory::Trajectory tmp(c_->dimension_);
    createTrajectory(req.traj,tmp);
    std::vector<double> timestamps;
    resp.set_timestamps_size((int)req.traj.get_points_size());
    timestamps.resize((int)req.traj.get_points_size());

    tmp.getTimeStamps(timestamps);

    for(int i=0; i < (int) req.traj.get_points_size(); i++)
    {
      resp.timestamps[i] = timestamps[i];
    }
  }

  request_trajectory_id_++;
  return true;
}

bool WholeBodyTrajectoryControllerNode::queryJointTrajSrv(pr2_mechanism_controllers::TrajectoryQuery::Request &req,
                                                    pr2_mechanism_controllers::TrajectoryQuery::Response &resp)
{
  resp.set_jointnames_size(c_->dimension_);
  resp.set_jointpositions_size(c_->dimension_);
  for(int i=0; i < c_->dimension_; i++)
  {
    resp.jointnames[i] = c_->joint_name_[i];
    resp.jointpositions[i] = c_->current_joint_position_[i];
  }

  if(req.trajectoryid == 0)
  {
    resp.trajectorytime = 0;
    if(joint_trajectory_vector_.size() == 0)
      resp.done = 1;
    else
      resp.done = 0;
    return true;
  }

  std::map<int, int>::const_iterator it = joint_trajectory_status_.find((int)req.trajectoryid);
  if(it == joint_trajectory_status_.end())
    return false;
  else
    resp.done = it->second;


  if(current_trajectory_id_ == (int)req.trajectoryid)
  {
    if((int) resp.done == 1)
      resp.trajectorytime = c_->trajectory_end_time_ - c_->trajectory_start_time_;
    else
      resp.trajectorytime = c_->current_time_ - c_->trajectory_start_time_;
  }
  else
  {
    std::map<int, double>::const_iterator it_time = joint_trajectory_time_.find((int)req.trajectoryid);
    if(it_time == joint_trajectory_time_.end())
      return false;

    resp.trajectorytime = it_time->second;
  }
  return true;
}

bool WholeBodyTrajectoryControllerNode::cancelJointTrajSrv(pr2_mechanism_controllers::TrajectoryCancel::Request &req,
                                                     pr2_mechanism_controllers::TrajectoryCancel::Response &resp)
{
  int status = WholeBodyTrajectoryControllerNode::NUM_STATUS;

  std::vector<trajectory::Trajectory::TPoint> trajectory_points_vector;
  std::map<int, int>::const_iterator it = joint_trajectory_status_.find((int)req.trajectoryid);

  if(it == joint_trajectory_status_.end())
    return false;
  else
    status = it->second;

  if(status == WholeBodyTrajectoryControllerNode::QUEUED)
  {
    deleteTrajectoryFromQueue(req.trajectoryid);
  }
  else if(status == WholeBodyTrajectoryControllerNode::ACTIVE)
  {
    updateTrajectoryQueue(CANCELED);
    // Add two points since every good trajectory must have at least two points, otherwise its just a point :-)
    for(int j=0; j < c_->dimension_; j++)
      c_->trajectory_point_.q_[j] = c_->current_joint_position_[j];

    c_->trajectory_point_.time_ = 0.0;
    trajectory_points_vector.push_back(c_->trajectory_point_);

    for(int i=0; i < c_->dimension_; i++)
      c_->trajectory_point_.q_[i] =  c_->current_joint_position_[i];
    c_->trajectory_point_.time_ = 0.0;
    trajectory_points_vector.push_back(c_->trajectory_point_);
    if(!c_->joint_trajectory_->setTrajectory(trajectory_points_vector))
      ROS_WARN("Trajectory not set correctly");
  }
  return true;
}

int WholeBodyTrajectoryControllerNode::createTrajectory(const robot_msgs::JointTraj &new_traj,trajectory::Trajectory &return_trajectory)
{
  std::vector<trajectory::Trajectory::TPoint> tp;

  if(new_traj.get_points_size() > 0)
  {
    if((int) new_traj.points[0].get_positions_size() != (int) c_->dimension_)
    {
      ROS_WARN("Dimension of input trajectory = %d does not match number of controlled joints = %d",(int) new_traj.points[0].get_positions_size(), (int) c_->dimension_);
      return -1;
    }
  }
  else
  {
    ROS_WARN("Trajectory message has no way points");
    return -1;
  }

  tp.resize((int)new_traj.get_points_size());

  for(int i=0; i < (int) new_traj.get_points_size(); i++)
  {
    tp[i].setDimension((int) c_->dimension_);
    for(int j=0; j < (int) c_->dimension_; j++)
    {
      tp[i].q_[j] = new_traj.points[i].positions[j];
      tp[i].time_ = new_traj.points[i].time;
    }
  }
  return_trajectory.setMaxRates(c_->joint_velocity_limits_);
  return_trajectory.setInterpolationMethod(c_->trajectory_type_);

  if(!return_trajectory.setTrajectory(tp))
  {
    ROS_WARN("Trajectory not set correctly");
    return -1;
  }
  return 1;
}

void WholeBodyTrajectoryControllerNode::addTrajectoryToQueue(robot_msgs::JointTraj new_traj, int id)
{
  joint_trajectory_vector_.push_back(new_traj);
  joint_trajectory_id_.push_back(id);
  joint_trajectory_status_[id] = WholeBodyTrajectoryControllerNode::QUEUED;
  joint_trajectory_time_[id] = 0.0;
}

void WholeBodyTrajectoryControllerNode::deleteTrajectoryFromQueue(int id)
{
// do a linear search
  for(int i = 0; i < (int) joint_trajectory_vector_.size(); i++)
  {
    if(joint_trajectory_id_[i] == id)
    {
      joint_trajectory_vector_.erase(joint_trajectory_vector_.begin()+i);
      joint_trajectory_id_.erase(joint_trajectory_id_.begin()+i);
      joint_trajectory_status_[id] = WholeBodyTrajectoryControllerNode::DELETED;
      break;
    }
  }
}


void WholeBodyTrajectoryControllerNode::publishDiagnostics()
{
//  ROS_INFO("Starting diagnostics");
  if(diagnostics_publisher_->trylock())
  {
//    ROS_INFO("Started diagnostics");
    robot_msgs::JointCmd cmd;
    cmd.set_names_size(1);
    cmd.set_positions_size(1);
    cmd.set_velocity_size(1);

    vector<robot_msgs::DiagnosticStatus> statuses;
    vector<robot_msgs::DiagnosticValue> values;
    vector<robot_msgs::DiagnosticString> strings;
    robot_msgs::DiagnosticStatus status;
    robot_msgs::DiagnosticValue v;

    status.name = "Whole Body Trajectory Controller";
    status.level = 0;
    status.message = "OK";

//    ROS_INFO("Diagnostics 1");

    for(unsigned int i=0; i < c_->joint_pv_controllers_.size(); i++)
    {
      v.label = c_->joint_pv_controllers_[i]->getJointName() + "/Position/Actual";
      v.value = c_->joint_pv_controllers_[i]->joint_state_->position_;
      values.push_back(v);

//      ROS_INFO("Diagnostics %d: 1",i);

      v.label = c_->joint_pv_controllers_[i]->getJointName() + "/Position/Command";
      c_->joint_pv_controllers_[i]->getCommand(cmd);
      v.value = cmd.positions[0];
      values.push_back(v);

      v.label = c_->joint_pv_controllers_[i]->getJointName() + "/Position/Error (Command-Actual)";
      v.value = cmd.positions[0] - c_->joint_pv_controllers_[i]->joint_state_->position_;
      values.push_back(v);

//      ROS_INFO("Diagnostics %d: 2",i);

    }


    for(unsigned int i=0; i < c_->base_pid_controller_.size(); i++)
    {
      int joint_index = c_->base_joint_index_[i];
      v.label = c_->joint_name_[joint_index] + "/Position/Actual";
      v.value = c_->current_joint_position_[joint_index];
      values.push_back(v);

//      ROS_INFO("Diagnostics %d: 1",i);

      v.label = c_->joint_name_[joint_index] + "/Position/Command";
      v.value = c_->joint_cmd_rt_[joint_index];
      values.push_back(v);

      v.label = c_->joint_name_[joint_index] + "/Position/Error (Command-Actual)";
      v.value = c_->joint_cmd_rt_[joint_index] - c_->current_joint_position_[joint_index];
      values.push_back(v);

//      ROS_INFO("Diagnostics %d: 2",i);
    }

    v.label = "Trajectory id";
    v.value = current_trajectory_id_;
    values.push_back(v);

//    ROS_INFO("Diagnostics 2");

    v.label = "Trajectory Status:: ";
    std::map<int, int>::const_iterator it = joint_trajectory_status_.find((int)current_trajectory_id_);
    if(it == joint_trajectory_status_.end())
    {
      v.label += "UNKNOWN";
      v.value = -1;
    }
    else
    {
      v.label += JointTrajectoryStatusString[it->second];
      v.value = it->second;
    }
    values.push_back(v);

//    ROS_INFO("Diagnostics 3");

    v.label = "Trajectory Current Time";
    v.value = c_->current_time_-c_->trajectory_start_time_;
    values.push_back(v);

//    ROS_INFO("Diagnostics 4");

    v.label = "Trajectory Expected End Time (computed)";
    v.value = c_->trajectory_end_time_-c_->trajectory_start_time_;
    values.push_back(v);

//    ROS_INFO("Diagnostics 5");

    status.set_values_vec(values);
    status.set_strings_vec(strings);
    statuses.push_back(status);
    diagnostics_publisher_->msg_.set_status_vec(statuses);
//    ROS_INFO("Set diagnostics info");
    diagnostics_publisher_->unlockAndPublish();
  }
}
