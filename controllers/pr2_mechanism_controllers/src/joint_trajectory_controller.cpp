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

#include "pr2_mechanism_controllers/joint_trajectory_controller.h"

using namespace controller;
using namespace std;

ROS_REGISTER_CONTROLLER(JointTrajectoryController);

JointTrajectoryController::JointTrajectoryController() : Controller(), node_(ros::Node::instance())
{
  controller_state_publisher_ = NULL;
  num_joints_ = 0;
  trajectory_type_ = "linear";
  trajectory_wait_time_= 0.0;
  max_update_time_= 0.0;
  watch_dog_active_ = false;
  request_trajectory_id_ = 0; 
  current_trajectory_id_ = -1; 
  trajectory_wait_timeout_ = 10.0;
  diagnostics_publisher_ = NULL;

  num_trajectory_available_ = 0;
  next_free_index_ = 0;
  current_trajectory_index_ = 0;
  trajectory_preempted_ = false;
}


JointTrajectoryController::~JointTrajectoryController()
{
  for(unsigned int i=0; i<joint_pv_controllers_.size();++i)
    delete joint_pv_controllers_[i];


  stopPublishers();
  unadvertiseServices();
  unsubscribeTopics();

  delete joint_trajectory_;
}

bool JointTrajectoryController::initXml(mechanism::RobotState * robot, TiXmlElement * config)
{
  name_ = config->Attribute("name");
  prefix_ = name_ + "/";

  getParams();

  loadXmlFile(robot,config);

  joint_trajectory_vector_.resize(max_trajectory_queue_size_);
  joint_trajectory_id_.resize(max_trajectory_queue_size_);

  subscribeTopics();

  advertiseServices();

  initializePublishers();


  ROS_INFO("Loaded JointTrajectoryController: %s",name_.c_str());
  return true;
}

bool JointTrajectoryController::starting()
{
  current_time_ = robot_->hw_->current_time_;
  at_rest_ = true;
  updateJointValues();
  last_time_ = current_time_;
  setTrajectoryCmdToCurrentValues();
  return true;
}

void JointTrajectoryController::getParams()
{
  double scale;
  node_->param<double>(prefix_+"velocity_scaling_factor",scale,0.25);
  node_->param<double>(prefix_+"trajectory_wait_timeout",trajectory_wait_timeout_,10.0);
  node_->param<double>(prefix_+"trajectory_update_timeout",max_allowed_update_time_,0.2);
  node_->param<std::string>(prefix_+"listen_topic_name",listen_topic_name_,"command");
  node_->param<double>(prefix_+"at_rest_velocity_threshold_",at_rest_velocity_threshold_,1e-5);
  node_->param<double>(prefix_+"max_allowed_update_time_",max_allowed_update_time_,100.0);
  node_->param<double>(prefix_+"diagnostics_publish_delta_time",diagnostics_publish_delta_time_,1.0);
  node_->param<int>(prefix_+"max_trajectory_queue_size",max_trajectory_queue_size_,100);
  velocity_scaling_factor_ = std::min(1.0,std::max(0.0,scale));

}

void JointTrajectoryController::initializePublishers()
{
  if (controller_state_publisher_ != NULL)// Make sure that we don't memory leak if initXml gets called twice
    delete controller_state_publisher_ ;
  controller_state_publisher_ = new realtime_tools::RealtimePublisher <robot_msgs::ControllerState> (prefix_+"controller_state", 1) ;

  if (diagnostics_publisher_ != NULL)// Make sure that we don't memory leak if initXml gets called twice
    delete diagnostics_publisher_ ;
  diagnostics_publisher_ = new realtime_tools::RealtimePublisher <robot_msgs::DiagnosticMessage> ("/diagnostics", 2) ;

  last_diagnostics_publish_time_ = robot_->hw_->current_time_;
  node_->param<double>(prefix_+"diagnostics_publish_delta_time",diagnostics_publish_delta_time_,0.05);

  controller_state_publisher_->msg_.name = name_; 
  ROS_INFO("Initialized publishers.");
}

void JointTrajectoryController::stopPublishers()
{
  controller_state_publisher_->stop();
  diagnostics_publisher_->stop();

  delete controller_state_publisher_;
  delete diagnostics_publisher_;
}

void JointTrajectoryController::advertiseServices()
{
  node_->advertiseService(prefix_+"TrajectoryStart", &JointTrajectoryController::setJointTrajSrv, this);
  node_->advertiseService(prefix_+"TrajectoryQuery", &JointTrajectoryController::queryJointTrajSrv, this);
  node_->advertiseService(prefix_+"TrajectoryCancel", &JointTrajectoryController::cancelJointTrajSrv, this);

  ROS_INFO("Service for setting trajectories : %sTrajectoryStart",prefix_.c_str());
  ROS_INFO("Service for querying trajectories : %sTrajectoryQuery",prefix_.c_str());
  ROS_INFO("Service for canceling trajectories : %sTrajectoryCancel",prefix_.c_str());
}

void JointTrajectoryController::unadvertiseServices()
{
  node_->unadvertiseService(prefix_+"TrajectoryStart");
  node_->unadvertiseService(prefix_+"TrajectoryQuery");
  node_->unadvertiseService(prefix_+"TrajectoryCancel");
}

void JointTrajectoryController::subscribeTopics()
{
  node_->subscribe(prefix_+listen_topic_name_, traj_msg_, &JointTrajectoryController::TrajectoryReceivedOnTopic, this, 1);
  ROS_INFO("Listening to topic: %s%s",prefix_.c_str(),listen_topic_name_.c_str());
}

void JointTrajectoryController::unsubscribeTopics()
{
  node_->unsubscribe(prefix_ + listen_topic_name_);
}

bool JointTrajectoryController::loadXmlFile(mechanism::RobotState * robot, TiXmlElement * config)
{
  ROS_DEBUG("Loading joint trajectory controller from XML.");

  robot_ = robot->model_;
  robot_state_ = robot;
  current_time_ = robot_->hw_->current_time_;
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

      addJoint(jpc->getJointName());
    }
    else if(static_cast<std::string>(elt->Attribute("type")) == std::string("BasePIDController"))
    {
      addRobotBaseJoint(elt);
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

  initTrajectory(config);

  joint_cmd_.resize(num_joints_);
  joint_cmd_dot_.resize(num_joints_);

  last_time_ = current_time_;
  last_traj_req_time_ = current_time_;

  goal_reached_threshold_.resize(num_joints_);
  max_allowable_joint_errors_.resize(num_joints_);
  for(int i=0; i< num_joints_;i++)
  {
    node_->param<double>(prefix_+ joint_name_[i] + "/goal_reached_threshold",goal_reached_threshold_[i],GOAL_REACHED_THRESHOLD);
    node_->param<double>(prefix_+ joint_name_[i] + "/joint_error_threshold",max_allowable_joint_errors_[i],MAX_ALLOWABLE_JOINT_ERROR_THRESHOLD);
  }

  ROS_INFO("Initialized joint trajectory controller");

  return true;
}


void JointTrajectoryController::initTrajectory(TiXmlElement * config)
{
  TiXmlElement *elt = config->FirstChildElement("trajectory");
  if(!elt)
    ROS_WARN("No trajectory information in xml file. ");
  else
  {
    trajectory_type_ = std::string(elt->Attribute("interpolation"));
    ROS_INFO("JointTrajectoryController:: interpolation type:: %s",trajectory_type_.c_str());
  }

  joint_trajectory_ = new trajectory::Trajectory((int) num_joints_);
  joint_trajectory_->setMaxRates(joint_velocity_limits_);
  joint_trajectory_->setInterpolationMethod(trajectory_type_);

  for(int i=0; i<num_joints_; i++)
  {
    if(joint_type_[i] == mechanism::JOINT_CONTINUOUS)
    {
      ROS_INFO("Setting joint %d to wrap",i);      
      joint_trajectory_->setJointWraps(i);
    }
  }

  trajectory_start_time_ = current_time_;
  trajectory_end_time_ = current_time_;
  current_trajectory_finished_ = true;
  joint_trajectory_->autocalc_timing_ = true;

  current_joint_position_vector_.resize(2);
  current_joint_position_vector_[0].setDimension(num_joints_);
  current_joint_position_vector_[1].setDimension(num_joints_);

  trajectory_point_.setDimension(num_joints_);

}

void JointTrajectoryController::setTrajectoryCmdToCurrentValues()
{
  for(int i=0; i < 2; i++)
  {
    for(int j=0; j < num_joints_; j++)
    {
      current_joint_position_vector_[i].q_[j] = current_joint_position_[j];
    }
    current_joint_position_vector_[i].time_ = 0.0;    
  }
//  ROS_DEBUG("Size of trajectory points vector : %d",current_joint_position_vector_.size());

  if(setTrajectoryCmd(current_joint_position_vector_))
  {
    current_trajectory_id_ = -1;
  }
}


void JointTrajectoryController::addJoint(const std::string &name)
{
  mechanism::Joint *joint;
  joint = robot_state_->getJointState(name)->joint_;
  if(!joint)
  {
    ROS_ERROR("Joint %s not found in class robot",name.c_str());
    return;
  }

  joint_velocity_limits_.push_back(joint->velocity_limit_*velocity_scaling_factor_);
  joint_type_.push_back(joint->type_);
  joint_name_.push_back(name);

  current_joint_position_.push_back(0.0);
  current_joint_velocity_.push_back(0.0);

  joint_position_errors_.push_back(0.0);
  joint_velocity_errors_.push_back(0.0);
  num_joints_++;
}

void JointTrajectoryController::addRobotBaseJoint(TiXmlElement *elt)
{
  TiXmlElement *pj = elt->FirstChildElement("joint");
  if(pj)
  {
    std::string joint_name = static_cast<std::string>(pj->Attribute("name"));
    double velocity_limit = atof(pj->Attribute("limit"));

    if(static_cast<std::string>(pj->Attribute("type")) == std::string("ROTARY_CONTINUOUS"))
    {
      ROS_INFO("Adding joint %s with type: ROTARY_CONTINUOUS and velocity limit: %f",joint_name.c_str(),velocity_limit);
      joint_type_.push_back(mechanism::JOINT_CONTINUOUS);
      base_theta_index_ = num_joints_;
    }
    else if(static_cast<std::string>(pj->Attribute("type")) == std::string("PRISMATIC_CONTINUOUS"))
    {
      ROS_INFO("Adding joint %s with type: PRISMATIC_CONTINUOUS and velocity limit: %f",joint_name.c_str(),velocity_limit);
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
    {
      ROS_ERROR("BasePIDController's config did not specify the default pid parameters.\n");
    }
    base_pid_controller_.push_back(pid);
      

    joint_name_.push_back(joint_name);        
    joint_velocity_limits_.push_back(velocity_limit*velocity_scaling_factor_);

    current_joint_position_.push_back(0.0);
    current_joint_velocity_.push_back(0.0);
    joint_position_errors_.push_back(0.0);
    joint_velocity_errors_.push_back(0.0);

    base_joint_index_.push_back(num_joints_);
    num_joints_++;
  }
}

bool JointTrajectoryController::setTrajectoryCmd(const std::vector<trajectory::Trajectory::TPoint>& joint_trajectory)
{
  if(joint_trajectory.size() < 1)
  {
    ROS_WARN("JointTrajectoryController:: No points in trajectory");
    return false;
  }
  joint_trajectory_->setTrajectory(joint_trajectory);

  return true;
}

bool JointTrajectoryController::errorsWithinThreshold()
{
  for(int i=0; i < num_joints_; i++)
  {
    if(fabs(joint_position_errors_[i]) > max_allowable_joint_errors_[i])
    {
      return false;
    }
  }
  return true;
}

bool JointTrajectoryController::atRest()
{
  for(int i=0; i < num_joints_; i++)
  {
    if(fabs(current_joint_velocity_[i]) > at_rest_velocity_threshold_)
    {
      return false;
    }
  }
  return true;
}

void JointTrajectoryController::computeJointErrors()
{
  for(unsigned int i=0; i < (unsigned int) num_joints_; i++)
  {
    if(joint_type_[i] == mechanism::JOINT_CONTINUOUS)
    {
      joint_position_errors_[i] = angles::shortest_angular_distance(joint_cmd_[i], current_joint_position_[i]);
    }
    else if(joint_type_[i] == mechanism::JOINT_ROTARY)
    {
      joint_position_errors_[i] = angles::shortest_angular_distance(joint_cmd_[i], current_joint_position_[i]);
    }
    else
    {
      joint_position_errors_[i] = current_joint_position_[i] - joint_cmd_[i];
    }
    joint_velocity_errors_[i] = current_joint_velocity_[i] - joint_cmd_dot_[i];
  }
}

bool JointTrajectoryController::checkWatchDog(double current_time)
{
  if(current_time - last_traj_req_time_ < max_allowed_update_time_ && errorsWithinThreshold())
  {
    if(watch_dog_active_)
    {
      watch_dog_active_ = false;
      current_trajectory_finished_ = true;
      return false;
    }
    return false;
  }
  else
  {
    return true;
  }
}

void JointTrajectoryController::stopMotion()
{
  setTrajectoryCmdToCurrentValues();
  base_controller_node_.setCommand(0.0,0.0,0.0);
}

void JointTrajectoryController::resetTrajectoryTimes()
{
  trajectory_start_time_ = current_time_;
  trajectory_wait_time_ = 0.0;
  trajectory_end_time_ = joint_trajectory_->getTotalTime()+trajectory_start_time_;
//  ROS_INFO("Resetting trajectory time");
//  ROS_INFO("Trajectory time: %f",joint_trajectory_->getTotalTime());
}

void JointTrajectoryController::getJointCommands()
{
  double sample_time(0.0);
  sample_time = current_time_ - trajectory_start_time_;
  joint_trajectory_->sample(trajectory_point_,sample_time);

  for(int i=0; i < num_joints_; ++i)
  {
    joint_cmd_[i] = trajectory_point_.q_[i];
    joint_cmd_dot_[i] = trajectory_point_.qdot_[i];
  }
}

bool JointTrajectoryController::trajectoryDone()
{
  if(current_time_ >= trajectory_end_time_ && current_trajectory_finished_ == false)
  {
    trajectory_wait_time_ = current_time_ - trajectory_end_time_;
    if(reachedGoalPosition(joint_cmd_))
    {
      trajectory_wait_time_ = 0.0;
      updateTrajectoryQueue(current_trajectory_id_,JointTrajectoryController::DONE);
      return true;
    }
    if(trajectory_wait_time_ >= trajectory_wait_timeout_)
    {
      trajectory_wait_time_ = 0.0;
      updateTrajectoryQueue(current_trajectory_id_,JointTrajectoryController::FAILED);
      return true;
    }
  }
  return false;
}

void JointTrajectoryController::update(void)
{
#ifdef PUBLISH_MAX_TIME
  double start_time = realtime_gettime();
#endif
  current_time_ = robot_->hw_->current_time_;
  updateJointValues();
  int get_trajectory_index = 0;

/** Actual decision making done here
    (a) check if watch dog is activated, if it is preempt current trajectory and set command to current desired values,  set current_trajectory_finished to true
    (b) check if current trajectory was preempted, if it is, set status to CANCELED, set current_trajectory_finished to true
    (c) check if current trajectory is done, if it is, set its status to DONE or FAILED if timeout was exceeded, set current_trajectory_finished to true
    THEN
    (d) if current trajectory is finished, get a new trajectory from the queue, if no trajectory is available, set current desired poisitions to current joint positions 
        effectively bringing the robot to rest.
**/

  if(checkWatchDog(current_time_))
  {
    if(!watch_dog_active_)
    {
      watch_dog_active_ = true;
      updateTrajectoryQueue(current_trajectory_id_,JointTrajectoryController::CANCELED);
      setTrajectoryCmdToCurrentValues();
      at_rest_ = true;
      current_trajectory_id_ = -1;
      current_trajectory_finished_ = false;
    }
  }
  else if(trajectory_preempted_)
  {
    updateTrajectoryQueue(current_trajectory_id_,JointTrajectoryController::CANCELED);
    get_trajectory_index = current_trajectory_index_;
    trajectory_preempted_ = false;
    current_trajectory_finished_ = true;
  }
  else if(trajectoryDone())
  {
    get_trajectory_index = (current_trajectory_index_ + 1)%max_trajectory_queue_size_;
    current_trajectory_finished_ = true;    
  }

  if(current_trajectory_finished_)
  {
    if(getTrajectoryFromQueue(get_trajectory_index))
    {
//      ROS_INFO("Updated to trajectory with index: %d",get_trajectory_index);
      current_trajectory_index_ = get_trajectory_index;
      at_rest_ = false;
      current_trajectory_finished_ = false;
      resetTrajectoryTimes();
    }
    else if(!at_rest_)
    {
      at_rest_ = true;
      current_trajectory_id_ = -1;
      setTrajectoryCmdToCurrentValues();
    }
  }

  getJointCommands();

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

  publishDiagnostics();

  last_time_ = current_time_;
}

void JointTrajectoryController::updateBaseCommand(double time)
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
      error = angles::shortest_angular_distance(joint_cmd_[joint_index], current_joint_position_[joint_index]);
    }
    else //prismatic
    {
      error = current_joint_position_[joint_index] - joint_cmd_[joint_index];
    }
    error_dot = current_joint_velocity_[joint_index] - joint_cmd_dot_[joint_index];      
    cmd[i] = base_pid_controller_[i].updatePid(error, time - last_time_);
    cmd[i] += joint_cmd_dot_[joint_index];
    if(joint_index == base_theta_index_)
    {
      theta = current_joint_position_[joint_index];
    }
//    ROS_INFO("Joint cmd: %d %f %f %f",i,error,error_dot,cmd[i]);
  }

  //Transform the cmd back into the base frame
  double vx = cmd[0]*cos(theta) + cmd[1]*sin(theta);
  double vy = -cmd[0]*sin(theta) + cmd[1]*cos(theta);
  double vw = cmd[2];

  base_controller_node_.setCommand(vx,vy,vw);

}

bool JointTrajectoryController::reachedGoalPosition(std::vector<double> joint_cmd)
{
  bool return_val = true;
  double error(0.0);
  for(int i=0;i < num_joints_;++i)
  {
    if(joint_type_[i] == mechanism::JOINT_CONTINUOUS || joint_type_[i] == mechanism::JOINT_ROTARY)
    {
      error = fabs(angles::shortest_angular_distance(joint_cmd_[i], current_joint_position_[i]));
    }
    else //prismatic
    {
      error = fabs(current_joint_position_[i] - joint_cmd_[i]);
    }
    return_val = return_val && (error <= goal_reached_threshold_[i]);
  }
  return return_val;
}

void JointTrajectoryController::updateJointControllers(void)
{
  //compute errors
  computeJointErrors();

  // Set the commands for all the joints
  for(unsigned int i=0;i<joint_pv_controllers_.size();++i)
  {
    joint_pv_controllers_[i]->setCommand(joint_cmd_[i],joint_cmd_dot_[i]);
  }

  // Set the commands for the base
  updateBaseCommand(current_time_);

  // Call update on all the controllers
  for(unsigned int i=0;i<joint_pv_controllers_.size();++i)
    joint_pv_controllers_[i]->update();  

  base_controller_node_.update();
}

void JointTrajectoryController::updateJointValues()
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
    current_joint_position_[joint_index] = q[i];
    current_joint_velocity_[joint_index] = qdot[i];
  }
}

void JointTrajectoryController::updateTrajectoryQueue(int id, int finish_status)
{
  if(num_trajectory_available_ > 0 && id >= 0)
  {
    joint_trajectory_status_[id] = finish_status;
    joint_trajectory_time_[id] = trajectory_end_time_ - trajectory_start_time_;
    if(finish_status != JointTrajectoryController::ACTIVE)
      num_trajectory_available_--;
  }
}

bool JointTrajectoryController::getTrajectoryFromQueue(int &index)
{
  if(num_trajectory_available_ < 1)
  {
//    ROS_INFO("No trajectories available");
    return false;
  }
  if(joint_trajectory_status_[joint_trajectory_id_[index]] == JointTrajectoryController::QUEUED)
  {
    setTrajectoryCmdFromMsg(joint_trajectory_vector_[index],joint_trajectory_id_[index]);
    return true;
  }
  else
  {
// do a linear search from current index + 1
    int iter = (index + 1)%max_trajectory_queue_size_;
    int num_iterations = 0;
    while(num_iterations < max_trajectory_queue_size_)
    { 
      if(joint_trajectory_status_[joint_trajectory_id_[iter]] == JointTrajectoryController::QUEUED)
      {
        setTrajectoryCmdFromMsg(joint_trajectory_vector_[iter],joint_trajectory_id_[iter]);
        index = iter;
        return true;
      }
      num_iterations++;
      iter = (iter+1)%max_trajectory_queue_size_;
    }
  }
//  ROS_INFO("No match found");
  return false;
}


void JointTrajectoryController::setTrajectoryCmdFromMsg(robot_msgs::JointTraj traj_msg, int id)
{
  std::vector<trajectory::Trajectory::TPoint> tp;
  int msg_size = std::max<int>((int)traj_msg.get_points_size(),1);

  tp.resize(msg_size+1);

  //set first point in trajectory to current position and velocity of the joints
  tp[0].setDimension((int) num_joints_);

  for(int j=0; j < num_joints_; j++)
  {
    tp[0].q_[j] = current_joint_position_[j];
    tp[0].qdot_[j] = current_joint_velocity_[j];
    tp[0].time_ = 0.0;
  }

  if((int)traj_msg.get_points_size() > 0)
  {
    if((int) traj_msg.points[0].get_positions_size() != (int) num_joints_)
    {
      ROS_WARN("Dimension of input trajectory = %d does not match number of controlled joints = %d",(int) traj_msg.points[0].get_positions_size(), (int) num_joints_);
      return;
    }
    else
    {
      for(int i=0; i < (int) traj_msg.get_points_size(); i++)
      {
        tp[i+1].setDimension((int) num_joints_);
        for(int j=0; j < (int) num_joints_; j++)
        {
          tp[i+1].q_[j] = traj_msg.points[i].positions[j];
          tp[i+1].time_ = traj_msg.points[i].time;
//          ROS_INFO("Trajectory: %d %d %f %f",i,j,tp[i+1].q_[j],tp[i+1].time_);
        }
      }
    }
  }
  else
  {
//    ROS_WARN("Trajectory message has no way points");
    //set second point in trajectory to current position of the arm
    tp[1].setDimension((int) num_joints_);

    for(int j=0; j < num_joints_; j++)
    {
      tp[1].q_[j] = current_joint_position_[j];
      tp[1].time_ = 0.0;
    }
  }

  if(setTrajectoryCmd(tp))
  {
//    ROS_INFO("Setting trajectory command");
    current_trajectory_id_ = id;
    joint_trajectory_status_[current_trajectory_id_] = JointTrajectoryController::ACTIVE;
  }
}

void JointTrajectoryController::TrajectoryReceivedOnTopic()
{
  // PREEMPTS anything on the queue
  this->ros_lock_.lock();
  last_traj_req_time_ = current_time_;
//  ROS_INFO("Locked and setting command");
  preemptTrajectoryQueue(traj_msg_,request_trajectory_id_);
  request_trajectory_id_++;
  this->ros_lock_.unlock();
}


bool JointTrajectoryController::setJointTrajSrv(pr2_mechanism_controllers::TrajectoryStart::Request &req,
                                                pr2_mechanism_controllers::TrajectoryStart::Response &resp)
{
  addTrajectoryToQueue(req.traj, request_trajectory_id_);
  request_trajectory_id_++;
  last_traj_req_time_ = current_time_;
  resp.trajectoryid = request_trajectory_id_;

  if(req.requesttiming)
  {
    trajectory::Trajectory tmp(num_joints_);
    std::vector<double> timestamps;

    createTrajectoryFromMsg(req.traj,tmp);
    resp.set_timestamps_size((int)req.traj.get_points_size());
    timestamps.resize((int)req.traj.get_points_size());

    tmp.getTimeStamps(timestamps);

    for(int i=0; i < (int) req.traj.get_points_size(); i++)
    {
      resp.timestamps[i] = timestamps[i];
    }
  }
  return true;
}

bool JointTrajectoryController::queryJointTrajSrv(pr2_mechanism_controllers::TrajectoryQuery::Request &req,
                                                  pr2_mechanism_controllers::TrajectoryQuery::Response &resp)
{
  resp.set_jointnames_size(num_joints_);
  resp.set_jointpositions_size(num_joints_);
  for(int i=0; i < num_joints_; i++)
  {
    resp.jointnames[i] = joint_name_[i];
    resp.jointpositions[i] = current_joint_position_[i];
  }

  if(req.trajectoryid >= joint_trajectory_status_.size())
  {
    resp.trajectorytime = 0.0;
    resp.done = JointTrajectoryController::DOES_NOT_EXIST;
    return false;
  }
  else
  {
    resp.done = joint_trajectory_status_[req.trajectoryid];
  }

  if(current_trajectory_id_ == (int)req.trajectoryid)
  {
    if((int) resp.done == JointTrajectoryController::DONE)
      resp.trajectorytime = trajectory_end_time_ - trajectory_start_time_;
    else
      resp.trajectorytime = current_time_ - trajectory_start_time_;
  }
  else
  {
    resp.trajectorytime = joint_trajectory_time_[req.trajectoryid];
  }
  return true;
}

bool JointTrajectoryController::cancelJointTrajSrv(pr2_mechanism_controllers::TrajectoryCancel::Request &req,
                                                   pr2_mechanism_controllers::TrajectoryCancel::Response &resp)
{
  int status = JointTrajectoryController::NUM_STATUS;
  std::vector<trajectory::Trajectory::TPoint> trajectory_points_vector;
  if(req.trajectoryid > joint_trajectory_status_.size())
  {
    return false;
  }
  else
  {
    status = joint_trajectory_status_[req.trajectoryid];
  }
  deleteTrajectoryFromQueue(req.trajectoryid);
  return true;
}


bool JointTrajectoryController::createTrajectoryPointsVectorFromMsg(const robot_msgs::JointTraj &new_traj, std::vector<trajectory::Trajectory::TPoint> &tp)
{
  if(new_traj.get_points_size() > 0)
  {
    if((int) new_traj.points[0].get_positions_size() != (int) num_joints_)
    {
      ROS_WARN("Dimension of input trajectory = %d does not match number of controlled joints = %d",(int) new_traj.points[0].get_positions_size(), (int) num_joints_);
      return false;
    }
  }
  else
  {
    ROS_WARN("Trajectory message has no way points");
    return false;
  }

  tp.resize((int)new_traj.get_points_size());

  for(int i=0; i < (int) new_traj.get_points_size(); i++)
  {
    tp[i].setDimension((int) num_joints_);
    for(int j=0; j < (int) num_joints_; j++)
    {
      tp[i].q_[j] = new_traj.points[i].positions[j];
      tp[i].time_ = new_traj.points[i].time;
    }
  }

  return true;
}

bool JointTrajectoryController::createTrajectoryFromMsg(const robot_msgs::JointTraj &new_traj,trajectory::Trajectory &return_trajectory)
{
  std::vector<trajectory::Trajectory::TPoint> tp;

  if(!createTrajectoryPointsVectorFromMsg(new_traj, tp))
  {
    return false;
  }

  return_trajectory.setMaxRates(joint_velocity_limits_);
  return_trajectory.setInterpolationMethod(trajectory_type_);

  if(!return_trajectory.setTrajectory(tp))
  {
    ROS_WARN("Trajectory not set correctly");
    return false;
  }
  return true;
}

void JointTrajectoryController::addTrajectoryToQueue(robot_msgs::JointTraj new_traj, int id)
{
  trajectory_queue_.try_lock();

  joint_trajectory_vector_[next_free_index_] = new_traj;
  joint_trajectory_id_[next_free_index_] = id;
  joint_trajectory_time_.push_back(0.0);
  joint_trajectory_status_.push_back(JointTrajectoryController::QUEUED);
  num_trajectory_available_ = (num_trajectory_available_+1)%max_trajectory_queue_size_;
  next_free_index_ = (next_free_index_+1)%max_trajectory_queue_size_;
  trajectory_queue_.unlock();
}

void JointTrajectoryController::preemptTrajectoryQueue(robot_msgs::JointTraj new_traj, int id)
{
  int index = std::max(current_trajectory_index_,0);
  trajectory_queue_.try_lock();
//  ROS_INFO("Setting preempt command");
  joint_trajectory_vector_[index] = new_traj;
  joint_trajectory_id_[index] = id;
  joint_trajectory_time_.push_back(0.0);
  joint_trajectory_status_.push_back(JointTrajectoryController::QUEUED);
  num_trajectory_available_ = (num_trajectory_available_+1)%max_trajectory_queue_size_;
  trajectory_queue_.unlock();

  trajectory_preempted_ = true;
//  ROS_INFO("Done setting preempt command");
}

void JointTrajectoryController::deleteTrajectoryFromQueue(int id)
{
  trajectory_queue_.lock();
  if(id == joint_trajectory_id_[current_trajectory_index_])
  {
    joint_trajectory_vector_[current_trajectory_index_].set_points_size(0);
    trajectory_queue_.unlock();
    trajectory_preempted_ = true;
    return;
  }

  if(num_trajectory_available_ < 1)
    return;

// do a linear search from current index + 1
  int iter = (current_trajectory_index_ + 1)%max_trajectory_queue_size_;
  int num_iterations = 0;
  while(num_iterations < max_trajectory_queue_size_)
  { 
    if(joint_trajectory_id_[iter] == id)
    {
      joint_trajectory_status_[id] = JointTrajectoryController::DELETED;
      num_trajectory_available_--;
      break;
    }
    num_iterations++;
    iter = (iter+1)%max_trajectory_queue_size_;
  }
  trajectory_queue_.unlock();
}

void JointTrajectoryController::publishDiagnostics()
{
  if(!((current_time_ - last_diagnostics_publish_time_) > diagnostics_publish_delta_time_))
  {
    return;
  }

  if(diagnostics_publisher_->trylock())
  {
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
    if(watch_dog_active_)
    {
      status.message = "WATCHDOG";
    }
    else
    {
      status.message = "OK";
    }

    for(unsigned int i=0; i < joint_pv_controllers_.size(); i++)
    {
      v.label = joint_pv_controllers_[i]->getJointName() + "/Position/Actual";
      v.value = joint_pv_controllers_[i]->joint_state_->position_;
      values.push_back(v);
      v.label = joint_pv_controllers_[i]->getJointName() + "/Position/Command";
      joint_pv_controllers_[i]->getCommand(cmd);
      v.value = cmd.positions[0];
      values.push_back(v);
      v.label = joint_pv_controllers_[i]->getJointName() + "/Position/Error (Command-Actual)";
      v.value = cmd.positions[0] - joint_pv_controllers_[i]->joint_state_->position_;
      values.push_back(v);
    }

    for(unsigned int i=0; i < base_pid_controller_.size(); i++)
    {
      int joint_index = base_joint_index_[i];
      v.label = joint_name_[joint_index] + "/Position/Actual";
      v.value = current_joint_position_[joint_index];
      values.push_back(v);
      v.label = joint_name_[joint_index] + "/Position/Command";
      v.value = joint_cmd_[joint_index];
      values.push_back(v);
      v.label = joint_name_[joint_index] + "/Position/Error (Command-Actual)";
      v.value = joint_cmd_[joint_index] - current_joint_position_[joint_index];
      values.push_back(v);
    }
    v.label = "Trajectory id";
    v.value = current_trajectory_id_;
    values.push_back(v);

    v.label = "Trajectory Status:: ";

    if(current_trajectory_id_ < 0)
    {
      v.label += "AT REST";
      v.value = -1;
    }
    else
    {
      v.label += JointTrajectoryStatusString[joint_trajectory_status_[current_trajectory_id_]];
      v.value = joint_trajectory_status_[current_trajectory_id_];
    }
    values.push_back(v);

    v.label = "Trajectory Current Time";
    v.value = current_time_-trajectory_start_time_;
    values.push_back(v);

    v.label = "Trajectory Expected End Time (computed)";
    v.value = trajectory_end_time_-trajectory_start_time_;
    values.push_back(v);

    v.label = "Current trajectory queue index";
    v.value = current_trajectory_index_;
    values.push_back(v);

    v.label = "Number queued trajectories";
    v.value = num_trajectory_available_;
    values.push_back(v);

    v.label = "Next queue free index";
    v.value = next_free_index_;
    values.push_back(v);

    status.set_values_vec(values);
    status.set_strings_vec(strings);
    statuses.push_back(status);

    last_diagnostics_publish_time_ = current_time_;
    diagnostics_publisher_->msg_.set_status_vec(statuses);
    diagnostics_publisher_->unlockAndPublish();
  }
}
