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

// Original version: Melonee Wise <mwise@willowgarage.com>

#include "pr2_mechanism_controllers/head_pan_tilt_controller.h"

using namespace controller;
using namespace std;


ROS_REGISTER_CONTROLLER(HeadPanTiltController);

HeadPanTiltController::HeadPanTiltController():num_joints_(0),last_time_(0)
{
}

HeadPanTiltController::~HeadPanTiltController()
{
  // Assumes the joint controllers are owned by this class.
  for(unsigned int i=0; i < num_joints_;++i)
  {
    delete joint_position_controllers_[i];
  }
}

bool HeadPanTiltController::initXml(mechanism::RobotState * robot, TiXmlElement * config)
{
  robot_ = robot->model_;
  TiXmlElement *elt = config->FirstChildElement("controller");
  while (elt)
  {
    JointPositionController * jpc = new JointPositionController();
    std::cout<<elt->Attribute("type")<<elt->Attribute("name")<<std::endl;
    assert(static_cast<std::string>(elt->Attribute("type")) == std::string("JointPositionController"));
    joint_position_controllers_.push_back(jpc);

    if(!jpc->initXml(robot, elt))
      return false;

    elt = elt->NextSiblingElement("controller");
  }

  num_joints_ = joint_position_controllers_.size();

  fprintf(stderr,"HeadPanTiltController:: num_joints_: %d\n",num_joints_);

  set_pts_.resize(num_joints_);
  last_time_ = robot->hw_->current_time_;

  return true;
}

void HeadPanTiltController::setJointCmd(const std::vector<double> &j_values, const std::vector<std::string> & j_names)
{
  assert(j_values.size() == j_names.size());
  for(uint i = 0; i < j_values.size(); ++i)
  {
    const std::string & name = j_names[i];
    const int id = getJointControllerByName(name);
    assert(id>=0);
    if(id >= 0)
    {
      set_pts_[id] = j_values[i];
    }
  }
}

void HeadPanTiltController::getJointCmd(robot_msgs::JointCmd & cmd) const
{
  const unsigned int n = joint_position_controllers_.size();
  cmd.set_names_size(n);
  for(unsigned int i=0; i<n; ++i)
      cmd.names[i] = joint_position_controllers_[i]->getJointName();

  cmd.set_positions_vec(set_pts_);
}


controller::JointPositionController* HeadPanTiltController::getJointPositionControllerByName(std::string name)
{
  for(int i=0; i< (int) num_joints_; i++)
  {
    if(joint_position_controllers_[i]->getJointName() == name)
    {
      return joint_position_controllers_[i];
    }
  }
    return NULL;
}


int HeadPanTiltController::getJointControllerByName(std::string name)
{
  for(int i=0; i< (int) num_joints_; i++)
  {
    if(joint_position_controllers_[i]->getJointName() == name)
    {
      return i;
    }
  }
  return -1;
}

void HeadPanTiltController::update(void)
{
  for(unsigned int i=0; i < num_joints_;++i)
  {
   joint_position_controllers_[i]->setCommand(set_pts_[i]);
  }

  updateJointControllers();
}

void HeadPanTiltController::updateJointControllers(void)
{

  for(unsigned int i=0;i<num_joints_;++i)
  {
    if (!joint_position_controllers_[i]->joint_state_->calibrated_)
      return;  // joints are not calibrated

    joint_position_controllers_[i]->update();
  }
}

//------ Head controller node --------

ROS_REGISTER_CONTROLLER(HeadPanTiltControllerNode)

HeadPanTiltControllerNode::HeadPanTiltControllerNode()
: Controller(), node_(ros::Node::instance()), TF(*ros::Node::instance(),false, 10000000000ULL)
{
  c_ = new HeadPanTiltController();
}

HeadPanTiltControllerNode::~HeadPanTiltControllerNode()
{
  delete c_;
}

void HeadPanTiltControllerNode::update()
{
  c_->update();
}

bool HeadPanTiltControllerNode::initXml(mechanism::RobotState * robot, TiXmlElement * config)
{
  assert(node_);
  service_prefix_ = config->Attribute("name");

  // Parses subcontroller configuration
  if(!c_->initXml(robot, config))
    return false;
  //suscriptions
  node_->subscribe(service_prefix_ + "/set_command_array", joint_cmds_, &HeadPanTiltControllerNode::setJointCmd, this,1);
  guard_set_command_array_.set(service_prefix_ + "/set_command_array");
  node_->subscribe(service_prefix_ + "/head_track_point", head_track_point_, &HeadPanTiltControllerNode::headTrackPoint, this, 1);
  guard_head_track_point_.set(service_prefix_ + "/head_track_point");

  node_->subscribe(service_prefix_ + "/frame_track_point", frame_track_point_, &HeadPanTiltControllerNode::frameTrackPoint, this, 1);
  guard_frame_track_point_.set(service_prefix_ + "/frame_track_point");
  //services
  node_->advertise_service(service_prefix_ + "/get_command_array", &HeadPanTiltControllerNode::getJointCmd, this);
  guard_get_command_array_.set(service_prefix_ + "/get_command_array");
  node_->advertise<std_msgs::VisualizationMarker>( "visualizationMarker", 0 );
  return true;

}

void HeadPanTiltControllerNode::setJointCmd()
{
  c_->setJointCmd(joint_cmds_.positions,joint_cmds_.names);
}

bool HeadPanTiltControllerNode::getJointCmd(robot_srvs::GetJointCmd::request &req,
					    robot_srvs::GetJointCmd::response &resp)
{
  robot_msgs::JointCmd cmd;
  c_->getJointCmd(cmd);
  resp.command = cmd;
  return true;
}

void HeadPanTiltControllerNode::headTrackPoint()
{
  std::vector<double> pos;
  std::vector<std::string> names;

  tf::Stamped<tf::Point> point;
  point.setX(head_track_point_.point.x);
  point.setY(head_track_point_.point.y);
  point.setZ(head_track_point_.point.z);
  point.stamp_ = ros::Time();
  point.frame_id_ = head_track_point_.header.frame_id;

  std::string pan_parent;
  TF.getParent("head_pan_link", ros::Time(), pan_parent);
  std::string tilt_parent("head_pan_link");

  //     Pan angle

  tf::Stamped<tf::Point> pan_point;

  try{
    //TF.transformPoint("head_pan_link",point, pan_point);
    TF.transformPoint(pan_parent, point, pan_point);
  }
  catch(tf::TransformException& ex){
    ROS_WARN("Transform Exception %s", ex.what());
    return;
  }
  int id = c_->getJointControllerByName("head_pan_joint");
  assert(id>=0);
  double head_pan_angle = atan2(pan_point.y(), pan_point.x());

  names.push_back("head_pan_joint");
  pos.push_back(head_pan_angle);


  //     Tilt angle

  tf::Stamped<tf::Point> tilt_point;

  try{
    TF.transformPoint(tilt_parent, point, tilt_point);
  }
  catch(tf::TransformException& ex){
    ROS_WARN("Transform Exception %s", ex.what());
    return;
  }

  id = c_->getJointControllerByName("head_tilt_joint");
  assert(id>=0);
  double head_tilt_angle = atan2(-tilt_point.z(), tilt_point.x());

  names.push_back("head_tilt_joint");
  pos.push_back(head_tilt_angle);

  c_->setJointCmd(pos,names);
}


void HeadPanTiltControllerNode::frameTrackPoint()
{
  std::vector<double> pos;
  std::vector<std::string> names;
  tf::Stamped<tf::Transform> frame;

  tf::Stamped<tf::Point> point;
  point.setX(frame_track_point_.point.x);
  point.setY(frame_track_point_.point.y);
  point.setZ(frame_track_point_.point.z);
  point.stamp_ = ros::Time();//get the latest transform
  point.frame_id_ = frame_track_point_.header.frame_id;

  try
  {
    TF.lookupTransform(point.frame_id_,"head_pan_link",ros::Time(),frame);
  }
  catch(tf::TransformException& ex)
  {
    ROS_WARN("Transform Exception %s", ex.what());
    return;
  }

  tf::Stamped<tf::Point> pan_point;

  try
  {
    TF.transformPoint("head_pan_link",point, pan_point);
  }
  catch(tf::TransformException& ex)
  {
    ROS_WARN("Transform Exception %s", ex.what());
    return;
  }
  int id = c_->getJointControllerByName("head_pan_joint");
  assert(id>=0);
  double radius = pow(pan_point.x(),2)+pow(pan_point.y(),2);
  double x_intercept = sqrt(radius-pow(frame.getOrigin().y(),2));
  double delta_theta = atan2(pan_point.y(), pan_point.x())-atan2(frame.getOrigin().y(),x_intercept);
  double meas_pan_angle = c_->joint_position_controllers_[id]->joint_state_->position_;
  double head_pan_angle= meas_pan_angle + delta_theta;

  names.push_back("head_pan_joint");
  pos.push_back(head_pan_angle);

  tf::Stamped<tf::Point> tilt_point;
  try{
    TF.transformPoint("head_tilt_link",point,tilt_point);
  }
  catch(tf::TransformException& ex){
    ROS_WARN("Transform Exception %s", ex.what());
    return;
  }


  id = c_->getJointControllerByName("head_tilt_joint");
  assert(id>=0);
  radius = pow(tilt_point.x(),2)+pow(tilt_point.y(),2);
  x_intercept = sqrt(radius-pow(frame.getOrigin().z(),2));
  delta_theta = atan2(-tilt_point.z(), tilt_point.x())-atan2(frame.getOrigin().z(),x_intercept);
  double meas_tilt_angle= c_->joint_position_controllers_[id]->joint_state_->position_;
  double head_tilt_angle= meas_tilt_angle + delta_theta;

  names.push_back("head_tilt_joint");
  pos.push_back(head_tilt_angle);

  c_->setJointCmd(pos,names);

  std_msgs::VisualizationMarker marker;
  marker.header.frame_id = "stereo";
  marker.id = 0;
  marker.type = 0;
  marker.action = 0;
  marker.x = 0;
  marker.y = 0;
  marker.z = 0;
  marker.yaw = 0;
  marker.pitch = 0;
  marker.roll = 0.0;
  marker.xScale = 2;
  marker.yScale = 0.05;
  marker.zScale = 0.05;
  marker.alpha = 255;
  marker.r = 0;
  marker.g = 255;
  marker.b = 0;
  node_->publish("visualizationMarker", marker );

  marker.header.frame_id = "head_pan_link";
  marker.id = 1;
  marker.type = 2;
  marker.action = 0;
  marker.x = pan_point.x();
  marker.y = pan_point.y();
  marker.z = pan_point.z();
  marker.yaw = 0;
  marker.pitch = 0;
  marker.roll = 0.0;
  marker.xScale = 0.1;
  marker.yScale = 0.1;
  marker.zScale = 0.1;
  marker.alpha = 255;
  marker.r = 255;
  marker.g = 0;
  marker.b = 0;
  node_->publish("visualizationMarker", marker );


}
