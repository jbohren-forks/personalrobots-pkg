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
 
 #include <ros/node.h>
 #include <rosconsole/rosconsole.h>
 #include <mechanism_model/robot.h>
 #include <mechanism_model/joint.h>
 #include <robot_msgs/JointCmd.h>
 #include <robot_mechanism_controllers/ros_serialchain_model.h>

namespace controller
{

bool SerialChainModelWrapper::toState(const mechanism::RobotState * rstate, StateVector &state) const
{
  ROS_ASSERT(rstate);
  const int N=state.rows();
  //Assuming we are in the (position,velocity) representation
  ROS_ASSERT(N==int(2*indexes_.size()));
  
  for(IndexMap::const_iterator it=indexes_.begin();it!=indexes_.end();++it)
  {
    const std::string &name=it->first;
    const int n=it->second;
    const mechanism::JointState *joint=rstate->getJointState(name);
    if(!joint)
    {
      ROS_ERROR_STREAM("Failed to get state for joint name "<<name);
      return false;
    }
    state(n,0)=joint->position_;
    state(N/2+n,0)=joint->velocity_;
  }
  return true;
}
  
bool SerialChainModelWrapper::toState(const robot_msgs::JointCmd * cmd, StateVector & state) const
{
  ROS_ASSERT(cmd);
  const std::vector<std::string> &names=cmd->names;
  const std::vector<double> &positions = cmd->positions;
  const std::vector<double> &velocities = cmd->velocity;
  const int n=indexes_.size();
  if(!(names.size()==positions.size() && names.size()==velocities.size()))
  {
    ROS_ERROR("Size mismatch");
    return false;
  }
  
  int s=0;
  for(unsigned int i=0;i<names.size();++i)
  {
    IndexMap::const_iterator it=indexes_.find(names.at(i));
    if(it!=indexes_.end())
    {
      state(it->second,0)=positions.at(i);
      state(it->second+n,0)=velocities.at(i);
      s++;
    }
  }
  if(s<n)
  {
    ROS_ERROR("Missing variables to be affected");
    return false;
  }
  return true;
}


bool SerialChainModelWrapper::setEffort(mechanism::RobotState *robot_state, const InputVector &effort) const
{
  for(IndexMap::const_iterator it=indexes_.begin();it!=indexes_.end();++it)
  {
    const std::string &name=it->first;
    const int n=it->second;
    mechanism::JointState *state=robot_state->getJointState(name);
    if(!state)
    {
      ROS_ERROR_STREAM("Failed to get state for joint name "<<name);
      return false;
    }
    state->commanded_effort_=effort(n);
  }
  return true;
}

bool SerialChainModelWrapper::initXml(mechanism::RobotState * robot, TiXmlElement *config)
{
  indexes_.clear();
  ROS_ASSERT(config);
  if(static_cast<std::string>(config->Attribute("name"))!="serial_chain")
  {
    ROS_ERROR("Wrong model type");
    return false;
  }
  ROS_DEBUG(">>Wrapper CONF");
  TiXmlElement *kin_node=config->FirstChildElement("kinematics");
  if(!kin_node)
  {
    ROS_ERROR("Missing kinematic chain name");
    return false;
  }
  const std::string chain_name=kin_node->GetText();
  ROS_DEBUG_STREAM("Kinematic chain is "<<chain_name);
  TiXmlElement *desc_node=config->FirstChildElement("robot_description");
  if(!desc_node)
  {
    ROS_ERROR("Missing robot description name");
    return false;
  }
  const std::string desc_name=desc_node->GetText();
  ROS_DEBUG_STREAM("robot description is "<<desc_name);
  
  ros::node * const node = ros::node::instance();
  ROS_ASSERT(node);
  std::string desc_content;
  node->get_param("robotdesc/"+desc_name, desc_content);
  if(!init(desc_content, chain_name))
  {
    ROS_ERROR("Failed to initialize model kinematics");
    return false;
  }
  
  TiXmlElement *joints = config->FirstChildElement("joints");
  if(!joints)
  {
    ROS_ERROR("Missing array of joints");
    return false;
  }
  joints=joints->FirstChildElement("joint");
  int s=0;
  while(joints)
  {
    const std::string & j_name = static_cast<std::string>(joints->Attribute("name"));
    indexes_.insert(std::pair<std::string,int>(j_name,s));
    ROS_DEBUG_STREAM(j_name<<"\t"<<s);
    s++;
    joints=joints->NextSiblingElement("joint");
  }
  ROS_DEBUG("Loaded joints");
  
  if(2*s!=states())
  {
    ROS_ERROR_STREAM("Size mismatch: we have "<<s<<" loaded joints but the model advertizes "<<states()<<" states");
    return false;
  }
  return true;
}
}