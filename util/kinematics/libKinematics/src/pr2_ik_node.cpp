//Software License Agreement (BSD License)

//Copyright (c) 2008, Willow Garage, Inc.
//All rights reserved.

//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions
//are met:

// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of Willow Garage, Inc. nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.

//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//POSSIBILITY OF SUCH DAMAGE.


#include <libKinematics/pr2_ik_node.h>
#include <angles/angles.h>

LibKinematicsNode::LibKinematicsNode(std::string node_name,std::string arm_name):ros::Node(node_name),arm_name_(arm_name),increment_(0.01),root_x_(0.0),root_y_(0.0), root_z_(0.0), tf_(*this)
{
    advertiseService("perform_pr2_ik", &LibKinematicsNode::processIKRequest); 
    advertiseService("perform_pr2_ik_closest", &LibKinematicsNode::processIKClosestRequest); 
};

LibKinematicsNode::~LibKinematicsNode()
{
    unadvertiseService("perform_pr2_ik"); 
    unadvertiseService("perform_pr2_ik_closest"); 
};

bool LibKinematicsNode::initializeKinematicModel()
{
  std::vector<NEWMAT::Matrix> axis;
  std::vector<NEWMAT::Matrix> anchor;
  std::vector<std::string> joint_type;
  std::vector<double> min_joint_limits;
  std::vector<double> max_joint_limits;
  std::vector<double> angle_multipliers;

  NEWMAT::Matrix aj(3,1);
  NEWMAT::Matrix an(3,1);

  std::string xml_content;
  std::vector<robot_desc::URDF::Group*> groups;


  (ros::g_node)->getParam("robotdesc/pr2",xml_content);

  // wait for robotdesc/pr2 on param server
  while(!urdf_model_.loadString(xml_content.c_str()))
  {
    ROS_INFO("WARNING: grasp point node is waiting for robotdesc/pr2 in param server.  run roslaunch send.xml or similar.");
    (ros::g_node)->getParam("robotdesc/pr2",xml_content);
    usleep(100000);
  }

  urdf_model_.getGroups(groups);

  int group_index = -1;

  for(int i=0; i < (int) groups.size(); i++)
  {
    if(groups[i]->name == arm_name_)
    {
      group_index = i;
      break;
    }
  }

  if(!group_index) 
    return false;

  if((int) groups[group_index]->linkRoots.size() != 1)
  {
    fprintf(stderr,"robot_kinematics.cpp::Too many roots in serial chain!\n");
    return -1;
  }

  robot_desc::URDF::Link *link_current = groups[group_index]->linkRoots[0];

  root_x_ = link_current->xyz[0];
  root_y_ = link_current->xyz[1];
  root_z_ = link_current->xyz[2];

  root_link_name_ = link_current->name;
  joint_type.resize(NUM_JOINTS);

  min_joint_limits.resize(NUM_JOINTS);
  max_joint_limits.resize(NUM_JOINTS);
  angle_multipliers.resize(NUM_JOINTS);
//  for(int i=0; i<NUM_JOINTS; i++)
  int joint_counter = 0;
  while(joint_counter < NUM_JOINTS)
  {
    if(link_current->joint->type == robot_desc::URDF::Link::Joint::FIXED)
    {
      link_current = findNextLinkInGroup(link_current, groups[group_index]);
      continue;
    }

    aj << fabs(link_current->joint->axis[0]) << fabs(link_current->joint->axis[1]) << fabs(link_current->joint->axis[2]);
    axis.push_back(aj);
    if(joint_counter > 0)
    {
      an(1,1) = an(1,1) + link_current->xyz[0];
      an(2,1) = an(2,1) + link_current->xyz[1];
      an(3,1) = an(3,1) + link_current->xyz[2];
    }
    else
    {
      an(1,1) = 0.0;
      an(2,1) = 0.0;
      an(3,1) = 0.0;
    }
    anchor.push_back(an);   
    min_joint_limits[joint_counter] = link_current->joint->limit[0];
    max_joint_limits[joint_counter] = link_current->joint->limit[1];
    angle_multipliers[joint_counter] = link_current->joint->axis[0]*fabs(link_current->joint->axis[0]) +  link_current->joint->axis[1]*fabs(link_current->joint->axis[1]) +  link_current->joint->axis[2]*fabs(link_current->joint->axis[2]);
    ROS_INFO("Adding joint %s\naxis: %f %f %f\nanchor: %f %f %f",link_current->joint->name.c_str(),link_current->joint->axis[0],link_current->joint->axis[1],link_current->joint->axis[2],an(1,1),an(2,1),an(3,1));
    if(min_joint_limits[joint_counter] == 0.0 && max_joint_limits[joint_counter] == 0.0)
    {
      ROS_INFO("Continuous joint");
      min_joint_limits[joint_counter] = -M_PI;
      max_joint_limits[joint_counter] = M_PI;
    }
    if(joint_counter==2)
    {
      init_solution_theta3_ = (min_joint_limits[joint_counter]+max_joint_limits[joint_counter])/2.0;
      ROS_INFO("Initial guess for inverse kinematics: %f",init_solution_theta3_);
    }
    ROS_INFO("Joint limits %f, %f\n",min_joint_limits[joint_counter],max_joint_limits[joint_counter]);

    link_current = findNextLinkInGroup(link_current, groups[group_index]);
    joint_counter++;
  }

  for(int i=0; i < 7; i++)
    joint_type[i] = std::string("ROTARY");

  arm_kinematics_ = new kinematics::arm7DOF(anchor,axis,joint_type);
  arm_kinematics_->SetJointLimits(min_joint_limits,max_joint_limits);
  arm_kinematics_->increment_ = increment_;
  arm_kinematics_->setAngleMultipliers(angle_multipliers);
  return true;
}


robot_desc::URDF::Link* LibKinematicsNode::findNextLinkInGroup(robot_desc::URDF::Link *link_current, robot_desc::URDF::Group* group)
{
  std::vector<robot_desc::URDF::Link*>::iterator link_iter;

#ifdef DEBUG
  cout << "Current link:: " << link_current->name << endl; 
#endif
  for(link_iter = link_current->children.begin(); link_iter != link_current->children.end(); link_iter++)
  {
#ifdef DEBUG
    cout << (*link_iter)->name;
#endif
    if((*link_iter)->insideGroup(group))
      return *link_iter;
  }
  return NULL;
}

bool LibKinematicsNode::init()
{
  if(!initializeKinematicModel())
    return false;

  return true;
}

// Transform incoming EE pose to the frame that we're hardwired for,
// obeying timeout in the request.  Return true on success, false
// otherwise.
bool LibKinematicsNode::transformPose(robot_srvs::IKService::Request& req,
                                      tf::Stamped<tf::Pose>& pose_out)
{
  tf::Stamped<tf::Pose> tf_pose_in;
  tf::PoseMsgToTF(req.pose, tf_pose_in);

  tf_pose_in.stamp_ = req.header.stamp;
  tf_pose_in.frame_id_ = req.header.frame_id;

  // Transform into the desired frame
  ros::Time start = ros::Time::now();
  bool transformed = false;
  // Heuristically chosen value
  ros::Duration sleeptime = ros::Duration().fromSec(0.01);
  while((req.timeout.toSec() == 0.0) ||
        ((ros::Time::now() - start) < req.timeout))
  {
    try
    {
      tf_.transformPose(getTargetFrame(), tf_pose_in, pose_out);
      transformed = true;
      break;
    }
    catch(tf::TransformException e)
    {
      // Try, try, again
      sleeptime.sleep();
    }
  }

  return transformed;
}

bool LibKinematicsNode::processIKRequest(robot_srvs::IKService::Request &req, robot_srvs::IKService::Response &resp)
{
  tf::Stamped<tf::Pose> tf_pose;

  if(!transformPose(req, tf_pose))
  {
    ROS_WARN("Failed to transform from %s to %s within timeout (%f)",
             req.header.frame_id.c_str(), 
             getTargetFrame().c_str(), 
             req.timeout.toSec());
    return false;
  }

  NEWMAT::Matrix g0(4,4);
  btScalar m[16];
  tf_pose.getOpenGLMatrix(m);

  //ROS_INFO("computeIKSolution: Input transform");
  for(int i=0; i < 4; i++)
  {
    for(int j=0; j < 4; j++)
    {
      g0(j+1,i+1) = m[i*4+j];
    }
  }

  g0(1,4) = g0(1,4) - root_x_;
  g0(2,4) = g0(2,4) - root_y_;
  g0(3,4) = g0(3,4) - root_z_;

  if(arm_kinematics_->computeIKFast(g0,2,init_solution_theta3_))
  {
//     ROS_INFO("Solution::");
    resp.traj.set_points_size(arm_kinematics_->solution_ik_.size());

    for(int i=0; i < (int) arm_kinematics_->solution_ik_.size(); i++)
    {
      resp.traj.points[i].set_positions_size(7); 
      for(int j=0; j < 7; j++)
      {
        resp.traj.points[i].positions[j] = arm_kinematics_->solution_ik_[i][j]; 
//        ROS_INFO("%f\n",arm_kinematics_->solution_ik_[i][j]);
      }
    }

    return true;
  }
  return false;
}

bool LibKinematicsNode::processIKClosestRequest(robot_srvs::IKService::Request &req, robot_srvs::IKService::Response &resp)
{
  tf::Stamped<tf::Pose> tf_pose;

  if(!transformPose(req, tf_pose))
  {
    ROS_WARN("Failed to transform from %s to %s within timeout (%f)",
             req.header.frame_id.c_str(), 
             getTargetFrame().c_str(), 
             req.timeout.toSec());
    return false;
  }

  int num_joints = req.joint_pos.positions.size();

  if(num_joints != 7)
  {
    ROS_ERROR("Request.joint_pos has no joint values: %d",num_joints);
    return false;
  }

  std::vector<double> current_joint_pos;
  current_joint_pos.resize(num_joints);

  for(int i=0; i < num_joints; i++)
  {
    current_joint_pos[i] = req.joint_pos.positions[i];
  }

  double init_solution_t3 = current_joint_pos[2];

  NEWMAT::Matrix g0(4,4);
  btScalar m[16];
  tf_pose.getOpenGLMatrix(m);

  //ROS_INFO("computeIKSolution: Input transform");
  for(int i=0; i < 4; i++)
  {
    for(int j=0; j < 4; j++)
    {
      g0(j+1,i+1) = m[i*4+j];
    }
  }

  g0(1,4) = g0(1,4) - root_x_;
  g0(2,4) = g0(2,4) - root_y_;
  g0(3,4) = g0(3,4) - root_z_;

  if(arm_kinematics_->computeIKFast(g0,2,init_solution_t3))
  {

    resp.traj.set_points_size(1);
    resp.traj.points[0].set_positions_size(7); 
    int sol_index = closestJointSolution(current_joint_pos,arm_kinematics_->solution_ik_);
//    ROS_INFO("sol_index : %d of %d solutions",sol_index,(int) arm_kinematics_->solution_ik_.size());
    for(int j=0; j < 7; j++)
    {
      resp.traj.points[0].positions[j] = arm_kinematics_->solution_ik_[sol_index][j]; 
    }

    return true;
  }
  return false;
}


int LibKinematicsNode::closestJointSolution(const std::vector<double> current_joint_pos, const std::vector<std::vector<double> > new_positions)
{
  int num_joints = current_joint_pos.size();
  std::vector<double> euc_distances;

//  ROS_INFO("Number of solutions: %d",(int)new_positions.size());
  euc_distances.resize(new_positions.size());

  int sol_index = 0;
  double sol_min = 0.0;

  for(int i=0; i < (int) new_positions.size(); i++)
  {
    euc_distances[i] = 0.0;
    for(int j=0; j< num_joints; j++)
    {
      euc_distances[i] += pow(angles::shortest_angular_distance(new_positions[i][j],current_joint_pos[j]),2);
    }
    if(i==0)
    {
      sol_min = euc_distances[i];
    }
//    ROS_INFO("Euclidean joint distance: %d, %f",i,euc_distances[i]);
    if(euc_distances[i] < sol_min)
    {
      sol_min = euc_distances[i];
      sol_index = i;
    }
  }
  return sol_index;
}


using namespace kinematics;

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  LibKinematicsNode kn("lib_kinematic_node","right_arm");
  kn.init();

/*  NEWMAT::Matrix g(4,4); 
  g(1,1) = 1.0;
  g(1,2) = 0.0;
  g(1,3) = 0.0;
  g(2,1) = 0.0;
  g(2,2) = 1.0;
  g(2,3) = 0.0;
  g(3,1) = 0.0;
  g(3,2) = 0.0;
  g(3,3) = 1.0;

  g(1,4) = 0.75;
  g(2,4) = 0.0;
  g(3,4) = 0.0;
  g(4,4) = 1.0;

 kn.arm_kinematics_->computeIKFast(g,2,kn.init_solution_theta3_);

 for(int i=0; i < (int) kn.arm_kinematics_->solution_ik_.size(); i++)
 { 
   for(int j=0; j< 7; j++)
   {
     printf("%f ",kn.arm_kinematics_->solution_ik_[i][j]);
   }
   printf("\n");
 }
*/
  try {
    kn.spin();
  }
  catch(char const* e){
    std::cout << e << std::endl;
  }
  
  return(0);
}

