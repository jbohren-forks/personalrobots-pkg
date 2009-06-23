/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

/** \author Mrinal Kalakrishnan */

#include <dynamics_estimation/dynamics_estimation_node.h>
#include <boost/shared_ptr.hpp>
#include <ros/time.h>
#include <tinyxml/tinyxml.h>
#include <cstdio>
#include <rosrecord/Player.h>
#include <kdl/chainfksolverposfull_recursive.hpp>
#include <kdl/chainfksolvervelfull_recursive.hpp>

#define DYNAMICS_DEBUG 1

using namespace Eigen;

namespace dynamics_estimation
{

DynamicsEstimationNode::DynamicsEstimationNode():
  use_bag_file_(false),
  hardware_interface_(0),
  finished_data_collection_(false)
{

}

DynamicsEstimationNode::~DynamicsEstimationNode()
{
}

bool DynamicsEstimationNode::init()
{
  bool success;

  // read parameters
  node_handle_.param("~max_samples", max_samples_, 1000000);
  node_handle_.param("~chain_root", chain_root_, std::string("torso_lift_link"));
  node_handle_.param("~chain_tip", chain_tip_, std::string("r_gripper_tool_frame"));
  node_handle_.param("~robot", robot_param_, std::string("robotdesc/pr2"));

  if (node_handle_.getParam("~bag_file", bag_file_))
  {
    use_bag_file_ = true;
  }

  // get robot information
  std::string robot_desc;
  success = node_handle_.getParam(robot_param_, robot_desc);
  if (!success)
  {
    ROS_ERROR("ERROR: Could not access %s from param server", robot_param_.c_str());
    return false;
  }

  // parse robot information
  TiXmlDocument doc;
  doc.Parse(robot_desc.c_str());
  TiXmlElement *root = doc.FirstChildElement("robot");
  if (!root)
  {
    ROS_ERROR("Error finding 'robot' tag in xml\n");
    return false;
  }
  robot_.hw_ = &hardware_interface_;
  robot_.initXml(root);

  // create the mechanism_chain_ and kdl_chain_
  if (!mechanism_chain_.init(&robot_, chain_root_, chain_tip_))
  {
    ROS_ERROR("Error creating mechanism chain");
    return false;
  }
  mechanism_chain_.toKDL(kdl_chain_);
  num_joints_ = kdl_chain_.getNrOfJoints();

  // initialize the joint_name_to_index_ mapping:
  for (int i=0; i<num_joints_; i++)
  {
    std::string name = mechanism_chain_.getJointName(i);
    joint_name_to_index_.insert(std::make_pair(name,i));
  }

  // allocate memory for trajectory points
  ROS_INFO("Allocating memory for %d data points, %d joints", max_samples_, num_joints_);
  trajectory_points_.resize(max_samples_,TrajectoryPoint(num_joints_));
  num_trajectory_points_ = 0;

  if (use_bag_file_)
  {
    if (!loadDataFromBagFile())
    {
      ROS_ERROR("Couldn't read bag file!");
      return false;
    }
  }
  else
  {
    // subscribe to /mechanism_state
    ROS_INFO("Subscribing to /mechanism_state");
    mechanism_state_sub_ = node_handle_.subscribe(std::string("/mechanism_state"), 10000, &DynamicsEstimationNode::mechanismStateCallback, this);
  }

  return true;
}

int DynamicsEstimationNode::run()
{
  while (node_handle_.ok())
  {
    ros::spinOnce();
    if (finished_data_collection_)
    {
      runProcessing();
      return 0;
    }
  }
  return 0;
}

void DynamicsEstimationNode::mechanismStateCallback(const boost::shared_ptr<mechanism_msgs::MechanismState const>& mechanism_state)
{
  // a safety guard to prevent accidental buffer overrun
  if (finished_data_collection_)
    return;

  // iterate through the joints in mechanism_state and find the corresponding joint index from the map:
  for (unsigned int i=0; i<mechanism_state->joint_states.size(); i++)
  {
    std::string name = mechanism_state->joint_states[i].name;
    std::map<std::string,int>::iterator it = joint_name_to_index_.find(name);
    if (it == joint_name_to_index_.end())
      continue;
    int joint_index = it->second;

    //copy the position and effort:
    trajectory_points_[num_trajectory_points_].joint_pos_vel_acc_.q(joint_index) =
        mechanism_state->joint_states[i].position;
    trajectory_points_[num_trajectory_points_].joint_effort_(joint_index) =
        mechanism_state->joint_states[i].applied_effort;

  }

  // copy the time:
  trajectory_points_[num_trajectory_points_].time_ = mechanism_state->time;

  // increment the trajectory point counter:
  num_trajectory_points_++;
  if (num_trajectory_points_ % 1000 == 0)
  {
    ROS_INFO("%d points received", num_trajectory_points_);
  }

  // check if we're done collecting data
  if (num_trajectory_points_ == max_samples_)
  {
    finished_data_collection_ = true;
    // unsubscribe from mechanism_state:
    if (!use_bag_file_)
      mechanism_state_sub_.shutdown();
  }
}

bool DynamicsEstimationNode::loadDataFromBagFile()
{
  ros::record::Player player;
  if (!player.open(bag_file_, ros::Time::now()))
    return false;
  player.addHandler("/mechanism_state", &DynamicsEstimationNode::mechanismStatePlayerCallback, this, NULL);

  // process all the messages:
  while (player.nextMsg())
  {
    if (finished_data_collection_)
      break;
  }

  // reached the end of the data, so set finished_data_collection_ to true anyway
  finished_data_collection_ = true;
  player.close();
  return true;
}

void DynamicsEstimationNode::mechanismStatePlayerCallback(std::string name, mechanism_msgs::MechanismState* m, ros::Time play_time, ros::Time record_time, void* n)
{
  boost::shared_ptr<const mechanism_msgs::MechanismState> m_ptr(new mechanism_msgs::MechanismState(*m));
  mechanismStateCallback(m_ptr);
}

void DynamicsEstimationNode::runProcessing()
{
  filterTrajectories();
  performForwardKinematics();
  removeJointLimitPoints();
  processDataForRegression();
  runRegression();
}

void DynamicsEstimationNode::filterTrajectories()
{
#ifdef DYNAMICS_DEBUG
  FILE *debug_file = fopen("/tmp/dynamics_debug.txt", "w");
#endif

  const int diff_rule_points = diff_rule_points_;
  int invalid_points = 0;

  // generate the velocities and accelerations:
  for (int i=0; i<num_trajectory_points_; i++)
  {
    // set the first few and last few points as invalid:
    if (i < diff_rule_points/2 || i >= (num_trajectory_points_-diff_rule_points/2))
    {
      trajectory_points_[i].valid_ = false;
      invalid_points++;
      continue;
    }

    // get the average sampling time:
    double avg_sampling_time = (trajectory_points_[i+diff_rule_points/2].time_ -
         trajectory_points_[i-diff_rule_points/2].time_)/(diff_rule_points-1.0);
    trajectory_points_[i].avg_sampling_time_ = avg_sampling_time;

    // check the individual sampling times, reject this data point if any of them are off by
    // 10% of the avg:
    for (int j=i-diff_rule_points/2; j<i+diff_rule_points/2; j++)
    {
      double sampling_time = trajectory_points_[j+1].time_ - trajectory_points_[j].time_;
      if (fabs(sampling_time - avg_sampling_time) > 0.1*avg_sampling_time)
      {
        trajectory_points_[i].valid_ = false;
        break;
      }
    }
    if (!trajectory_points_[i].valid_)
    {
      invalid_points++;
      continue;
    }

    // now apply the differentiation rules:
    for (int j=0; j<num_joints_; j++)
    {
      trajectory_points_[i].joint_pos_vel_acc_.qdot(j) =
          (1.0/(12.0*avg_sampling_time)) * (
          +1.0*trajectory_points_[i-2].joint_pos_vel_acc_.q(j)
          -8.0*trajectory_points_[i-1].joint_pos_vel_acc_.q(j)
          +8.0*trajectory_points_[i+1].joint_pos_vel_acc_.q(j)
          -1.0*trajectory_points_[i+2].joint_pos_vel_acc_.q(j));

      trajectory_points_[i].joint_pos_vel_acc_.qdotdot(j) =
          (1.0/(12.0*avg_sampling_time*avg_sampling_time)) * (
          -1.0*trajectory_points_[i-2].joint_pos_vel_acc_.q(j)
          +16.0*trajectory_points_[i-1].joint_pos_vel_acc_.q(j)
          -30.0*trajectory_points_[i].joint_pos_vel_acc_.q(j)
          +16.0*trajectory_points_[i+1].joint_pos_vel_acc_.q(j)
          -1.0*trajectory_points_[i+2].joint_pos_vel_acc_.q(j));

#ifdef DYNAMICS_DEBUG
      fprintf(debug_file, "%f\t%f\t%f\t",
          trajectory_points_[i].joint_pos_vel_acc_.q(j),
          trajectory_points_[i].joint_pos_vel_acc_.qdot(j),
          trajectory_points_[i].joint_pos_vel_acc_.qdotdot(j));
#endif
    }
#ifdef DYNAMICS_DEBUG
    fprintf(debug_file, "\n");
#endif

  }

  ROS_INFO("%d total trajectory trajectory received.", num_trajectory_points_);
  ROS_INFO("%d points discarded due to possible missed messages or control cycles.", invalid_points);

#ifdef DYNAMICS_DEBUG
  fclose(debug_file);
#endif

}

void DynamicsEstimationNode::removeJointLimitPoints()
{
  std::vector<double> joint_max;
  std::vector<double> joint_min;
  std::vector<bool> has_limits;

  int num_joint_limit_points=0;

  joint_max.resize(num_joints_, 0.0);
  joint_min.resize(num_joints_, 0.0);
  has_limits.resize(num_joints_, true);

  // create the max and min values as 90% of the joint angle range:
  for (int i=0; i<num_joints_; i++)
  {
    mechanism::Joint *joint = mechanism_chain_.getJoint(i);
    if (joint->type_== mechanism::JOINT_CONTINUOUS)
    {
      has_limits[i] = false;
      continue;
    }
    double max = joint->joint_limit_max_;
    double min = joint->joint_limit_min_;
    double range = max - min;
    double safety_region = 0.05*range;
    joint_max[i] = max - safety_region;
    joint_min[i] = min + safety_region;
  }

  for (int i=0; i<num_trajectory_points_; i++)
  {
    if (!trajectory_points_[i].valid_)
      continue;
    for (int j=0; j<num_joints_; j++)
    {
      if (!has_limits[j])
        continue;
      if (trajectory_points_[i].joint_pos_vel_acc_.q(j) < joint_min[j] ||
          trajectory_points_[i].joint_pos_vel_acc_.q(j) > joint_max[j])
      {
        trajectory_points_[i].valid_ = false;
        num_joint_limit_points++;
        break;
      }
    }
  }

  ROS_INFO("%d points discarded due to joint limit proximity", num_joint_limit_points);
}

void DynamicsEstimationNode::performForwardKinematics()
{
  // create the FK position and velocity solver:
  KDL::ChainFkSolverVelFull_recursive vel_solver(kdl_chain_);

  std::vector<KDL::FrameVel> velocities;

  KDL::JntArrayVel jnt_array_vel;
  jnt_array_vel.resize(kdl_chain_.getNrOfJoints());

  int num_segments = kdl_chain_.getNrOfSegments();

  velocities.resize(num_segments);

  // do the FK for all trajectory points:
  for (int i=0; i<num_trajectory_points_; i++)
  {
    //if (!trajectory_points_[i].valid_)
    //  continue;
    jnt_array_vel.q = trajectory_points_[i].joint_pos_vel_acc_.q;
    jnt_array_vel.qdot = trajectory_points_[i].joint_pos_vel_acc_.qdot;
    if (vel_solver.JntToCart(jnt_array_vel, velocities)!=0)
    {
      ROS_ERROR("Forward kinematics error!");
    }
    int joint = 0;
    KDL::FrameVel prevVel = KDL::FrameVel::Identity();
    for (int j=0; j<num_segments; j++)
    {
      if(kdl_chain_.getSegment(j).getJoint().getType()!=KDL::Joint::None)
      {
        // position and velocity
        trajectory_points_[i].cart_pos_vel_acc_[joint].p.p = prevVel.p.p;
        trajectory_points_[i].cart_pos_vel_acc_[joint].p.v = prevVel.p.v;

        // orientation and angular velocity
        trajectory_points_[i].cart_pos_vel_acc_[joint].M.R = prevVel.M.R;
        trajectory_points_[i].cart_pos_vel_acc_[joint].M.w = prevVel.M.w;
        joint++;
      }
      prevVel = velocities[j];
    }
  }

  // differentiate the velocities to get accelerations
  for (int i=0; i<num_trajectory_points_; i++)
  {
    for (int j=i-diff_rule_points_/2; j<=i+diff_rule_points_/2; j++)
    {
      // mark the edges as invalid:
      if (j < 0 || j >= num_trajectory_points_)
      {
        trajectory_points_[i].valid_fk_ = false;
        break;
      }

      // mark points involving invalid joint velocities as invalid:
      if (!trajectory_points_[j].valid_)
      {
        trajectory_points_[i].valid_fk_ = false;
        break;
      }
    }
    if (!trajectory_points_[i].valid_fk_)
      continue;

    // now apply the differentiation rules:
    for (int j=0; j<num_joints_; j++)
    {
      for (int k=0; k<3; k++)
      {
        trajectory_points_[i].cart_pos_vel_acc_[j].p.dv(k) =
            (1.0/(12.0*trajectory_points_[i].avg_sampling_time_)) * (
            +1.0*trajectory_points_[i-2].cart_pos_vel_acc_[j].p.v(k)
            -8.0*trajectory_points_[i-1].cart_pos_vel_acc_[j].p.v(k)
            +8.0*trajectory_points_[i+1].cart_pos_vel_acc_[j].p.v(k)
            -1.0*trajectory_points_[i+2].cart_pos_vel_acc_[j].p.v(k));

        trajectory_points_[i].cart_pos_vel_acc_[j].M.dw(k) =
            (1.0/(12.0*trajectory_points_[i].avg_sampling_time_)) * (
            +1.0*trajectory_points_[i-2].cart_pos_vel_acc_[j].M.w(k)
            -8.0*trajectory_points_[i-1].cart_pos_vel_acc_[j].M.w(k)
            +8.0*trajectory_points_[i+1].cart_pos_vel_acc_[j].M.w(k)
            -1.0*trajectory_points_[i+2].cart_pos_vel_acc_[j].M.w(k));
      }
    }

  }

}

static inline Eigen::Matrix3d getCrossProductMatrix(const Eigen::Vector3d& v)
{
  return (Eigen::Matrix3d() << 0, -v.z(), v.y(),
                        v.z(), 0, -v.x(),
                        -v.y(), v.x(), 0).finished();
}

static inline Eigen::Matrix<double, 3, 6> getInertiaMultiplierMatrix(const Eigen::Vector3d& v)
{
  return (Eigen::Matrix<double, 3, 6>() <<
        v.x(), v.y(), v.z(), 0, 0, 0,
        0, v.x(), 0, v.y(), v.z(), 0,
        0, 0, v.x(), 0, v.y(), v.z()
      ).finished();
}

void DynamicsEstimationNode::processDataForRegression()
{
  // All the math is from An, Atkeson and Hollerbach, "Model-based control of
  // a robot manipulator", 1988, MIT Press, pp. 91--92

  Ktrans_K_ = MatrixXd::Zero(num_joints_*11, num_joints_*11);
  Ktrans_tau_ = VectorXd::Zero(num_joints_*11);

  Matrix<double, 6, 10> A[num_joints_];         // kinematic matrix describing link motions
  Matrix<double, 6, 6> T[num_joints_-1];        // wrench transmission matrices
  Matrix<double, 6, 1> joint_axes[num_joints_]; // joint axis
  MatrixXd bigK(6*num_joints_, 10*num_joints_); // the upper triangular-ish matrix of U's from pg 92

  Vector3d gravity = Vector3d::UnitZ() * -9.81;
  Vector3d omega_dot;
  Vector3d omega;
  Vector3d p_dot_dot;
  Matrix3d R;
  Vector3d p;
  RowVectorXd K_row(num_joints_*11);

  // preload the joint axes:
  int num_segments = kdl_chain_.getNrOfSegments();
  int joint=0;
  for (int i=0; i<num_segments; i++)
  {
    const KDL::Joint *kdl_joint = &kdl_chain_.getSegment(i).getJoint();
    KDL::Joint::JointType joint_type = kdl_joint->getType();
    if (joint_type == KDL::Joint::None)
      continue;
    KDL::Vector axis = kdl_joint->JointAxis();

    joint_axes[joint].setZero(6);
    // convert joint axis into a 6D vector for convenience:
    if (joint_type == KDL::Joint::RotAxis ||
            joint_type == KDL::Joint::RotX ||
            joint_type == KDL::Joint::RotY ||
            joint_type == KDL::Joint::RotZ)
    {
      joint_axes[joint][3] = axis[0];
      joint_axes[joint][4] = axis[1];
      joint_axes[joint][5] = axis[2];
    }
    else
    {
      joint_axes[joint][0] = axis[0];
      joint_axes[joint][1] = axis[1];
      joint_axes[joint][2] = axis[2];
    }
    joint++;
  }


  for (int i=0; i<num_trajectory_points_; i++)
  {
    if (!trajectory_points_[i].valid_ || !trajectory_points_[i].valid_fk_)
      continue;

    // build the A matrix for each joint/link:
    for (int j=0; j<num_joints_; j++)
    {
      // reset the matrix:
      A[j].setZero(6, 10);

      // assign all the KDL vectors to Eigen vectors:
      KDL::FrameAcc *frame = &trajectory_points_[i].cart_pos_vel_acc_[j];

      for (int k=0; k<3; k++)
      {
        p_dot_dot[k] = frame->p.dv[k];
        omega[k] = frame->M.w[k];
        omega_dot[k] = frame->M.dw[k];
      }

      // build the A matrix:
      A[j].block<3,1>(0,0) = p_dot_dot - gravity;
      A[j].block<3,3>(0,1) = getCrossProductMatrix(omega_dot) +
          getCrossProductMatrix(omega)*getCrossProductMatrix(omega);
      A[j].block<3,3>(3,1) = getCrossProductMatrix(gravity - p_dot_dot);
      A[j].block<3,6>(3,4) = getInertiaMultiplierMatrix(omega_dot) +
          getCrossProductMatrix(omega)*getInertiaMultiplierMatrix(omega);

      // std::cout << A[j] << std::endl;
    }

    // build the wrench transmission matrices (T):
    for (int j=0; j<num_joints_-1; j++)
    {
      KDL::Frame frame_parent = trajectory_points_[i].cart_pos_vel_acc_[j].GetFrame();
      KDL::Frame frame_child = trajectory_points_[i].cart_pos_vel_acc_[j+1].GetFrame();

      // not sure about this...
      KDL::Frame frame_relative = frame_parent.Inverse()*frame_child;

      // copy KDL stuff into Eigen matrices and vectors:
      for (int a=0; a<3; a++)
      {
        for (int b=0; b<3; b++)
        {
          R(a,b) = frame_relative.M(a,b);
        }
       p[a] = frame_relative.p[a];
      }

      // form the matrix:
      T[j].setZero(6, 6);
      T[j].block<3,3>(0,0) = R;
      T[j].block<3,3>(3,0) = getCrossProductMatrix(p)*R;
      T[j].block<3,3>(3,3) = R;
    }

    // now construct the bigK matrix:
    // we do it from the last joint to the first because we can make
    // use of the recurrence relation
    bigK.setZero(6*num_joints_, 10*num_joints_);
    for (int j=num_joints_-1; j>=0; j--)
    {
      int row = 6*j;
      for (int k=num_joints_-1; k>=j; k--)
      {
        int column = 10*k;
        if (j==k)
          bigK.block<6,10>(row,column) = A[j];
        else
        {
          bigK.block<6,10>(row,column) = T[j]*bigK.block<6,10>(row+6,column);
        }
      }
    }

    // now generate one row for each joint that we can actually make use of
    // which is the only along the joint axis:
    for (int j=0; j<num_joints_; j++)
    {
      K_row.setZero(11*num_joints_);
      for (int k=0; k<6; k++) // for each row
      {
        for (int l=0; l<num_joints_; l++) // for each joint contibution
        {
          K_row.segment<10>(l*11) += joint_axes[j][k] * bigK.block<1,10>(j*6+k,l*10);
        }
      }
      // assign the viscous friction term:
      K_row[j*11 + 10] = trajectory_points_[i].joint_pos_vel_acc_.qdot(j);
      //std::cout << K_row << std::endl;
    }

  }
}

void DynamicsEstimationNode::runRegression()
{
  psi_ = Eigen::VectorXd::Zero(num_joints_*11);
 // Ktrans_K_.svd().solve(Ktrans_tau_, &psi_);
}

} // namespace dynamics_estimation

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dynamics_estimation_node");
  dynamics_estimation::DynamicsEstimationNode den;
  den.init();
  return den.run();
}
