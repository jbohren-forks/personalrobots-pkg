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
#include <pr2_mechanism_controllers/odometry.h>

using namespace controller;
ROS_REGISTER_CONTROLLER(Odometry)

Odometry::Odometry()
{
  init();
}

Odometry::~Odometry()
{
  odometry_publisher_->stop();
  transform_publisher_->stop();

  delete odometry_publisher_;
  delete transform_publisher_;
}

void Odometry::init()
{
  ros::Node::instance()->param("~odometer/distance",odometer_distance_,0.0);
  ros::Node::instance()->param("~odometer/angle",odometer_angle_,0.0);
  ros::Node::instance()->param("~odom/x",odom_.x,0.0);
  ros::Node::instance()->param("~odom/y",odom_.y,0.0);
  ros::Node::instance()->param("~odom/z",odom_.z,0.0);
  ros::Node::instance()->param("~wheel_radius",wheel_radius_,0.070);

  ros::Node::instance()->param<std::string>("~ils_weight_type",ils_weight_type_,"Gaussian");
  ros::Node::instance()->param<int>("~ils_max_iterations",ils_max_iterations_,3);
  ros::Node::instance()->param<std::string>("~xml_wheel_name",xml_wheel_name_,"wheel");
  ros::Node::instance()->param<std::string>("~odom_frame",odom_frame_,"odom");
  ros::Node::instance()->param("~expected_publish_time",expected_publish_time_,0.03);

  odometry_publisher_ = NULL;
  transform_publisher_ = NULL;
  num_wheels_ = 0;
}

void Wheel::initXml(mechanism::RobotState *robot_state, TiXmlElement *config)
{
   std::string name = config->Attribute("name");
   mechanism::Link *link = robot_state->model_->getLink(name);
   joint_ = robot_state->getJointState(link->joint_name_);
   tf::Transform offset = link->getOffset();
   offset_.x = offset.getOrigin().x();
   offset_.y = offset.getOrigin().y();
   offset_.z = offset.getOrigin().z();
   link = robot_state->model_->getLink(link->parent_name_);
   offset = link->getOffset();
   steer_offset_.x = offset.getOrigin().x();
   steer_offset_.y = offset.getOrigin().y();
   steer_offset_.z = offset.getOrigin().z();
   steer_ = robot_state->getJointState(link->joint_name_);
   assert(steer_);
   ROS_INFO("Loaded wheel: %s with parent caster joint: %s",name.c_str(),link->joint_name_.c_str());
}

bool Odometry::initXml(mechanism::RobotState *robot_state, TiXmlElement *config)
{
  TiXmlElement *elt = config->FirstChildElement(xml_wheel_name_);
  Wheel tmp;
  while(elt){
     tmp.initXml(robot_state,elt);
     wheel_.push_back(tmp);
     elt = elt->NextSiblingElement(xml_wheel_name_);
     num_wheels_++;
  }

/*  cbv_lhs_ = OdomMatrix16x3::Zero();
  cbv_rhs_ = OdomMatrix16x1::Zero();
  cbv_soln_= OdomMatrix3x1::Zero();

  fit_lhs_ = OdomMatrix16x3::Zero();
  fit_rhs_ = OdomMatrix16x1::Zero();
  fit_soln_ = OdomMatrix3x1::Zero();

  fit_residual_ = OdomMatrix16x1::Zero();
  odometry_residual_ = OdomMatrix16x1::Zero();

  weight_matrix_ = OdomMatrix16x16::Identity();

*/

  cbv_lhs_ = Eigen::MatrixXf::Zero(16,3);
  cbv_rhs_ = Eigen::MatrixXf::Zero(16,1);
  cbv_soln_= Eigen::MatrixXf::Zero(3,1);

  fit_lhs_ = Eigen::MatrixXf::Zero(16,3);
  fit_rhs_ = Eigen::MatrixXf::Zero(16,1);
  fit_soln_ = Eigen::MatrixXf::Zero(3,1);

  fit_residual_ =  Eigen::MatrixXf::Zero(16,1);
  odometry_residual_ = Eigen::MatrixXf::Zero(16,1);

  weight_matrix_ = Eigen::MatrixXf::Identity(16,16);

  
  robot_state_ = robot_state;

  if (odometry_publisher_ != NULL)// Make sure that we don't memory leak if initXml gets called twice
    delete odometry_publisher_ ;
  odometry_publisher_ = new realtime_tools::RealtimePublisher <pr2_msgs::Odometry> ("/base/"+odom_frame_, 1) ;

  if (transform_publisher_ != NULL)// Make sure that we don't memory leak if initXml gets called twice
    delete transform_publisher_ ;
  transform_publisher_ = new realtime_tools::RealtimePublisher <tf::tfMessage> ("~tf_message", 1) ;
  transform_publisher_->msg_.set_transforms_size(2);

  ROS_INFO("Odometry::initXml initialized");
  return true;
}

bool Odometry::starting()
{
  last_time_ = robot_state_->hw_->current_time_;
  last_publish_time_ = robot_state_->hw_->current_time_;
  return true;
}

robot_msgs::PoseDot Odometry::pointVel2D(const robot_msgs::Point& pos, const robot_msgs::PoseDot& vel)
{
  robot_msgs::PoseDot result;
  result.vel.vx = vel.vel.vx - pos.y * vel.ang_vel.vz;
  result.vel.vy = vel.vel.vy + pos.x * vel.ang_vel.vz;
  result.vel.vz = 0;
  return result;
}

void Odometry::update()
{
   current_time_ = robot_state_->hw_->current_time_;
   updateOdometry();
   publish();
   last_time_ = current_time_;
}

void Odometry::updateOdometry()
{
  double dt = current_time_-last_time_;
  double theta = odom_.z;
  double costh = cos(theta);
  double sinth = sin(theta);

  computeBaseVelocity();

  double odom_delta_x = (odom_vel_.vel.vx * costh - odom_vel_.vel.vx * sinth)*dt;
  double odom_delta_y = (odom_vel_.vel.vx * sinth + odom_vel_.vel.vy * costh)*dt;
  double odom_delta_th = odom_vel_.ang_vel.vz*dt;

  odom_.x += odom_delta_x;
  odom_.y += odom_delta_y;
  odom_.z += odom_delta_th;

  odometer_distance_ += sqrt(odom_delta_x*odom_delta_x + odom_delta_y*odom_delta_y);
  odometer_angle_ += fabs(odom_delta_th);
}

void Odometry::getOdometry(robot_msgs::Point &odom, robot_msgs::PoseDot &odom_vel)
{
   odom = odom_;
   odom_vel = odom_vel_;
   return;
}

void Odometry::getOdometryMessage(pr2_msgs::Odometry &msg)
{
  msg.header.frame_id   = odom_frame_;
  msg.header.stamp.fromSec(current_time_);
  msg.odom.position.x = odom_.x;
  msg.odom.position.y = odom_.y;
  msg.odom.position.z = 0.0;

  tf::Quaternion quat_trans = tf::Quaternion(odom_.z,0.0,0.0);
  msg.odom.orientation.x = quat_trans.x();
  msg.odom.orientation.y = quat_trans.y();
  msg.odom.orientation.z = quat_trans.z();
  msg.odom.orientation.w = quat_trans.w();

  msg.odom_vel = odom_vel_;
  msg.angle = odometer_angle_;
  msg.distance = odometer_distance_;
  msg.residual = odometry_residual_max_;
}

void Odometry::getOdometry(double &x, double &y, double &yaw, double &vx, double &vy, double &vw)
{
  x = odom_.x;
  y = odom_.y;
  yaw = odom_.z;
  vx = odom_vel_.vel.vx;
  vy = odom_vel_.vel.vy;
  vw = odom_vel_.ang_vel.vz;
}

void Wheel::updatePosition()
{
   robot_msgs::Point result = steer_offset_;
   double costh = cos(steer_->position_);
   double sinth = sin(steer_->position_);
   result.x += costh*offset_.x-sinth*offset_.y;
   result.y += sinth*offset_.x+costh*offset_.y;
   result.z = 0.0;
   position_ = result;
}

void Odometry::computeBaseVelocity()
{
  double steer_angle,wheel_speed,costh,sinth;
  robot_msgs::PoseDot caster_local_velocity;
  robot_msgs::PoseDot wheel_local_velocity;
  robot_msgs::Point wheel_position;

  for(int i = 0; i < num_wheels_; i++) 
  {
    wheel_[i].updatePosition();
    steer_angle = wheel_[i].steer_->position_;
    wheel_position = wheel_[i].position_;

    costh = cos(steer_angle);
    sinth = sin(steer_angle);
    wheel_speed = getCorrectedWheelSpeed(i);

    cbv_rhs_(i*2,0)   = wheel_radius_*wheel_speed;
    cbv_rhs_(i*2+1,0) = 0;

    cbv_lhs_(i*2, 0)   = costh;
    cbv_lhs_(i*2, 1)   = sinth;
    cbv_lhs_(i*2, 2)   = -costh * wheel_position.y + sinth * wheel_position.x;
    cbv_lhs_(i*2+1, 0)   = -sinth;
    cbv_lhs_(i*2+1, 1)   = costh;
    cbv_lhs_(i*2+1, 2)   = sinth * wheel_position.y + costh* wheel_position.x;
  }
  cbv_soln_ = iterativeLeastSquares(cbv_lhs_,cbv_rhs_,ils_weight_type_,ils_max_iterations_);

  odometry_residual_ = cbv_lhs_*cbv_soln_-cbv_rhs_;
  odometry_residual_max_ = odometry_residual_.cwise().abs().maxCoeff();

  odom_vel_.vel.vx = cbv_soln_(0,0);
  odom_vel_.vel.vy = cbv_soln_(1,0);
  odom_vel_.ang_vel.vz = cbv_soln_(2,0);
}

double Odometry::getCorrectedWheelSpeed(int index)
{
  double wheel_speed;
  robot_msgs::PoseDot caster_local_vel;
  robot_msgs::PoseDot wheel_local_vel;
  caster_local_vel.ang_vel.vz = wheel_[index].joint_->velocity_;
  wheel_local_vel = pointVel2D(wheel_[index].offset_,caster_local_vel);
  wheel_speed = wheel_[index].joint_->velocity_-wheel_local_vel.vel.vx/(wheel_radius_);
  return wheel_speed;
}

Eigen::MatrixXf Odometry::iterativeLeastSquares(Eigen::MatrixXf lhs, Eigen::MatrixXf rhs, std::string weight_type, int max_iter)
{
  weight_matrix_ = Eigen::MatrixXf::Identity(16,16);
  for(int i=0; i < max_iter; i++)
  {
    fit_lhs_ = weight_matrix_ * lhs;
    fit_rhs_ = weight_matrix_ * rhs;

    Eigen::MatrixXf mlhs(16,3);
    Eigen::MatrixXf mrhs(16,1);
    Eigen::VectorXf soln; 
//    mlhs.svd().solve(mrhs,&soln);

    fit_lhs_.svd().solve(fit_rhs_,&fit_soln_);


    fit_residual_ = rhs - lhs * fit_soln_;

    for(int j=0; j < num_wheels_; j++)
    {
      int fw = 2*j;
      int sw = fw+1;
      if(fabs(fit_residual_(fw,0)) > fabs(fit_residual_(sw,0)))
      {
        fit_residual_(fw,0) = fabs(fit_residual_(fw,0));
        fit_residual_(sw,0) = fit_residual_(fw,0);
      }
      else
      {
        fit_residual_(fw,0) = fabs(fit_residual_(sw,0));
        fit_residual_(sw,0) = fit_residual_(fw,0);
      }
    }
    weight_matrix_ = findWeightMatrix(fit_residual_,weight_type);
  }
  return fit_soln_;
};


Eigen::MatrixXf Odometry::findWeightMatrix(Eigen::MatrixXf residual, std::string weight_type)
{
  Eigen::MatrixXf w_fit = Eigen::MatrixXf::Identity(16,16);
  double epsilon = 0;
  double g_sigma = 0.1;

  if(weight_type == std::string("BubeLagan"))
  {
    for(int i=0; i < 2*num_wheels_; i++) 
    {
      if(fabs(residual(i,0) > epsilon))
        epsilon = fabs(residual(i,0));
    }
    epsilon = epsilon/100.0;
  }
  for(int i=0; i < 2*num_wheels_; i++) 
  {
    if(weight_type == std::string("L1norm"))
    {
      w_fit(i,i) = 1.0/(1 + sqrt(fabs(residual(i,0))));
    }
    else if(weight_type == std::string("fair"))
    {
      w_fit(i,i) = 1.0/(1 + fabs(residual(i,0)));
    }
    else if(weight_type == std::string("Cauchy"))
    {
      w_fit(i,i) = 1.0/(1 + residual(i,0)*residual(i,0));
    }
    else if(weight_type == std::string("BubeLangan"))
    {
      w_fit(i,i) = 1.0/pow((1 + pow((residual(i,0)/epsilon),2)),0.25);
    }
    else if(weight_type == std::string("Gaussian"))
    {
      w_fit(i,i) = sqrt(exp(-pow(residual(i,0),2)/(2*g_sigma*g_sigma)));
    }
    else // default to fair
    {
      w_fit(i,i) = 1.0/(0.1 + sqrt(fabs(residual(i,0))));
    }
  }
  return w_fit;
};

void Odometry::publish()
{
    if (odometry_publisher_->trylock())
    {
      getOdometryMessage(odometry_publisher_->msg_);
      odometry_publisher_->unlockAndPublish() ;
      last_publish_time_ = current_time_;
    }

    if (transform_publisher_->trylock())
    {
      double x(0.),y(0.0),yaw(0.0),vx(0.0),vy(0.0),vyaw(0.0);
      this->getOdometry(x,y,yaw,vx,vy,vyaw);

      robot_msgs::TransformStamped &out = transform_publisher_->msg_.transforms[0];
      out.header.stamp.fromSec(current_time_);
//      out.header.stamp.sec  = (unsigned long)floor(current_time_);
//      out.header.stamp.nsec = (unsigned long)floor(  1e9 * (current_time_ - out.header.stamp.sec) );

      out.header.frame_id = "odom";
      out.parent_id = "base_footprint";
      out.transform.translation.x = -x*cos(yaw) - y*sin(yaw);
      out.transform.translation.y = +x*sin(yaw) - y*cos(yaw);
      out.transform.translation.z = 0;
      tf::Quaternion quat_trans = tf::Quaternion(-yaw,0.0,0.0);

      out.transform.rotation.x = quat_trans.x();
      out.transform.rotation.y = quat_trans.y();
      out.transform.rotation.z = quat_trans.z();
      out.transform.rotation.w = quat_trans.w();

      robot_msgs::TransformStamped &out2 = transform_publisher_->msg_.transforms[1];
      out2.header.stamp.fromSec(current_time_);
//      out2.header.stamp.sec  = (unsigned long)floor(current_time_);
//      out2.header.stamp.nsec = (unsigned long)floor(  1e9 * (current_time_ - out.header.stamp.sec) );
      out2.header.frame_id = "base_link";
      out2.parent_id = "base_footprint";
      out2.transform.translation.x = 0;
      out2.transform.translation.y = 0;

      // FIXME: this is the offset between base_link origin and the ideal floor
      out2.transform.translation.z = 0.051; // FIXME: this is hardcoded, considering we are deprecating base_footprint soon, I will not get this from URDF.

      out2.transform.rotation.x = 0;
      out2.transform.rotation.y = 0;
      out2.transform.rotation.z = 0;
      out2.transform.rotation.w = 1;

      transform_publisher_->unlockAndPublish() ;
    }
};
