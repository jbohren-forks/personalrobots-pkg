/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Stuart Glaser
 */

#include "mechanism_model/pr2_gripper_transmission.h"
#include <algorithm>
#include <numeric>
#include <angles/angles.h>

namespace mechanism {

ROS_REGISTER_TRANSMISSION(PR2GripperTransmission)

bool PR2GripperTransmission::initXml(TiXmlElement *config, Robot *robot)
{
  const char *name = config->Attribute("name");
  name_ = name ? name : "";

  TiXmlElement *ael = config->FirstChildElement("actuator");
  const char *actuator_name = ael ? ael->Attribute("name") : NULL;
  if (!actuator_name || !robot->getActuator(actuator_name))
  {
    ROS_WARN("PR2GripperTransmission could not find actuator named \"%s\"", actuator_name);
    return false;
  }
  robot->getActuator(actuator_name)->command_.enable_ = true;
  actuator_names_.push_back(actuator_name);

  for (TiXmlElement *j = config->FirstChildElement("gap_joint"); j; j = j->NextSiblingElement("gap_joint"))
  {
    const char *joint_name = j->Attribute("name");
    if (!joint_name || !robot->getJoint(joint_name))
    {
      ROS_WARN("PR2GripperTransmission could not find joint named \"%s\"", joint_name);
      return false;
    }
    gap_joint_ = std::string(joint_name);
    joint_names_.push_back(joint_name);

    // get the mechanical reduction
    const char *joint_reduction = j->Attribute("mechanical_reduction");
    if (!joint_reduction)
    {
      ROS_WARN("PR2GripperTransmission's joint \"%s\" was not given a reduction.", joint_name);
      return false;
    }
    gap_mechanical_reduction_ = atof(joint_reduction);

    // get the nonlinear transmission parameters
    const char *pA = j->Attribute("A");
    if( pA != NULL )
        A_ = atof(pA);

    const char *pB = j->Attribute("B");
    if( pB != NULL )
        B_ = atof(pB);

    const char *pC = j->Attribute("C");
    if( pC != NULL )
        C_ = atof(pC);
  }

  for (TiXmlElement *j = config->FirstChildElement("passive_joint"); j; j = j->NextSiblingElement("passive_joint"))
  {
    const char *joint_name = j->Attribute("name");
    if (!joint_name || !robot->getJoint(joint_name))
    {
      ROS_WARN("PR2GripperTransmission could not find joint named \"%s\"", joint_name);
      return false;
    }
    joint_names_.push_back(joint_name);
    passive_joints_.push_back(joint_name);

  }

  return true;
}

///////////////////////////////////////////////////////////
/// assign joint position, velocity, effort from actuator state
/// all passive joints are assigned by single actuator state through mimic?
void PR2GripperTransmission::propagatePosition(
  std::vector<Actuator*>& as, std::vector<JointState*>& js)
{
  ROS_ASSERT(as.size() == 1);
  ROS_ASSERT(js.size() == passive_joints_.size() + 1); // passive joints and 1 gap joint

  // rosen: sin(pos*reduction) = A*motor+B
  // rosen: reduction * cos(pos*reduction) * dpos/dt = A * dmotor/dt

  //
  // below transforms from encoder value to gap size, based on 090224_link_data.xls provided by Functions Engineering
  //
  double encoder_value  = as[0]->state_.position_ * gap_mechanical_reduction_; // motor revs
  double arg            = (coef_a_*coef_a_+coef_b_*coef_b_-pow(L0_+encoder_value*screw_reduction_/gear_ratio_,2))/(2.0*coef_a_*coef_b_);
  arg = arg < -1.0 ? -1.0 : arg > 1.0 ? 1.0 : arg;
  double theta          = angles::from_degrees(angles::from_degrees(theta0_) - angles::from_degrees(phi0_)) + acos(arg);
  double gap_size_mm    = t0_ + coef_r_ * ( sin(theta) - sin(angles::from_degrees(theta0_)) ); // in mm
  double gap_size       = gap_size_mm / mm2m_; // in meters

  //
  // based on similar transforms, get the velocity of the gripper gap size based on encoder velocity
  //
  double encoder_value_dot   = as[0]->state_.velocity_ * gap_mechanical_reduction_; // revs per sec
  double arg_dot_mm          = -(L0_ * screw_reduction_)/(gear_ratio_*coef_a_*coef_b_) // d(arg)/d(encoder_value)
                               -screw_reduction_*encoder_value*pow(screw_reduction_/gear_ratio_,2);
  double arg_dot             = arg_dot_mm / mm2m_;
  double theta_dot           = -1.0/sqrt(1.0-pow(arg,2)) * arg_dot; // derivative of acos
  double gap_velocity        = encoder_value_dot * theta_dot;

  //
  // get the effort at the gripper gap based on torque at the motor
  //
  double theta_effort        = as[0]->state_.last_measured_effort_ * gap_mechanical_reduction_;
  double gap_effort          = theta_effort      * theta_dot;


  // assign joint states
  for (unsigned int i = 0; i < js.size(); ++i)
  {
    if (js[i]->joint_->name_ == gap_joint_)
    {
      // assign gap joint
      js[i]->position_       = gap_size;
      js[i]->velocity_       = gap_velocity;
      js[i]->applied_effort_ = gap_effort;
      // std::cout << "gap joint propagatePosition js[" << i << "]:" << js[i]->joint_->name_
      //           << " encoder_value:" << encoder_value
      //           << " encoder_value_dot:" << encoder_value_dot
      //           << " theta_effort:" << theta_effort
      //           << " gap_size:" << gap_size
      //           << " gap_velocity:" << gap_velocity
      //           << " gap_effort:" << gap_effort
      //           << " arg:" << arg
      //           << " theta:" << theta
      //           << " arg_dot:" << arg_dot
      //           << " theta_dot:" << theta_dot
      //           << std::endl;
    }
    else
    {
      // find the passive joint name
      std::vector<std::string>::iterator it = std::find(passive_joints_.begin(),passive_joints_.end(),js[i]->joint_->name_);
      if (it != passive_joints_.end())
      {
        // assign passive joints
        js[i]->position_       = theta - angles::from_degrees(theta0_) ;
        js[i]->velocity_       = theta_dot      ;
        js[i]->applied_effort_ = theta_effort      ;
        // std::cout << "passive joint propagatePosition js[" << i << "]:" << js[i]->joint_->name_
        //           << " arg:" << arg
        //           << " theta:" << theta
        //           << " theta_dot:" << theta_dot
        //           << " theta_effort:" << theta_effort
        //           << std::endl;
      }
      else
      {
        // std::cout << " js[" << i << "]:" << js[i]->joint_->name_ << " not a gap nor passive joint " << std::endl;
      }
    }
  }
}

// this is needed for simulation, so we can recover encoder value given joint angles
void PR2GripperTransmission::propagatePositionBackwards(
  std::vector<JointState*>& js, std::vector<Actuator*>& as)
{
  ROS_ASSERT(as.size() == 1);
  ROS_ASSERT(js.size() == passive_joints_.size() + 1);

  // obtain the physical location of passive joints in sim, and average them
  double mean_joint_angle  = 0.0;
  double mean_joint_rate   = 0.0;
  double mean_joint_torque = 0.0;
  double count             = 0;

  for (unsigned int i = 0; i < js.size(); ++i)
  {
    // find encoder value based on joint position, this really is just based on the single
    // physical joint with the encoder mounted on it.
    // find the passive joint name
    std::vector<std::string>::iterator it = std::find(passive_joints_.begin(),passive_joints_.end(),js[i]->joint_->name_);
    if (it != passive_joints_.end())
    {
      // assign passive joints
      mean_joint_angle    += angles::shortest_angular_distance(mean_joint_angle,js[i]->position_) + mean_joint_angle;
      mean_joint_rate     += js[i]->velocity_      ;
      mean_joint_torque   += js[i]->applied_effort_;
      count++;
      // std::cout << "passive joint propagatePositionBackwards js[" << i << "]:" << js[i]->joint_->name_
      //           << " mean_joint_angle:" << mean_joint_angle / count
      //           << " mean_joint_rate:" << mean_joint_rate / count
      //           << " mean_joint_torque:" << mean_joint_torque / count
      //           << " count:" << count
      //           << std::endl;
    }
    else
    {
      // std::cout << " js[" << i << "]:" << js[i]->joint_->name_ << " not a passive joint " << std::endl;
    }
  }
  
  double avg_joint_angle  = mean_joint_angle  / count;
  double avg_joint_rate   = mean_joint_rate   / count;
  double avg_joint_torque = mean_joint_torque / count;

  // now do the reverse transform
  double theta          = angles::from_degrees(theta0_) + avg_joint_angle; // should we filter this value?
  double arg            = sqrt(-2.0*coef_a_*coef_b_*cos(theta-angles::from_degrees(theta0_)+angles::from_degrees(phi0_))
                               -coef_h_*coef_h_+coef_a_*coef_a_+coef_b_*coef_b_);
  double encoder_value  =  gear_ratio_/screw_reduction_ * ( arg - L0_ );
  double dMR_dtheta_mm  = -gear_ratio_/(2.0 * screw_reduction_) / arg
                          * 2.0 * coef_a_ * coef_b_ * sin(theta + angles::from_degrees(phi0_) - angles::from_degrees(theta0_));
  double dMR_dtheta     = dMR_dtheta_mm / mm2m_;
  // std::cout << "    "
  //           << " avg_joint_angle:" << avg_joint_angle
  //           << " avg_joint_rate:" << avg_joint_rate
  //           << " avg_joint_torque:" << avg_joint_torque
  //           << " theta:" << theta
  //           << " arg:" << arg
  //           << " encoder_value:" << encoder_value
  //           << " dMR_dtheta:" << dMR_dtheta
  //           << std::endl;
  // std::cout << "check nan" << (-2.0*coef_a_*coef_b_*cos(theta-angles::from_degrees(theta0_)+angles::from_degrees(phi0_))-coef_h_*coef_h_+coef_a_*coef_a_+coef_b_*coef_b_) << " cos of:" << (theta-angles::from_degrees(theta0_)+angles::from_degrees(phi0_)) << std::endl;

  as[0]->state_.position_             = encoder_value                 / gap_mechanical_reduction_;
  as[0]->state_.velocity_             = avg_joint_rate   * dMR_dtheta / gap_mechanical_reduction_;
  as[0]->state_.last_measured_effort_ = avg_joint_torque * dMR_dtheta / gap_mechanical_reduction_;
}

void PR2GripperTransmission::propagateEffort(
  std::vector<JointState*>& js, std::vector<Actuator*>& as)
{
  ROS_ASSERT(as.size() == 1);
  ROS_ASSERT(js.size() == passive_joints_.size() + 1);

  // obtain the physical location of passive joints in sim, and average them, need this to compute dMR/dtheta
  double mean_joint_angle  = 0.0;
  double count             = 0;

  for (unsigned int i = 0; i < js.size(); ++i)
  {
    // find the passive joint name
    std::vector<std::string>::iterator it = std::find(passive_joints_.begin(),passive_joints_.end(),js[i]->joint_->name_);
    if (it != passive_joints_.end())
    {
      // assign passive joints
      mean_joint_angle    += angles::shortest_angular_distance(mean_joint_angle,js[i]->position_) + mean_joint_angle;
      count++;
      // std::cout << "passive joint propagateEffort js[" << i << "]:" << js[i]->joint_->name_
      //           << " mean_joint_angle:" << mean_joint_angle / count
      //           << " count:" << count
      //           << std::endl;
    }
    else
    {
      // std::cout << " js" << js[i]->joint_->name_ << " not a passive joint " << std::endl;
    }
  }
  
  double avg_joint_angle  = mean_joint_angle  / count;

  // now do the difficult reverse transform
  double theta          = angles::from_degrees(theta0_) + avg_joint_angle; // should we filter this value?
  double arg            = sqrt(-2.0*coef_a_*coef_b_*cos(theta-angles::from_degrees(theta0_)+angles::from_degrees(phi0_))
                               -coef_h_*coef_h_+coef_a_*coef_a_+coef_b_*coef_b_);
  double dMR_dtheta_mm  = -gear_ratio_/(2.0 * screw_reduction_) / arg
                          * 2.0 * coef_a_ * coef_b_ * sin(theta + angles::from_degrees(phi0_) - angles::from_degrees(theta0_));
  double dMR_dtheta     = dMR_dtheta_mm / mm2m_;

  // std::cout << "    "
  //           << " avg_joint_angle:" << avg_joint_angle
  //           << " theta:" << theta
  //           << " arg:" << arg
  //           << " dMR_dtheta:" << dMR_dtheta
  //           << std::endl;

  // get the gap commanded effort
  double gap_commanded_effort = 0.0;
  for (unsigned int i = 0; i < js.size(); ++i)
  {
    if (js[i]->joint_->name_ == gap_joint_)
    {
      gap_commanded_effort = js[i]->commanded_effort_ / gap_mechanical_reduction_;
      break; // better be just one of these, need to check
    }
  }

  // assign avctuator commanded effort 
  as[0]->command_.effort_ = gap_commanded_effort * dMR_dtheta / gap_mechanical_reduction_;
}

void PR2GripperTransmission::propagateEffortBackwards(
  std::vector<Actuator*>& as, std::vector<JointState*>& js)
{
  ROS_ASSERT(as.size() == 1);
  ROS_ASSERT(js.size() == passive_joints_.size() + 1);

  //
  // below transforms from encoder value to gap size, based on 090224_link_data.xls provided by Functions Engineering
  //
  double encoder_value  = as[0]->state_.position_ * gap_mechanical_reduction_; // motor revs
  double arg            = (coef_a_*coef_a_+coef_b_*coef_b_-pow(L0_+encoder_value*screw_reduction_/gear_ratio_,2))/(2.0*coef_a_*coef_b_);
  arg = arg < -1.0 ? -1.0 : arg > 1.0 ? 1.0 : arg;

  //
  // based on similar transforms, get the velocity of the gripper gap size based on encoder velocity
  //
  double arg_dot_mm          = -(L0_ * screw_reduction_)/(gear_ratio_*coef_a_*coef_b_) // d(arg)/d(encoder_value)
                               -screw_reduction_*encoder_value*pow(screw_reduction_/gear_ratio_,2);
  double arg_dot             = arg_dot_mm / mm2m_;
  double theta_dot           = -1.0/sqrt(1.0-pow(arg,2)) * arg_dot; // derivative of acos

  //
  // get the effort at the gripper gap based on torque at the motor
  //
  double theta_effort        = as[0]->command_.effort_ * gap_mechanical_reduction_;
  double gap_effort          = theta_effort      * theta_dot;


  // assign joint states
  for (unsigned int i = 0; i < js.size(); ++i)
  {
    if (js[i]->joint_->name_ == gap_joint_)
    {
      // assign gap joint
      js[i]->commanded_effort_ = gap_effort;
        // std::cout << "gap joint propagateEffortBackwards js[" << i << "]:" << js[i]->joint_->name_
        //           << " encoder_value:" << encoder_value
        //           << " arg:" << arg
        //           << " arg_dot:" << arg_dot
        //           << " theta_dot:" << theta_dot
        //           << " theta_effort:" << theta_effort
        //           << " gap_effort:" << gap_effort
        //           << std::endl;
    }
    else
    {
      // find the passive joint name
      std::vector<std::string>::iterator it = std::find(passive_joints_.begin(),passive_joints_.end(),js[i]->joint_->name_);
      if (it != passive_joints_.end())
      {
        // assign passive joints
        js[i]->commanded_effort_ = theta_effort   ;
        // std::cout << "passive joint propagateEffortBackwards js[" << i << "]:" << js[i]->joint_->name_
        //           << " theta_effort:" << theta_effort
        //           << std::endl;
      }
      else
      {
        // std::cout << " js[" << i << "]:" << js[i]->joint_->name_ << " not a gap nor passive joint " << std::endl;
      }
    }
  }

}

} // namespace mechanism
