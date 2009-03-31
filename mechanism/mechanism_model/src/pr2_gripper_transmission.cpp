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

  }

  for (TiXmlElement *j = config->FirstChildElement("passive_joint"); j; j = j->NextSiblingElement("passive_joint"))
  {
    const char *joint_name = j->Attribute("name");
    if (!joint_name || !robot->getJoint(joint_name))
    {
      ROS_WARN("PR2GripperTransmission could not find joint named \"%s\"", joint_name);
      return false;
    }
    else
    {
      // add joint name to list
      joint_names_.push_back(joint_name);
      passive_joints_.push_back(joint_name);

      // add a pid for mimic
      control_toolbox::Pid* pid = new control_toolbox::Pid();

      // initialize pid
      TiXmlElement *pel = j->FirstChildElement("gazebo_mimic_pid");
      if (pel)
      {
        pid->initXml(pel);
        std::cout << "found gazebo_mimic_pid" << std::endl;
      }
      else
      {
        pid->initPid(1.0,.0,.0,.0,.0);  // hardcoded defaults
      }

      // add to global list, one per passive joint
      pids_.push_back(pid);
    }

  }

  // time for updating pid loops
  current_time_ = robot->hw_->current_time_;
  last_time_    = robot->hw_->current_time_;

  return true;
}

///////////////////////////////////////////////////////////
/// given actuator states (motor revolustion, joint torques), compute gap properties.
void PR2GripperTransmission::computeGapStates(
  std::vector<Actuator*>& as, std::vector<JointState*>& js,
  double MR,double MR_dot,double MT,
  double &theta,double &dtheta_dMR,double &dt_dtheta,double &dt_dMR,double &gap_size,double &gap_velocity,double &gap_effort)
{
  ROS_ASSERT(as.size() == 1);
  ROS_ASSERT(js.size() == passive_joints_.size() + 1); // passive joints and 1 gap joint

  // rosen: sin(pos*reduction) = A*motor+B
  // rosen: reduction * cos(pos*reduction) * dpos/dt = A * dmotor/dt

  // enforce MR limit: greater than 0
  MR = MR >= 0.0 ? MR : 0.0;

  //
  // below transforms from encoder value to gap size, based on 090224_link_data.xls provided by Functions Engineering
  //
  double arg          = (coef_a_*coef_a_+coef_b_*coef_b_-pow(L0_+MR*screw_reduction_/gear_ratio_,2))/(2.0*coef_a_*coef_b_);
  arg                 = arg < -1.0 ? -1.0 : arg > 1.0 ? 1.0 : arg;
  theta               = theta0_ - phi0_ + acos(arg);
  // limit theta
  theta = theta > 0 ? theta : 0;
  // force theta to be greater than theta0_
  //theta = angles::normalize_angle_positive(angles::shortest_angular_distance(theta0_,theta))+theta0_;

  gap_size            = t0_ + coef_r_ * ( sin(theta) - sin(theta0_) );

  //
  // based on similar transforms, get the velocity of the gripper gap size based on encoder velocity
  //
  double arg_dot      = -(L0_ * screw_reduction_)/(gear_ratio_*coef_a_*coef_b_) // d(arg)/d(MR)
                               -screw_reduction_*MR*pow(screw_reduction_/gear_ratio_,2);
  double u            = 1.0 - pow(arg,2);
  u                   = u > TOL ? u : TOL; //LIMIT: CAP u at TOL artificially
  dtheta_dMR          = -1.0/sqrt(u) * arg_dot; // derivative of acos
  dt_dtheta           = coef_r_ * cos( theta );
  dt_dMR              = dt_dtheta * dtheta_dMR;
  gap_velocity        = MR_dot * dt_dMR;

  //
  // get the effort at the gripper gap based on torque at the motor
  //
  gap_effort          = MT      * dt_dMR;
}

///////////////////////////////////////////////////////////
/// given joint properties (theta), compute actuator states (motor revolutions, motor torques).
void PR2GripperTransmission::inverseGapStates(
  std::vector<Actuator*>& as, std::vector<JointState*>& js,
  double theta,double &MR,double &dMR_dtheta,double &dtheta_dt,double &dMR_dt)
{
  ROS_ASSERT(as.size() == 1);
  ROS_ASSERT(js.size() == passive_joints_.size() + 1); // passive joints and 1 gap joint

  // limit theta
  theta = theta > 0 ? theta : 0;
  // force theta to be greater than theta0_
  //theta = angles::normalize_angle_positive(angles::shortest_angular_distance(theta0_,theta))+theta0_;

  // now do the reverse transform
  double arg         = -2.0*coef_a_*coef_b_*cos(theta-theta0_+phi0_)
                                   -coef_h_*coef_h_+coef_a_*coef_a_+coef_b_*coef_b_;
  if (arg >= 0.0)
  {
    MR               = gear_ratio_/screw_reduction_ * ( sqrt(arg) - L0_ );
    dMR_dtheta       = gear_ratio_/(2.0 * screw_reduction_) / sqrt(arg)
                       * 2.0 * coef_a_ * coef_b_ * sin(theta + phi0_ - theta0_);
  }
  else
  {
    MR               = gear_ratio_/screw_reduction_ * ( 0.0       - L0_ );
    dMR_dtheta       = 0.0;
  }

  // enforce MR limit: greater than 0
  MR = MR >= 0.0 ? MR : 0.0;

  // compute gap_size_mm from theta
  double gap_size_mm = t0_ + coef_r_ * ( sin(theta) - sin(theta0_) ); // in mm

  // compute more inverse gradients
  double u           = 1.0 - pow((gap_size_mm - t0_)/coef_r_ + sin(theta0_),2);
  u                  = u > TOL ? u : TOL; //LIMIT: CAP u at TOL artificially
  dtheta_dt          = 1.0 / sqrt( u ) / coef_r_;  // derivative of asin
  dMR_dt             = dMR_dtheta * dtheta_dt;  // remember, here, t is gap_size

}

///////////////////////////////////////////////////////////
/// assign joint position, velocity, effort from actuator state
/// all passive joints are assigned by single actuator state through mimic?
void PR2GripperTransmission::propagatePosition(
  std::vector<Actuator*>& as, std::vector<JointState*>& js)
{

  ROS_ASSERT(as.size() == 1);
  ROS_ASSERT(js.size() == passive_joints_.size() + 1); // passive joints and 1 gap joint

  /// \brief motor revolutions ( = encoder value * gap_mechanical_reduction )
  double MR        = as[0]->state_.position_ / gap_mechanical_reduction_; // motor revolution
  double MR_dot    = as[0]->state_.velocity_ / gap_mechanical_reduction_; // revs per sec
  /// \brief gripper motor torque
  double MT        = as[0]->state_.last_measured_effort_ / gap_mechanical_reduction_;
  /// internal theta state, gripper closed it is theta0_.  same as finger joint angles + theta0_.
  double theta, dtheta_dMR, dt_dtheta, dt_dMR;
  /// information on the fictitious joint: gap_joint
  double gap_size,gap_velocity,gap_effort;

  // compute gap position, velocity, applied_effort from actuator states
  computeGapStates(as,js,MR,MR_dot,MT,theta,dtheta_dMR, dt_dtheta, dt_dMR,gap_size,gap_velocity,gap_effort);

  // assign joint states
  for (unsigned int i = 0; i < js.size(); ++i)
  {
    if (js[i]->joint_->name_ == gap_joint_)
    {
      // assign gap joint
      js[i]->position_       = gap_size;
      js[i]->velocity_       = gap_velocity;
      js[i]->applied_effort_ = gap_effort;
      // std::cout << "propagatePosition gap_joint js[" << i << "]:" << js[i]->joint_->name_
      //           << " MR:" << MR
      //           << " MR_dot:" << MR_dot
      //           << " MT:" << MT
      //           << " gap_size:" << gap_size
      //           << " gap_velocity:" << gap_velocity
      //           << " gap_effort:" << gap_effort
      //           << " theta:" << theta
      //           << " dtheta_dMR:" << dtheta_dMR
      //           << " dt_dtheta:" << dt_dtheta
      //           << " dt_dMR:" << dt_dMR
      //           << std::endl;
    }
    else
    {
      // find the passive joint name
      std::vector<std::string>::iterator it = std::find(passive_joints_.begin(),passive_joints_.end(),js[i]->joint_->name_);
      if (it != passive_joints_.end())
      {
        // assign passive joints
        js[i]->position_       = theta - theta0_;
        js[i]->velocity_       = dtheta_dMR * MR_dot                   ;
        js[i]->applied_effort_ = dtheta_dMR * MT                       ;
        // std::cout << "propagatePosition passive_joint js[" << i << "]:" << js[i]->joint_->name_
        //           << " theta:" << theta
        //           << " dtheta_dMR:" << dtheta_dMR
        //           << " MT:" << MT
        //           << " MR_dot:" << MR_dot
        //           << " js_pos:" << js[i]->position_
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
    //if (it == passive_joints_.begin())
    if (it != passive_joints_.end())
    {
      // assign passive joints
      mean_joint_angle    += angles::shortest_angular_distance(mean_joint_angle,js[i]->position_) + mean_joint_angle;
      mean_joint_rate     += js[i]->velocity_      ;
      mean_joint_torque   += js[i]->applied_effort_;
      count++;
      // std::cout << "propagatePositionBackwards js[" << i << "]:" << js[i]->joint_->name_
      //           << " mean_joint_angle:" << mean_joint_angle / count
      //           << " mean_joint_rate:" << mean_joint_rate / count
      //           << " mean_joint_torque:" << mean_joint_torque / count
      //           << " angle:"  << angles::shortest_angular_distance(mean_joint_angle,js[i]->position_) + mean_joint_angle
      //           << " rate:"   << js[i]->velocity_
      //           << " torque:" << js[i]->applied_effort_
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
  double theta            = angles::shortest_angular_distance(theta0_,avg_joint_angle)+2.0*theta0_;
  double MR,dMR_dtheta,dtheta_dt,dMR_dt;
  // compute inverse transform for the gap joint, returns MR and dMR_dtheta
  inverseGapStates(as,js,theta,MR,dMR_dtheta,dtheta_dt,dMR_dt);

  // std::cout << "propagatePositionBackwards average joint stuff:"
  //           << " avg_joint_angle:" << avg_joint_angle
  //           << " avg_joint_rate:" << avg_joint_rate
  //           << " avg_joint_torque:" << avg_joint_torque
  //           << " theta:" << theta
  //           << " MR:" << MR
  //           << " dMR_dtheta:" << dMR_dtheta
  //           << " dtheta_dt:" << dtheta_dt
  //           << " dMR_dt:" << dMR_dt
  //           << " gap_size from MR:" <<  t0_ + coef_r_ * ( sin(theta) - sin(theta0_) )
  //           << std::endl;

  as[0]->state_.position_             = MR                            * gap_mechanical_reduction_;
  as[0]->state_.velocity_             = avg_joint_rate   * dMR_dtheta * gap_mechanical_reduction_;
  // FIXME: this could potentially come from the tactile sensors
  // FIXME: is averaging finger torques and transmitting the thing to do?
  as[0]->state_.last_measured_effort_ = avg_joint_torque * dMR_dtheta * gap_mechanical_reduction_;
  //as[0]->state_.last_measured_effort_ = 0.0;

  // std::cout << "propagatePositionBackwards as[0]:"
  //           << " as_pos:"                  << as[0]->state_.position_
  //           << " as_vel:"                  << as[0]->state_.velocity_
  //           << " as_last_measured_effort:" << as[0]->state_.last_measured_effort_
  //           << std::endl;
}

void PR2GripperTransmission::propagateEffort(
  std::vector<JointState*>& js, std::vector<Actuator*>& as)
{
  ROS_ASSERT(as.size() == 1);
  ROS_ASSERT(js.size() == passive_joints_.size() + 1);

  // obtain the physical location of passive joints in sim, and average them, need this to compute dMR/dtheta
  // in hardware, the position of passive joints are set by propagatePosition, so they should be identical and
  // the inverse transform should be consistent.
  // FIXME:  check consistency
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
      std::cout << "passive joint propagateEffort js[" << i << "]:" << js[i]->joint_->name_
                << " mean_joint_angle:" << mean_joint_angle / count
                << " count:" << count
                << std::endl;
    }
    else
    {
      // std::cout << " js" << js[i]->joint_->name_ << " not a passive joint " << std::endl;
    }
  }
  
  double avg_joint_angle  = mean_joint_angle  / count;
  double theta            = theta0_ + avg_joint_angle; // should we filter this value?

  // now do the reverse transform
  double MR,dMR_dtheta,dtheta_dt,dMR_dt;
  // compute inverse transform for the gap joint, returns MR and dMR_dtheta
  inverseGapStates(as,js,theta,MR,dMR_dtheta,dtheta_dt,dMR_dt);

  // std::cout << "    "
  //           << " avg_joint_angle:" << avg_joint_angle
  //           << " theta:" << theta
  //           << " dMR_dtheta:" << dMR_dtheta
  //           << std::endl;

  // get the gap commanded effort
  for (unsigned int i = 0; i < js.size(); ++i)
  {
    if (js[i]->joint_->name_ == gap_joint_)
    {
      double js_gap_effort = js[i]->commanded_effort_; // added this variable for clarifying things for myself
      // go from gap linear effort (js[i]->commanded_effort_) to actuator effort = MT * gap_mechanical_reduction_
      as[0]->command_.effort_ = js_gap_effort * dMR_dt * gap_mechanical_reduction_;
      // std::cout << "propagateEffort " << " js[" << i << "]:" << js[i]->joint_->name_
      //           << " as cmd eff:" << as[0]->command_.effort_
      //           << " js_gap_effort:" << js_gap_effort
      //           << " gap mec red:" << gap_mechanical_reduction_
      //           << " dMR_dt:" << dMR_dt
      //           << std::endl;
      break; //FIXME: better be just one of these, need to check
    }
  }

  // assign avctuator commanded effort 
}

void PR2GripperTransmission::propagateEffortBackwards(
  std::vector<Actuator*>& as, std::vector<JointState*>& js)
{
  ROS_ASSERT(as.size() == 1);
  ROS_ASSERT(js.size() == passive_joints_.size() + 1);

  current_time_ = 0; //FIXME: get time!
  //
  // below transforms from encoder value to gap size, based on 090224_link_data.xls provided by Functions Engineering
  //
  /// \brief motor revolutions ( = encoder value * gap_mechanical_reduction )
  double MR        = as[0]->state_.position_ / gap_mechanical_reduction_; // motor revs
  double MR_dot    = as[0]->state_.velocity_ / gap_mechanical_reduction_; // revs per sec
  /// \brief gripper motor torque
  double MT        = as[0]->command_.effort_ / gap_mechanical_reduction_;

  /// internal theta state, gripper closed it is theta0_.  same as finger joint angles + theta0_.
  double theta, dtheta_dMR, dt_dtheta, dt_dMR;
  /// information on the fictitious joint: gap_joint
  double gap_size,gap_velocity,gap_effort;

  // compute gap position, velocity, applied_effort from actuator states
  computeGapStates(as,js,MR,MR_dot,MT,theta,dtheta_dMR, dt_dtheta, dt_dMR,gap_size,gap_velocity,gap_effort);

  //   // mimic joints to avg_MR.
  //   // obtain the physical location of passive joints in sim, and average them.
  //   double mean_joint_angle  = 0.0;
  //   double mean_joint_rate   = 0.0;
  //   double mean_joint_torque = 0.0;
  //   double count             = 0;
  //   // get average state avg_MR for enforcing mimic
  //   for (unsigned int i = 0; i < js.size(); ++i)
  //   {
  //     // find encoder value based on joint position, this really is just based on the single
  //     // physical joint with the encoder mounted on it.
  //     // find the passive joint name
  //     std::vector<std::string>::iterator it = std::find(passive_joints_.begin(),passive_joints_.end(),js[i]->joint_->name_);
  //     //if (it == passive_joints_.begin())
  //     if (it != passive_joints_.end())
  //     {
  //       // assign passive joints
  //       mean_joint_angle    += angles::shortest_angular_distance(mean_joint_angle,js[i]->position_) + mean_joint_angle;
  //       mean_joint_rate     += js[i]->velocity_      ;
  //       mean_joint_torque   += js[i]->applied_effort_;
  //       count++;
  //       // std::cout << "propagatePositionBackwards js[" << i << "]:" << js[i]->joint_->name_
  //       //           << " mean_joint_angle:" << mean_joint_angle / count
  //       //           << " mean_joint_rate:" << mean_joint_rate / count
  //       //           << " mean_joint_torque:" << mean_joint_torque / count
  //       //           << " angle:"  << angles::shortest_angular_distance(mean_joint_angle,js[i]->position_) + mean_joint_angle
  //       //           << " rate:"   << js[i]->velocity_
  //       //           << " torque:" << js[i]->applied_effort_
  //       //           << " count:" << count
  //       //           << std::endl;
  //     }
  //     else
  //     {
  //       // std::cout << " js[" << i << "]:" << js[i]->joint_->name_ << " not a passive joint " << std::endl;
  //     }
  //   }
  //   double avg_joint_angle  = mean_joint_angle  / count;
  //   double avg_joint_rate   = mean_joint_rate   / count;
  //   double avg_joint_torque = mean_joint_torque / count;
  //   double avg_theta            = angles::shortest_angular_distance(theta0_,avg_joint_angle)+2.0*theta0_;
  //   double avg_MR,avg_dMR_dtheta,avg_dtheta_dt,avg_dMR_dt;
  //   // compute inverse transform for the gap joint, returns MR and dMR_dtheta
  //   inverseGapStates(as,js,avg_theta,avg_MR,avg_dMR_dtheta,avg_dtheta_dt,avg_dMR_dt);


  // assign joint states
  for (unsigned int i = 0; i < js.size(); ++i)
  {
    if (js[i]->joint_->name_ == gap_joint_)
    {
      // assign gap joint
      // js[i]->commanded_effort_ = gap_effort; // do not need to propagate fictitious joint effort backwards
      // std::cout << "propagateEffortBackwards gap_joint js[" << i << "]:" << js[i]->joint_->name_
      //           << " MR:" << MR
      //           << " dtheta_dMR:" << dtheta_dMR
      //           << " MT:" << MT
      //           << " gap_effort:" << gap_effort
      //           << std::endl;
    }
    else
    {
      // find the passive joint name
      std::vector<std::string>::iterator it = std::find(passive_joints_.begin(),passive_joints_.end(),js[i]->joint_->name_);
      if (it != passive_joints_.end())
      {
        // enforce all gripper positions based on gap position
        // check to see how off each finger link is

        // now do the reverse transform
        // get individual passive joint error
        //double joint_theta              = theta0_ + angles::normalize_angle_positive(js[i]->position_);
        double joint_theta              = angles::shortest_angular_distance(theta0_,js[i]->position_) + 2.0*theta0_;
        // now do the reverse transform
        double joint_MR,joint_dMR_dtheta,joint_dtheta_dt,joint_dMR_dt;
        // compute inverse transform for the gap joint, returns MR and dMR_dtheta
        inverseGapStates(as,js,joint_theta,joint_MR,joint_dMR_dtheta,joint_dtheta_dt,joint_dMR_dt);

        // assign passive joints efforts
        //double finger_MR_error_    = joint_MR - avg_MR; // appears unstable due to tips much faster than upper fingers
        double finger_MR_error_    = joint_MR - as[0]->state_.position_ / gap_mechanical_reduction_;
        unsigned int index = it - passive_joints_.begin();
        double MIMICT = pids_[index]->updatePid(finger_MR_error_,0.001); //FIXME: get time and use current_time_ - last_time_

        // sum joint torques from actuator motor and mimic constraint
        js[i]->commanded_effort_   = dtheta_dMR*(MT + MIMICT);

        double pp,ii,dd,ii11,ii22;
        pids_[index]->getGains(pp,ii,dd,ii11,ii22);
        std::cout << "propagateEffortBackwards js[" << i << "]:" << js[i]->joint_->name_
                  << " i:" << index
                  << " actuator motor torque:" << MT
                  << " joint_MR:" << joint_MR
                  << " finger_MR_error_:" << finger_MR_error_
                  << " mimic torque:" << MIMICT
                  << " pid:" << pp
                  << "," << ii
                  << "," << dd
                  << " js_cmd_eff:" << js[i]->commanded_effort_
                  << std::endl;
      }
      else
      {
        // std::cout << " js[" << i << "]:" << js[i]->joint_->name_ << " not a gap nor passive joint " << std::endl;
      }
    }
  }
  last_time_ = current_time_;

}

} // namespace mechanism
