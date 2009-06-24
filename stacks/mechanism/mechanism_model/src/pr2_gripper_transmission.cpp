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
  double u            = (coef_a_*coef_a_+coef_b_*coef_b_-pow(L0_+MR*screw_reduction_/gear_ratio_,2))/(2.0*coef_a_*coef_b_);
  u                   = u < -1.0 ? -1.0 : u > 1.0 ? 1.0 : u;
  theta               = theta0_ - phi0_ + acos(u);
  // limit theta
  theta = theta > 0 ? theta : 0;
  // force theta to be greater than theta0_
  //theta = angles::normalize_angle_positive(angles::shortest_angular_distance(theta0_,theta))+theta0_;

  gap_size            = t0_ + coef_r_ * ( sin(theta) - sin(theta0_) );

  //
  // based on similar transforms, get the velocity of the gripper gap size based on encoder velocity
  //
  double arg          = 1.0 - pow(u,2);
  arg                 = arg > TOL ? arg : TOL; //LIMIT: CAP u at TOL artificially

  double du_dMR       = -(L0_ * screw_reduction_)/(gear_ratio_*coef_a_*coef_b_) // d(arg)/d(MR)
                               -screw_reduction_*MR*pow(screw_reduction_/gear_ratio_,2);

  dtheta_dMR          = -1.0/sqrt(arg) * du_dMR; // derivative of acos

  dt_dtheta           = coef_r_ * cos( theta );
  dt_dMR              = dt_dtheta * dtheta_dMR;
  gap_velocity        = MR_dot * dt_dMR;

  //
  // get the effort at the gripper gap based on torque at the motor
  //
  gap_effort          = MT      / dt_dMR;
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
  if (arg > 0.0)
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

  // compute gap_size from theta
  double gap_size = t0_ + coef_r_ * ( sin(theta) - sin(theta0_) ); // in mm

  // compute more inverse gradients
  double u           = (gap_size - t0_)/coef_r_ + sin(theta0_);
  double arg2        = 1.0 - pow(u,2);
  arg2               = arg2 > TOL ? arg2 : TOL; //LIMIT: CAP arg2 at TOL artificially
  dtheta_dt          = 1.0 / sqrt( arg2 ) / coef_r_;  // derivative of asin
  dMR_dt             = dMR_dtheta * dtheta_dt;  // remember, here, t is gap_size
  // std::cout << "debug reverse transform: "
  //           << " theta:" << theta
  //           << " MR:" << MR
  //           << " gap_size:" << gap_size
  //           << " u:" << u
  //           << " arg2:" << arg2
  //           << " dMR_dtheta:" << dMR_dtheta
  //           << " dtheta_dt:" << dtheta_dt
  //           << " dMR_dt:" << dMR_dt
  //           << std::endl;

}

///////////////////////////////////////////////////////////
/// assign joint position, velocity, effort from actuator state
/// all passive joints are assigned by single actuator state through mimic?
void PR2GripperTransmission::propagatePosition(
  std::vector<Actuator*>& as, std::vector<JointState*>& js)
{

  ROS_ASSERT(as.size() == 1);
  ROS_ASSERT(js.size() == passive_joints_.size() + 1); // passive joints and 1 gap joint

  /// \brief motor revolutions ( = encoder value * gap_mechanical_reduction_ * rad2mr_ )
  double MR        = as[0]->state_.position_ / gap_mechanical_reduction_ * rad2mr_ ; // motor revolution
  double MR_dot    = as[0]->state_.velocity_ / gap_mechanical_reduction_ * rad2mr_ ; // revs per sec
  /// \brief gripper motor torque: originally in newton-meters, but we convert it to Nm*(MR/rad)
  double MT        = as[0]->state_.last_measured_effort_ / gap_mechanical_reduction_ * rad2mr_ ;
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
      js[i]->position_       = gap_size*2.0; // function engineering's transmission give half the total gripper size
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
        js[i]->applied_effort_ = MT / dtheta_dMR                       ;
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


void PR2GripperTransmission::getAngleRateTorqueFromMinRateJoint(
  std::vector<JointState*>& js, std::vector<Actuator*>& as,
  int &minRateJointIndex,double &angle,double &rate,double &torque)
{

  // obtain the physical location of passive joints in sim, and average them
  angle  = 0.0;
  double minRate   = 1.0e16; // a ridiculously large value
  torque = 0.0;
  minRateJointIndex = js.size();

  for (unsigned int i = 0; i < js.size(); ++i)
  {
    // find encoder value based on joint position, this really is just based on the single
    // physical joint with the encoder mounted on it.
    // find the passive joint name
    std::vector<std::string>::iterator it = std::find(passive_joints_.begin(),passive_joints_.end(),js[i]->joint_->name_);
    //if (it == passive_joints_.begin())
    if (it != passive_joints_.end())
    {
      if (abs(js[i]->velocity_) < minRate)
      {
        minRate              = abs(js[i]->velocity_)     ;
        minRateJointIndex = i;

      }
      // std::cout << "propagatePositionBackwards js[" << i << "]:" << js[i]->joint_->name_
      //           << " minRateJointIndex:" << minRateJointIndex
      //           << " angle:"  << angles::shortest_angular_distance(theta0_,js[i]->position_) + theta0_
      //           << " rate:"   << js[i]->velocity_
      //           << " torque:" << js[i]->applied_effort_
      //           << std::endl;
    }
  }
  assert(minRateJointIndex < js.size()); // some joint rate better be smaller than 1.0e16

  angle             = angles::shortest_angular_distance(theta0_,js[minRateJointIndex]->position_) + theta0_;
  rate              = js[minRateJointIndex]->velocity_;
  torque            = js[minRateJointIndex]->applied_effort_;
}

// this is needed for simulation, so we can recover encoder value given joint angles
void PR2GripperTransmission::propagatePositionBackwards(
  std::vector<JointState*>& js, std::vector<Actuator*>& as)
{
  ROS_ASSERT(as.size() == 1);
  ROS_ASSERT(js.size() == passive_joints_.size() + 1);

  //FIXME: we want to keep the simulation stable by using the minimum rate joint, this is until
  //       I complete the correct mimic behavior w/o dynamics
  int minRateJointIndex;
  double joint_angle, joint_rate, joint_torque;
  getAngleRateTorqueFromMinRateJoint(js, as, minRateJointIndex, joint_angle, joint_rate, joint_torque);

  // recover gripper intermediate variable theta from joint angle
  double theta            = angles::shortest_angular_distance(theta0_,joint_angle)+2.0*theta0_;

  // compute inverse transform for the gap joint, returns MR and dMR_dtheta
  double MR,dMR_dtheta,dtheta_dt,dMR_dt;
  inverseGapStates(as,js,theta,MR,dMR_dtheta,dtheta_dt,dMR_dt);

  // std::cout << "propagatePositionBackwards average joint stuff:"
  //           << " joint_angle:" << joint_angle
  //           << " joint_rate:" << joint_rate
  //           << " joint_torque:" << joint_torque
  //           << " theta:" << theta
  //           << " MR:" << MR
  //           << " dMR_dtheta:" << dMR_dtheta
  //           << " dtheta_dt:" << dtheta_dt
  //           << " dMR_dt:" << dMR_dt
  //           << " gap_size from MR:" <<  t0_ + coef_r_ * ( sin(theta) - sin(theta0_) )
  //           << std::endl;

  // pass through gripper effort command to the actuator
  double gripper_effort;
  for (unsigned int i = 0; i < js.size(); ++i)
  {
    if (js[i]->joint_->name_ == gap_joint_)
    {
      gripper_effort = js[i]->commanded_effort_; // added this variable for clarifying things for myself
      as[0]->state_.position_             = MR                        * gap_mechanical_reduction_ / rad2mr_ ;
      as[0]->state_.velocity_             = joint_rate   * dMR_dtheta * gap_mechanical_reduction_ / rad2mr_ ;
      // FIXME: this could potentially come from the tactile sensors
      // FIXME: is averaging finger torques and transmitting the thing to do?
      // as[0]->state_.last_measured_effort_ = joint_torque * dMR_dtheta * gap_mechanical_reduction_ / rad2mr_ ;
      as[0]->state_.last_measured_effort_ = gripper_effort   / dMR_dt * gap_mechanical_reduction_ / rad2mr_ ;
      //as[0]->state_.last_measured_effort_ = 0.0;

      // std::cout << "propagatePositionBackwards as[0]:"
      //           << " as_pos:"                  << as[0]->state_.position_
      //           << " as_vel:"                  << as[0]->state_.velocity_
      //           << " as_last_measured_effort:" << as[0]->state_.last_measured_effort_
      //           << std::endl;
      return;
    }
  }
  ROS_ERROR("PropagatePositionBackwards for %s failed, actuator states unassigned.\n",gap_joint_.c_str());

}

void PR2GripperTransmission::propagateEffort(
  std::vector<JointState*>& js, std::vector<Actuator*>& as)
{
  ROS_ASSERT(as.size() == 1);
  ROS_ASSERT(js.size() == passive_joints_.size() + 1);

  // obtain the physical location of passive joints in sim, and average them, need this to compute dMR/dtheta
  // in hardware, the position of passive joints are set by propagatePosition, so they should be identical and
  // the inverse transform should be consistent.
  // FIXME: using the min rate joint for the sake of sim stability before true non-dynamics mimic is implemented
  int minRateJointIndex;
  double joint_angle, joint_rate, joint_torque;
  getAngleRateTorqueFromMinRateJoint(js, as, minRateJointIndex, joint_angle, joint_rate, joint_torque);
  
  // recover gripper intermediate variable theta from joint angle
  double theta            = angles::shortest_angular_distance(theta0_,joint_angle)+2.0*theta0_;

  // now do the reverse transform
  double MR,dMR_dtheta,dtheta_dt,dMR_dt;
  // compute inverse transform for the gap joint, returns MR and dMR_dtheta
  inverseGapStates(as,js,theta,MR,dMR_dtheta,dtheta_dt,dMR_dt);

  // std::cout << "    "
  //           << " joint_angle:" << joint_angle
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
      as[0]->command_.effort_ = js_gap_effort / dMR_dt * gap_mechanical_reduction_ / rad2mr_ ;
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
  double MR        = as[0]->state_.position_ / gap_mechanical_reduction_ * rad2mr_ ; // motor revs
  double MR_dot    = as[0]->state_.velocity_ / gap_mechanical_reduction_ * rad2mr_ ; // revs per sec
  /// \brief gripper motor torque
  double MT        = as[0]->command_.effort_ / gap_mechanical_reduction_ * rad2mr_ ;

  /// internal theta state, gripper closed it is theta0_.  same as finger joint angles + theta0_.
  double theta, dtheta_dMR, dt_dtheta, dt_dMR;
  /// information on the fictitious joint: gap_joint
  double gap_size,gap_velocity,gap_effort;

  // compute gap position, velocity, applied_effort from actuator states
  computeGapStates(as,js,MR,MR_dot,MT,theta,dtheta_dMR, dt_dtheta, dt_dMR,gap_size,gap_velocity,gap_effort);

  // // mimic joints to some --- mimic_MR.
  // // NOTE: this is comment out/necessary as we are using min rate angle in propagatePositionBackwards already
  // // FIXME: using the min rate joint for the sake of sim stability before true non-dynamics mimic is implemented
  // int minRateJointIndex;
  // double joint_angle, joint_rate, joint_torque;
  // getAngleRateTorqueFromMinRateJoint(js, as, minRateJointIndex, joint_angle, joint_rate, joint_torque);
  // 
  // double mimic_theta            = angles::shortest_angular_distance(theta0_,joint_angle)+2.0*theta0_;
  // double mimic_MR,mimic_dMR_dtheta,mimic_dtheta_dt,mimic_dMR_dt;
  // // compute inverse transform for the gap joint, returns MR and dMR_dtheta
  // inverseGapStates(as,js,mimic_theta,mimic_MR,mimic_dMR_dtheta,mimic_dtheta_dt,mimic_dMR_dt);

  // assign joint states
  for (unsigned int i = 0; i < js.size(); ++i)
  {
    if (js[i]->joint_->name_ == gap_joint_)
    {
      // assign gap joint
      js[i]->commanded_effort_ = gap_effort; // do not need to propagate fictitious joint effort backwards
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
        //double finger_MR_error_    = joint_MR - mimic_MR; // appears unstable due to tips much faster than upper fingers
        double finger_MR_error_    = joint_MR - as[0]->state_.position_ / gap_mechanical_reduction_ * rad2mr_ ;
        unsigned int index = it - passive_joints_.begin();
        double MIMICT = pids_[index]->updatePid(finger_MR_error_,0.001); //FIXME: get time and use current_time_ - last_time_

        // FIXME: hackery, due to transmission values, MT is too large for the damping available
        // with the given time step size in sim, so until implicit damping is implemented,
        // we'll scale MT with inverse of velocity
        double scale = exp(-abs(js[i]->velocity_*100.0));
        // sum joint torques from actuator motor and mimic constraint and convert to joint torques
        js[i]->commanded_effort_   = (scale*MT + MIMICT) / dtheta_dMR;

        double pp,ii,dd,ii11,ii22;
        pids_[index]->getGains(pp,ii,dd,ii11,ii22);
        // std::cout << "propagateEffortBackwards js[" << i << "]:" << js[i]->joint_->name_
        //           << " i:" << index
        //           << " actuator motor torque:" << MT
        //           << " joint_MR:" << joint_MR
        //           << " MR:" << as[0]->state_.position_ / gap_mechanical_reduction_ * rad2mr_
        //           << " finger_MR_error_:" << finger_MR_error_
        //           << " mimic torque:" << MIMICT
        //           << " pid:" << pp
        //           << "," << ii
        //           << "," << dd
        //           << " js_cmd_eff:" << js[i]->commanded_effort_
        //           << std::endl;
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