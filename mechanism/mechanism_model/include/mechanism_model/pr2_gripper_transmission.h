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
 * <transmission type="PR2GripperTransmission" name="gripper_l_transmission">
 *   <actuator name="gripper_l_motor" />
 *   <joint name="gripper_l_upper1_joint" reduction="4" />
 *   <joint name="gripper_l_lower1_joint" reduction="-4" />
 *   <joint name="gripper_l_upper2_joint" reduction="-8" />
 *   <joint name="gripper_l_lower2_joint" reduction="8" />
 * </transmission>
 *
 * Author: Stuart Glaser
 */

#ifndef GRIPPER_TRANSMISSION_H
#define GRIPPER_TRANSMISSION_H

#include <vector>
#include "tinyxml/tinyxml.h"
#include "mechanism_model/transmission.h"
#include "mechanism_model/robot.h"

namespace mechanism {

class PR2GripperTransmission : public Transmission
{
public:
  PR2GripperTransmission() : A_(0), B_(1), C_(0) {}
  virtual ~PR2GripperTransmission() {}

  bool initXml(TiXmlElement *config, Robot *robot);

  void propagatePosition(std::vector<Actuator*>&, std::vector<JointState*>&);
  void propagatePositionBackwards(std::vector<JointState*>&, std::vector<Actuator*>&);
  void propagateEffort(std::vector<JointState*>&, std::vector<Actuator*>&);
  void propagateEffortBackwards(std::vector<Actuator*>&, std::vector<JointState*>&);

  std::string gap_joint_name_;
  //
  // per Functions Engineering doc, 090224_link_data.xml,
  // here, gap_mechanical_reduction_ transforms FROM ENCODER VALUE TO MOTOR REVOLUTIONS
  //
  double      gap_mechanical_reduction_;

  // store name for passive joints.
  std::vector<std::string> passive_joints_;

private:
  double A_, B_, C_ ; // gripper angle = reduction*acos(A*motor+B)

  //
  // SOME CONSTANTS
  // the default theta0 when gap size is 0 is needed to assign passive joint angles
  //
  // FIXME:  some of these are included implicitly in the code for now, if performance does not suffer,
  // replace all constants with below.
  //
  static const double t0_                  = -0.19543;
  static const double theta0_              = 2.97571;
  static const double phi0_                = 29.98717;
  static const double gear_ratio_          = 29.16; //729.0/25.0;
  static const double screw_reduction_     = 2.0;
  static const double L0_                  = 34.70821;
  static const double coef_h_              = 5.200;
  static const double coef_a_              = 67.56801;
  static const double coef_b_              = 48.97193;
  static const double coef_r_              = 91.50000;
  static const double mm2m_                = 1000.000;

};

} // namespace mechanism

#endif
