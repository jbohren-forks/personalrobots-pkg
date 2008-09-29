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
#include <motor_qualification_controllers/motor_test2.h>

using namespace std;
using namespace controller;
using namespace NEWMAT;

ROS_REGISTER_CONTROLLER(MotorTest2)

MotorTest2::MotorTest2():
  publisher_("/diagnostics", 1), test_voltage_(5000),test_current_(5000),test_velocity_(5000),test_baseline_(5000), U_(5000), M_(5000)
{
  robot_ = NULL;
  actuator_ = NULL;
  joint_ = NULL;

  count_=1;
  duration_=4;
  torque_=0;
  speed_constant_=100;
  resistance_=10;
  initial_time_=0;
  complete = false;


}

MotorTest2::~MotorTest2()
{
}

void MotorTest2::init(double speed_constant, double resistance, double torque, std::string fixture_name, double time, std::string name ,mechanism::RobotState *robot)
{
  robot_ = robot;
  joint_ = robot->getJointState(name);
  actuator_ = robot->model_->getActuator("test_motor");
  fixture_joint_ =robot->getJointState(fixture_name);

  resistance_=resistance;
  torque_=torque;
  speed_constant_=speed_constant;
  initial_time_=time;

}

bool MotorTest2::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  assert(robot);
  robot_ = robot;

  TiXmlElement *j = config->FirstChildElement("joint");
  if (!j)
  {
    fprintf(stderr, "MotorTest2 was not given a joint\n");
    return false;
  }

  const char *joint_name = j->Attribute("name");
  joint_ = joint_name ? robot->getJointState(joint_name) : NULL;
  if (!joint_)
  {
    fprintf(stderr, "MotorTest2 could not find joint named \"%s\"\n", joint_name);
    return false;
  }

  TiXmlElement *cd = j->FirstChildElement("controller_defaults");
  if (cd)
  {
    double speed_constant = atof(cd->Attribute("speed_constant"));
    double resistance = atof(cd->Attribute("resistance"));
    double torque = atof(cd->Attribute("torque"));
    std::string fixture_name = cd->Attribute("fixture_name");
    init(speed_constant, resistance, torque, fixture_name, robot->hw_->current_time_,j->Attribute("name"), robot);
  }
  else
  {
    fprintf(stderr, "MotorTest2's config did not specify the default control parameters.\n");
  }
  return true;
}


double MotorTest2::getTime()
{
  return robot_->hw_->current_time_;
}

void MotorTest2::update()
{

  double time = robot_->hw_->current_time_;
  static int first_time = 1;
  static double zero_offset = 0;


  if ((time-initial_time_)<duration_) {
    test_baseline_(count_) = actuator_->state_.motor_voltage_;

    count_++;
  }
  else if((time-initial_time_)<2*duration_)
  {

    if(first_time)
    {
      double sign = test_baseline_(1) < 0 ? -1.0 : 1.0;
      zero_offset = sign * test_baseline_.NormFrobenius()/sqrt(count_);
      count_=1;
      first_time=0;
    }
    joint_->commanded_effort_ = torque_;
  }
  else if((time-initial_time_)<3*duration_)
  {
    joint_->commanded_effort_ = torque_;
    test_voltage_(count_) =actuator_->state_.motor_voltage_- zero_offset;
    test_current_(count_) = actuator_->state_.last_measured_current_;
    test_velocity_(count_) =joint_->velocity_;
    count_++;
  }
  else if (!complete)
  {
    joint_->commanded_effort_ = 0.0;
    analysis();
    complete = true;
  }
  else
    return;
}

void MotorTest2::analysis()
{
  // Screw realtime
  publisher_.lock();

  publisher_.msg_.set_status_size(1);
  robot_msgs::DiagnosticStatus *status = publisher_.msg_.status;
  status->set_values_size(2);
  status->name = "MotorTest";
  NEWMAT::Matrix test_matrix =test_velocity_ | test_current_;
  QRZ(test_matrix, U_);
  QRZ(test_matrix, test_voltage_, M_);
  NEWMAT::Matrix solution=U_.i()*M_;

  double speed_const_meas = fabs(1/solution(1,1));
  //ouble resistance_meas =fabs(solution(2,1)); this was all over the place no idea why??

  if (fabs(speed_const_meas-speed_constant_)/speed_constant_ > 0.05)
  {
    //the motor isn't moving
    status->level = 2;
    status->message = "ERROR: The motor is not correctly labeled. The speed constant or resistance is not correct.";
    status->values[0].label = "measured speed constant";
    status->values[0].value = speed_const_meas;
    status->values[1].label = "expected speed constant";
    status->values[1].value = speed_constant_;

   }
  else
  {
    //test passed
    status->level = 0;
    status->message = "OK: Passed.";
    status->values[0].label = "measured speed constant";
    status->values[0].value = speed_const_meas;
    status->values[1].label = "expected speed constant";
    status->values[1].value = speed_constant_;
  }

  publisher_.unlockAndPublish();

  return;
}

ROS_REGISTER_CONTROLLER(MotorTest2Node)
MotorTest2Node::MotorTest2Node()
{
  c_ = new MotorTest2();
}

MotorTest2Node::~MotorTest2Node()
{
  delete c_;
}

void MotorTest2Node::update()
{
  c_->update();
}


bool MotorTest2Node::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  std::string topic = config->Attribute("topic") ? config->Attribute("topic") : "";
  if (topic == "")
  {
    fprintf(stderr, "No topic given to MotorTest2Node\n");
    return false;
  }
  // Advertise topics
  if (!c_->initXml(robot, config))
    return false;
  return true;
}

