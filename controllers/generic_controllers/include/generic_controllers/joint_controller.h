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
#pragma once
/***************************************************/
/*! \class controller::JointController
 \brief A Joint controller

 This class implements controller loops for
 PR2 Joint Control

 ASSUMES:
 Rotary joint

 Parameters to be set by SetParam
 PGain
 IGain
 DGain
 IMax
 IMin

 Parameters fetched from joint
 Time
 SaturationEffort
 MaxEffort

 Steps to bring a JointController online
 1. Initialize gains via SetParam
 2. Set the controller mode (SetMode(mode))
 3. EnableController()
 4. Set appropriate command (position, torque, velocity)
 5. Call Update from Real Time loop


 */
/***************************************************/
#include <sys/types.h>
#include <stdint.h>

#include <generic_controllers/controller.h>
#include <generic_controllers/pid.h>
#include <math_utils/angles.h>
#include <mechanism_model/joint.h>
#include <string>
#include <math.h>
#include <urdf/URDF.h>

namespace controller
{

class JointController : public Controller
{
  static const double ACCELERATION_THRESHOLD = 0.1; //Distance threshold below which linear acceleration limit is active, if enabled
public:
  /*!
   * \brief Default Constructor of the JointController class.
   *
   */
  JointController();

  /*!
   * \brief Destructor of the JointController class.
   */
  ~JointController();

  static Controller* create()
  {
    return new JointController();
  }

  void init(void);
  void initXml(mechanism::Robot *robot, TiXmlElement *config);

  /*!
   * \brief Functional way to initialize limits and gains.
   *
   */
  void init(PidControlParam pcp, ControllerControlMode mode, double time, double maxEffort, double minEffort,
            mechanism::Joint *joint);

  /*!
   * \brief Functional way to initialize limits and gains.
   *
   */
  void init(double time, mechanism::Joint *joint);

  /*!
   * \brief Functional way to initialize limits and gains.
   *
   */
  void init(double pGain, double iGain, double dGain, double windupMax, double windupMin, ControllerControlMode mode,
            double time, double maxEffort, double minEffort, mechanism::Joint *joint);

  //-------------------------------------------------------------------------//
  //MODE/ENABLE CALLS
  //-------------------------------------------------------------------------//

  /*!
   * \brief Switches command mode type (Torque, position, velocity control)
   *
   */
  ControllerControlMode setMode(ControllerControlMode mode);

  /*!
   * \brief Returns the current mode of the controller
   */
  ControllerControlMode getMode(void);

  /*!
   * \brief Allow controller to run
   */
  ControllerControlMode enableController();

  /*!
   * \brief Set torque to zero. Prevent controller from running
   */
  ControllerControlMode disableController();

  /*!
   * \brief Return true if last command saturated the torque
   *
   *
   */
  bool checkForSaturation(void);

  //-------------------------------------------------------------------------//
  //TORQUE CALLS
  //-------------------------------------------------------------------------//
  /*!
   * \brief Give a torque command to be issue on update (if in torque mode)
   *
   * \param torque Torque command to issue
   */
  ControllerErrorCode setTorqueCmd(double torque);

  /*!
   * \brief Fetch the latest user issued torque command
   *
   * \param double* torque Pointer to value to change
   */
  ControllerErrorCode getTorqueCmd(double *torque);

  /*!
   * \brief Get the actual torque of the joint motor.
   *
   * \param double* torque Pointer to value to change
   */
  ControllerErrorCode getTorqueAct(double *torque);

  //-------------------------------------------------------------------------//
  //POSITION CALLS
  //-------------------------------------------------------------------------//

  /*!
   * \brief Give set position of the joint for next update: revolute (angle) and prismatic (position)
   *
   * \param double pos Position command to issue
   */
  ControllerErrorCode setPosCmd(double pos);

  /*!
   * \brief Get latest position command to the joint: revolute (angle) and prismatic (position).
   * \param double* pos Pointer to value to change
   */
  ControllerErrorCode getPosCmd(double *pos);

  /*!
   * \brief Read the torque of the motor
   * \param double* pos Pointer to value to change
   */
  ControllerErrorCode getPosAct(double *pos);

  //-------------------------------------------------------------------------//
  //VELOCITY CALLS
  //-------------------------------------------------------------------------//

  /*!
   * \brief Set velocity command to the joint to be issue next update
   * \param double vel Velocity to issue next command
   */
  ControllerErrorCode setVelCmd(double vel);

  /*!
   * \brief Get latest velocity command to the joint
   * \param double* vel Pointer to value to change
   */
  ControllerErrorCode getVelCmd(double *vel);

  /*!
   * \brief Get actual velocity of the joint
   * \param double* vel Pointer to value to change
   */
  ControllerErrorCode getVelAct(double *vel);

  /*!
   * \brief Compute max velocity coming into the end stop to stop with linear velocity in endstop.
   *
   */
  double getMaxVelocity(void);

  //-------------------------------------------------------------------------//
  //UPDATE CALLS
  //-------------------------------------------------------------------------//
  /*!
   * \brief Issues commands to joint based on control mode
   *
   *
   */

  //Issues commands to the joint. Should be called at regular intervals
  virtual void update(void);

  //-------------------------------------------------------------------------//
  //PARAM SERVER CALLS
  //-------------------------------------------------------------------------//
  /*!
   * \brief Set parameters for this controller
   *
   * user can set maximum velocity
   * and maximum acceleration
   * constraints for this controller
   * \param string label Name of param to change
   * \param double value New value
   *<br>
   *<UL TYPE="none">
   *<LI> e.g. SetParam('PGain',10);
   *<LI>   or SetParam('IGain',10);
   *<LI>   or SetParam('DGain',1);
   *<LI>   or SetParam('IMax', 100);
   *<LI>   or SetParam('IMin',-100);
   *</UL>
   */
  ControllerErrorCode setParam(std::string label, double value);

  /*!
   * \brief Get parameters for this controller
   *
   * user can get maximum velocity
   * and maximum acceleration
   * constraints for this controller
   *<br>
   *<UL TYPE="none">
   *<UL TYPE="none">
   *<LI> e.g. GetParam('PGain',value);
   *<LI>   or GetParam('IGain',value);
   *<LI>   or GetParam('DGain',value);
   *<LI>   or GetParam('IMax', value);
   *<LI>   or GetParam('IMin', value);
   *</UL>
   */
  ControllerErrorCode getParam(std::string label, double* value);

  /*!
   * \brief Set the name of the controller
   *
   * \param name - std::string representation of the name of the controller
   */
  void setName(const std::string & name);

  /*!
   * \brief Return string name of the controller
   *
   * \return std::string representation of the name of the controller
   */
  std::string getName();

  bool cap_accel_; /*!<Flag to indicate whether we should cap acceleration.>*/
  double max_accel_; /*!<Maximum allowed acceleration/deceleration.>*/

  /*!
   * \brief load parameters from the XML file
   *
   * \param std::string representation of the filename for the XML file with complete path
   */
  ControllerErrorCode loadXML(std::string filename);

private:

  /*!
   * \brief Actually issue torque set command of the joint motor.
   */
  void setJointEffort(double torque);

  mechanism::Joint* joint_; /*!< Joint we're controlling>*/
  Pid pid_controller_; /*!< Internal PID controller>*/

  double last_time_;/*!< Last time stamp of update> */

  ControllerControlMode control_mode_; /*!< Indicate current controller mode (torque, position, velocity)>*/
  double cmd_torque_;/*!< Last commanded torque>*/
  double cmd_pos_;/*!< Last commanded position> */
  double cmd_vel_;/*!< Last commanded Velocity> */

  bool saturation_flag_; /*!< Flag to indicate last command exceed torque limits and was truncated>*/
  bool enabled_; /*!<Can controller issue commands?>*/

  double p_gain_; /*!< Proportional gain>*/
  double i_gain_;/*!< Integral gain >*/
  double d_gain_;/*!< Derivative gain> */
  double windup_max_;/*!< Upper integral clamp> */
  double windup_min_;/*!< Lower integral clamp> */
  double max_effort_; /*!<Temporary (until param server) : local copy of max effort.>*/
  double min_effort_; /*!<Temporary (until param server): local copy of min effort.>*/

  std::map<std::string,std::string> param_map_;
  void loadParam(std::string label, double &value);
  void loadParam(std::string label, int &value);
  std::string name_; /*!< Namespace ID for this controller>*/

  mechanism::Robot *robot_;
};

}
