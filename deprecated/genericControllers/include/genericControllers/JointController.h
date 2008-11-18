/////////////////////////////////////////////////////////////////////////////////////
//Software License Agreement (BSD License)
//
//Copyright (c) 2008, Willow Garage, Inc.
//All rights reserved.
//
//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions
//are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of the Willow Garage nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
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
/////////////////////////////////////////////////////////////////////////////////////

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


#include <iostream>

#include <genericControllers/Controller.h>
#include <genericControllers/Pid.h>
#include <angles/angles.h>
#include <mechanism_model/joint.h>
#include <string>
#include <math.h>
#include <urdf/URDF.h>

using namespace mechanism;
using namespace std;

namespace controller
{

  class JointController : Controller
  {

    static const double AccelerationThreshold = 0.1; //Distance threshold below which linear acceleration limit is active, if enabled
    public:

    /*!
     * \brief Default Constructor of the JointController class.
     *
     */
    JointController();

    /*!
     * \brief Destructor of the JointController class.
     */
    ~JointController( );

    static Controller* create() {
      return new JointController;
    }

    /*!
     * \brief   Initialization routine for the controller
     * \param Joint* joint The joint we are interacting with
     * \param string name The namespace identification in ROS
     */
    // controller::controllerErrorCode Init(Joint* joint, string name);
    // JointController(Joint* joint, string name);

    /*!
     * \brief Functional way to initialize limits and gains.
     *
     */
    void init(pidControlParam pcp, controllerControlMode mode, double time, double maxEffort, double minEffort, mechanism::Joint *joint);

    /*!
     * \brief Functional way to initialize limits and gains.
     *
     */
    void init(double time, mechanism::Joint *joint);

    /*!
     * \brief   Initialization routine for the controller
     * \param Joint* joint The joint we are interacting with
     * \param string name The namespace identification in ROS
     */
    // controller::controllerErrorCode Init(mechanism::Joint* joint, string name);

    // JointController(Joint* joint, string name);

    /*!
     * \brief Functional way to initialize limits and gains.
     *
     */
    void init(double pGain, double iGain, double dGain, double windupMax, double windupMin, controllerControlMode mode, double time, double maxEffort, double minEffort, mechanism::Joint *joint);

    /*!
     * \brief TODO: Get the actual time
     *
     *
     * \param double* time Pointer to value to change
     */
    void getTime(double* time);

//---------------------------------------------------------------------------------//
//MODE/ENABLE CALLS
//---------------------------------------------------------------------------------//

    /*!
     * \brief Switches command mode type (Torque, position, velocity control)
     *
     */
    controllerControlMode setMode(controller::controllerControlMode mode);

    /*!
     * \brief Returns the current mode of the controller
     */
    controller::controllerControlMode getMode(void);

    /*!
     * \brief Allow controller to run
     */
    controller::controllerControlMode enableController();


    /*!
     * \brief Set torque to zero. Prevent controller from running
     */
    controller::controllerControlMode disableController();

    /*!
     * \brief Return true if last command saturated the torque
     *
     *
     */
    bool checkForSaturation(void);

//---------------------------------------------------------------------------------//
//TORQUE CALLS
//---------------------------------------------------------------------------------//
    /*!
     * \brief Give a torque command to be issue on update (if in torque mode)
     *
     * \param torque Torque command to issue
     */
    controller::controllerErrorCode setTorqueCmd(double torque);

    /*!
     * \brief Fetch the latest user issued torque command
     *
     * \param double* torque Pointer to value to change
     */
    controller::controllerErrorCode getTorqueCmd(double *torque);

    /*!
     * \brief Get the actual torque of the joint motor.
     *
     * \param double* torque Pointer to value to change
     */
    controller::controllerErrorCode getTorqueAct(double *torque);

//---------------------------------------------------------------------------------//
//POSITION CALLS
//---------------------------------------------------------------------------------//

    /*!
     * \brief Give set position of the joint for next update: revolute (angle) and prismatic (position)
     *
     * \param double pos Position command to issue
     */
    controller::controllerErrorCode setPosCmd(double pos);

    /*!
     * \brief Get latest position command to the joint: revolute (angle) and prismatic (position).
     * \param double* pos Pointer to value to change
     */
    controller::controllerErrorCode getPosCmd(double *pos);

    /*!
     * \brief Read the torque of the motor
     * \param double* pos Pointer to value to change
     */
    controller::controllerErrorCode getPosAct(double *pos);

//---------------------------------------------------------------------------------//
//VELOCITY CALLS
//---------------------------------------------------------------------------------//

    /*!
     * \brief Set velocity command to the joint to be issue next update
     * \param double vel Velocity to issue next command
     */
    controller::controllerErrorCode setVelCmd(double vel);

    /*!
     * \brief Get latest velocity command to the joint
     * \param double* vel Pointer to value to change
     */
    controller::controllerErrorCode getVelCmd(double *vel);

    /*!
     * \brief Get actual velocity of the joint
     * \param double* vel Pointer to value to change
     */
    controller::controllerErrorCode getVelAct(double *vel);

    /*!
     * \brief Compute max velocity coming into the end stop to stop with linear velocity in endstop.
     *
     */
    double getMaxVelocity(void);

//---------------------------------------------------------------------------------//
//UPDATE CALLS
//---------------------------------------------------------------------------------//
    /*!
     * \brief Issues commands to joint based on control mode
     *
     *
     */

    //Issues commands to the joint. Should be called at regular intervals
    virtual void update(void);

    virtual void update(double current_time);

//---------------------------------------------------------------------------------//
//PARAM SERVER CALLS
//---------------------------------------------------------------------------------//
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
    controller::controllerErrorCode setParam(std::string label,double value);
    //controller::controllerErrorCode SetParam(std::string label,std::string value);

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
    controller::controllerErrorCode getParam(std::string label, double* value);
    //controller::controllerErrorCode GetParam(std::string label, std::string value);

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

    bool capAccel;  /*!<Flag to indicate whether we should cap acceleration.>*/

    double maxAccel; /*!<Maximum allowed acceleration/deceleration.>*/

    /*!
     * \brief load parameters from the XML file
     *
     * \param std::string representation of the filename for the XML file with complete path
     */
    controller::controllerErrorCode loadXML(std::string filename);

    private:


    /*!
     * \brief Actually issue torque set command of the joint motor.
     */
    void setJointEffort(double torque);

    mechanism::Joint* joint; /*!< Joint we're controlling>*/

    Pid pidController; /*!< Internal PID controller>*/

    double lastTime;/*!< Last time stamp of update> */

    double cmdTorque;/*!< Last commanded torque>*/

    double cmdPos;/*!< Last commanded position> */

    double cmdVel;/*!< Last commanded Velocity> */

    bool saturationFlag; /*!< Flag to indicate last command exceed torque limits and was truncated>*/

    bool enabled; /*!<Can controller issue commands?>*/

    controller::controllerControlMode controlMode;    /*!< Indicate current controller mode (torque, position, velocity)>*/

    double pGain; /*!< Proportional gain>*/

    double iGain;/*!< Integral gain >*/

    double dGain;/*!< Derivative gain> */

    double windupMax;/*!< Upper integral clamp> */

    double windupMin;/*!< Lower integral clamp> */

    double maxEffort; /*!<Temporary (until param server) : local copy of max effort.>*/

    double minEffort; /*!<Temporary (until param server): local copy of min effort.>*/

    std::map<std::string,std::string> paramMap;

    void loadParam(std::string label, double &value);

    void loadParam(std::string label, int &value);

    std::string name; /*!< Namespace ID for this controller>*/

  };
}
