/////////////////////////////////////////////////////////////////////////////////////
//Software License Agreement (BSD License)
//
//Copyright (c) 2008, David Li, Melonee Wise, John Hsu
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
/*! \class CONTROLLER::JointController
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
#include <math_utils/angles.h>
#include <robot_model/joint.h>
#include <string>

using namespace mechanism;

namespace CONTROLLER
{
  class JointController : Controller
  {
    public:
    //---------------------------------------------------------------------------------//
    //CONSTRUCTION/DESTRUCTION CALLS
    //---------------------------------------------------------------------------------//

      /*!
        * \brief Default Constructor of the JointController class.
        *
        */
      JointController();
      
      /*!
        * \brief Destructor of the JointController class.
        */       
      ~JointController( );
    
       /*!
        * \brief   Initialization routine for the controller
        * \param Joint* joint The joint we are interacting with
        * \param string name The namespace identification in ROS
        */
  // CONTROLLER::CONTROLLER_ERROR_CODE Init(Joint* joint, string name);
    
      // JointController(Joint* joint, string name);

        //TEMPORARY
        /*! 
        * \brief Temporary way to initialize limits and gains. Default argument for dt is 1 ms
        *
        */
      void Init(double PGain, double IGain, double DGain, double IMax, double IMin, CONTROLLER_CONTROL_MODE mode, double time, double maxPositiveTorque, double maxNegativeTorque, double maxEffort, mechanism::Joint *joint, double dt);

   /*!
        * \brief   Initialization routine for the controller
        * \param Joint* joint The joint we are interacting with
        * \param string name The namespace identification in ROS
        */
  // CONTROLLER::CONTROLLER_ERROR_CODE Init(Joint* joint, string name);
    
      // JointController(Joint* joint, string name);

        //TEMPORARY
        /*! 
        * \brief Temporary way to initialize limits and gains. Default argument for dt is 1 ms
        *
        */
      void Init(double PGain, double IGain, double DGain, double IMax, double IMin, CONTROLLER_CONTROL_MODE mode, double time, double maxPositiveTorque, double maxNegativeTorque, double maxEffort, mechanism::Joint *joint);

//---------------------------------------------------------------------------------//
//TIME CALLS:w
//
//---------------------------------------------------------------------------------//
    /*!
        * \brief TODO: Get the actual time
        *  
        *
        * \param double* time Pointer to value to change 
        */
       void GetTime(double* time);

     
//---------------------------------------------------------------------------------//
//MODE/ENABLE CALLS
//---------------------------------------------------------------------------------//
    
    /*!
        * \brief Switches command mode type (Torque, position, velocity control)
        *  
        */
      void SetMode(CONTROLLER::CONTROLLER_CONTROL_MODE mode);

        /*!
        * \brief Returns the current mode of the controller
        *  
        */
      CONTROLLER::CONTROLLER_CONTROL_MODE GetMode(void);

        /*!
        * \brief Allow controller to run
        * 
        *  
        */

      void EnableController();


        /*!
        * \brief Set torque to zero. Prevent controller from running
        *
        *  
        */

      void DisableController();

        /*!
        * \brief Return true if last command saturated the torque 
        *
        *  
        */
      bool CheckForSaturation(void);



//---------------------------------------------------------------------------------//
//TORQUE CALLS
//---------------------------------------------------------------------------------//
        /*!
        * \brief Give a torque command to be issue on update (if in torque mode)
        *
        * \param torque Torque command to issue
        */

      CONTROLLER::CONTROLLER_ERROR_CODE SetTorqueCmd(double torque);
      
       /*!
        * \brief Fetch the latest user issued torque command 
        * 
        * \param double* torque Pointer to value to change 
        */ 
      CONTROLLER::CONTROLLER_ERROR_CODE GetTorqueCmd(double *torque);
      
      /*!
        * \brief Get the actual torque of the joint motor.
        * 
        * \param double* torque Pointer to value to change
        */  
      CONTROLLER::CONTROLLER_ERROR_CODE GetTorqueAct(double *torque);

//---------------------------------------------------------------------------------//
//POSITION CALLS
//---------------------------------------------------------------------------------//

      /*!
        * \brief Give set position of the joint for next update: revolute (angle) and prismatic (position)
        * 
        * \param double pos Position command to issue
        */       
      CONTROLLER::CONTROLLER_ERROR_CODE SetPosCmd(double pos);
      
      /*!
        * \brief Get latest position command to the joint: revolute (angle) and prismatic (position).
        * \param double* pos Pointer to value to change
        */       
      CONTROLLER::CONTROLLER_ERROR_CODE GetPosCmd(double *pos);
      
      /*!
        * \brief Read the torque of the motor
        * \param double* pos Pointer to value to change
        */       
      CONTROLLER::CONTROLLER_ERROR_CODE GetPosAct(double *pos);    

//---------------------------------------------------------------------------------//
//VELOCITY CALLS
//---------------------------------------------------------------------------------//
   
      /*!
        * \brief Set velocity command to the joint to be issue next update
        * \param double vel Velocity to issue next command
        */
      CONTROLLER::CONTROLLER_ERROR_CODE SetVelCmd(double vel);
      
      /*!
        * \brief Get latest velocity command to the joint
        * \param double* vel Pointer to value to change
        */
      CONTROLLER::CONTROLLER_ERROR_CODE GetVelCmd(double *vel);
      
      /*!
        * \brief Get actual velocity of the joint
        * \param double* vel Pointer to value to change
        */
      CONTROLLER::CONTROLLER_ERROR_CODE GetVelAct(double *vel);

//---------------------------------------------------------------------------------//
//UPDATE CALLS
//---------------------------------------------------------------------------------//
    /*!
        * \brief Issues commands to joint based on control mode
        *
        * 
        */

      //Issues commands to the joint. Should be called at regular intervals
      virtual void Update(void);

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
      CONTROLLER::CONTROLLER_ERROR_CODE SetParam(std::string label,double value);

   
   //  CONTROLLER::CONTROLLER_ERROR_CODE SetParam(std::string label,std::string value);

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
      CONTROLLER::CONTROLLER_ERROR_CODE GetParam(std::string label, double* value);
      //CONTROLLER::CONTROLLER_ERROR_CODE GetParam(std::string label, std::string value);

    private:

//---------------------------------------------------------------------------------//
//SAFETY CALLS
//---------------------------------------------------------------------------------//

          /*!
        * \brief Actually issue torque set command of the joint motor.
        * 
        *
        */       
     double SafelySetTorqueInternal(double torque);
     
      std::string jointName; /*!< Namespace ID for this controller>*/  
      mechanism::Joint* joint; /*!< Joint we're controlling>*/  
      Pid pidController; /*!< Internal PID controller>*/  

     
      double lastTime;/*!< Last time stamp of update> */
//---------------------------------------------------------------------------------//
// Command parameters
//---------------------------------------------------------------------------------//

      //Command parameters
      double cmdTorque;/*!< Last commanded torque*/
      double cmdPos;/*!< Last commanded position */
      double cmdVel;/*!< Last commanded Velocity */

//---------------------------------------------------------------------------------//
// Mode flags/parameters
//---------------------------------------------------------------------------------//

      bool SaturationFlag; /*!< Flag to indicate last command exceed torque limits and was truncated*/  
      bool enabled; /*!<Can controller issue commands?>*/
      CONTROLLER::CONTROLLER_CONTROL_MODE controlMode;    /*!< Indicate current controller mode (torque, position, velocity)*/  

//---------------------------------------------------------------------------------//
//TEMPORARY: To be replaced by calls to param server
//---------------------------------------------------------------------------------//

      double PGain; /*!< Proportional gain*/
      double IGain;/*!< Integral gain */
      double DGain;/*!< Derivative gain */
      double IMax;/*!< Upper integral clamp */
      double IMin;/*!< Lower integral clamp */

      double maxPositiveTorque; /*!<Temporary (until param server) : local copy of max Positive Torque.*/
      double maxNegativeTorque; /*!<Temporary (until param server): local copy of max neg torque .*/
      double maxEffort; /*!<Temporary (until param server): local copy of max possible commanded effort.*/

      double dt; /*!<Timestep amount. */
       };
}
