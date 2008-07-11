
#pragma once
/***************************************************/
/*! \brief A PR2 Joint controller
    
    This class implements controller loops for
    PR2 Joint Control

*/
/***************************************************/
#include <newmat10/newmat.h>
#include <libKinematics/ik.h>
#include <sys/types.h>
#include <stdint.h>
#include <string>
#include <libKDL/kdl_kinematics.h> // for kinematics using KDL -- util/kinematics/libKDL

//#include <genericControllers/input_msg.h>
#include <iostream>
#include <genericControllers/Controller.h>
#include <genericControllers/Pid.h>
#include <libpr2API/pr2API.h>

using namespace std;
using namespace CONTROLLER;

class JointController : Controller
{
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

    /*!
     * \brief Temporary constructor for the simulator version. 
     *
     *
     */
JointController(double PGain, double IGain, double DGain, double IMax, double IMin, CONTROLLER::CONTROLLER_CONTROL_MODE mode, double time, double maxPositiveTorque, double maxNegativeTorque, double maxEffort);
  
 /*!
      * \brief Default Constructor of the JointController class.
      *
      * \param robot A pointer to the robot object
      * \param name A string identifying the joint and its namespace (e.g. ElbowPitch)
      */

	//Pass in robot model and namespace identification
//    JointController(Robot* robot, string name);
	JointController(string name);


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
    CONTROLLER::CONTROLLER_ERROR_CODE SetParam(string label,double value);

 
 //  CONTROLLER::CONTROLLER_ERROR_CODE SetParam(string label,string value);

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
    CONTROLLER::CONTROLLER_ERROR_CODE GetParam(string label, double* value);
    //CONTROLLER::CONTROLLER_ERROR_CODE GetParam(string label, string value);

    //Subscribe to receive setpoints
    
	
  /*!
      * \brief Issues commands to joint based on control mode
      *
      * 
      */

    //Issues commands to the joint. Should be called at regular intervals
    virtual void Update(void);

   
/*!
 *
 *\brief Manual update for simulator
 *
 *
 */
    double SimUpdate(double position, double velocity, double time);

      /*!
      * \brief TODO: Get the actual time
      *  
      *
      * \param double* time Pointer to value to change 
      */
       PR2::PR2_ERROR_CODE GetTime(double* time);
  
       /*!
      * \brief Returns the current mode of the controller
      *  
      */
    CONTROLLER::CONTROLLER_CONTROL_MODE GetMode(void);

  /*!
      * \brief Switches command mode type (Torque, position, velocity control)
      *  
      */
    void SetMode(CONTROLLER::CONTROLLER_CONTROL_MODE mode);
  /*!
      * \brief Return true if last command saturated the torque 
      *
      *  
      */
    bool CheckForSaturation(void);


 /*!
      * \brief Allow controller to run
      * 
      *  
      */

    void EnableController();


 /*!
      * \brief Set torque to zero. NOW.
      *
      *  
      */

    void DisableController();

  private:

        /*!
      * \brief Actually issue torque set command of the joint motor.
      * 
      *
      */       
   double SetTorque(double torque);
  /*!
      * \brief Callback for subscription to input_msg
      * 
      *
      */  
   // void InputCallback(void);
  /*!
      * \brief 
      * 
      *\param string name Identify the name of the subscription
      */  
   // void SubscribeToInput(string name);

    //genericControllers::input_msg inputBus; /*!< Provide a bus to read commands*/
 
    string jointName; /*!< Namespace ID for this controller*/  
    Joint* thisJoint; /*!< Joint we're controlling*/  
    Pid pidController; /*!< Internal PID controller*/  

    bool SaturationFlag; /*!< Flag to indicate last command exceed torque limits and was truncated*/  

    //Control loop parameters- 
    CONTROLLER::CONTROLLER_CONTROL_MODE controlMode;    /*!< Indicate current controller mode (torque, position, velocity)*/  
    double PGain; /*!< Proportional gain*/
    double IGain;/*!< Integral gain */
    double DGain;/*!< Derivative gain */
    double IMax;/*!< Upper integral clamp */
    double IMin;/*!< Lower integral clamp */

    double lastTime;/*!< Last time stamp of update */

    //Command parameters
    double cmdTorque;/*!< Last commanded torque*/
    double cmdPos;/*!< Last commanded position */
    double cmdVel;/*!< Last commanded Velocity */

    bool enabled; /*!<Can controller issue commands?>*/

    //Limits
    double maxPositiveTorque;
    double maxNegativeTorque;
    double maxEffort;
};


