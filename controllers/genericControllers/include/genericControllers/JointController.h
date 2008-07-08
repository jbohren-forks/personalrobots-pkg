
#pragma once
/***************************************************/
/*! \brief A PR2 Joint controller
    
    This class implements controller loops for
    PR2 Joint Control

*/
/***************************************************/
//#include <newmat10/newmat.h>
//#include <libKinematics/ik.h>
//#include <sys/types.h>
//#include <stdint.h>
//#include <string>
//#include <libKDL/kdl_kinematics.h> // for kinematics using KDL -- util/kinematics/libKDL


#include <iostream>
#include <genericControllers/Controller.h>
//#include <genericControllers/pid.h>


//#define SIMULATOR //Set flag to determine whether we're in the simulator 

#ifdef SIMULATOR
#include <gazebo/HingeJoint.hh>
#include <gazebo/SliderJoint.hh>
#endif
using namespace std;
using namespace CONTROLLER;

class JointController : Controller
{
  public:
  
    /*!
      * \brief Constructor of the JointController class.
      *
      * \param 
      */
    JointController();
    
    /*!
      * \brief Destructor of the JointController class.
      */       
    ~JointController( );
    
    /*!
      * \brief Set the torque of the joint motor.
      * 
      *
      */       
    CONTROLLER::CONTROLLER_ERROR_CODE setTorque(double torque);
    
		/*!
      * \brief Get the torque command of the joint motor.
      * 
      *
      */ 
    CONTROLLER::CONTROLLER_ERROR_CODE getTorqueCmd(double *torque);
    
    /*!
      * \brief Get the actual torque of the joint motor.
      * 
      *
      */  
    CONTROLLER::CONTROLLER_ERROR_CODE getTorqueAct(double *torque);

    /*!
      * \brief Set position of the joint: revolute (angle) and prismatic (position).
      * 
      */       
    CONTROLLER::CONTROLLER_ERROR_CODE setPos(double pos);
    
    /*!
      * \brief Set position of the joint: revolute (angle) and prismatic (position).
      * 
      */       
    CONTROLLER::CONTROLLER_ERROR_CODE getPosCmd(double *pos);
    
    /*!
      * \brief Set position of the joint: revolute (angle) and prismatic (position).
      * 
      */       
    CONTROLLER::CONTROLLER_ERROR_CODE getPosAct(double *pos);    
    
    /*!
      * \brief Set velocity of the joint: revolute (angular) and prismatic (linear).
      */
    CONTROLLER::CONTROLLER_ERROR_CODE setVel(double vel);
    
    /*!
      * \brief Set velocity of the joint: revolute (angular) and prismatic (linear).
      */
    CONTROLLER::CONTROLLER_ERROR_CODE getVelCmd(double *vel);
    
    /*!
      * \brief Set velocity of the joint: revolute (angular) and prismatic (linear).
      */
    CONTROLLER::CONTROLLER_ERROR_CODE getVelAct(double *vel);

    /*!
      * \brief Set parameters for this controller
      *
      * user can set maximum velocity
      * and maximum acceleration
      * constraints for this controller
      *<br> 
      *<UL TYPE="none">
      *<LI> e.g. setParam('maxVel',10);
      *<LI>   or setParam('maxAcc',10);
      *<LI>   or setParam('maxTorq',1);
      *<LI>   or setParam('type', 'prismatic');
      *</UL>
      */
    CONTROLLER::CONTROLLER_ERROR_CODE setParam(string label,double value);
    CONTROLLER::CONTROLLER_ERROR_CODE setParam(string label,string value);

   /*!
      * \brief Get parameters for this controller
      *
      * user can get maximum velocity
      * and maximum acceleration
      * constraints for this controller
      *<br> 
      *<UL TYPE="none">
      *<LI> e.g. getParam('maxVel',value);
      *<LI>   or getParam('maxAcc',value);
      *<LI>   or getParam('maxTorq',value);
      *<LI>   or getParam('type', stringValue);
      *</UL>
      */

    CONTROLLER::CONTROLLER_ERROR_CODE getParam(string label, double value);
    CONTROLLER::CONTROLLER_ERROR_CODE getParam(string label, string value);
    
    //Issues commands to the joint. Should be called at regular intervals
    virtual void Update(void);
	
    //Returns the current mode of the controller
    CONTROLLER::CONTROLLER_CONTROL_MODE GetMode(void);
  private:
/*	
    //Fetches the current time
    double GetTime(void);

//    //Joint of interest
//    Joint* myJoint;
    
    //Internal PID controller
    pid pidController;

    //Last time stamp of update
    double lastTime;

    //General parameters
    double maxVel; //max velocity
    double maxAcc; //max acceleration
    double maxTorq; //max torque

    string type;
    CONTROLLER::CONTROLLER_CONTROL_MODE controlMode;      
    //Control loop parameters- 
    double PGain;
    double IGain;
    double DGain;
    double IMax;
    double IMin;

    //Command parameters
    double cmdTorque;
    double cmdPos;
    double cmdVel;

    //Possibly needed interfaces
//    Robot* robot;
 //   Motor* motor;

*/
};


