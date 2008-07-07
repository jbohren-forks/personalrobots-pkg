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
#include <pr2Core/pr2Core.h>
#include <libpr2HW/pr2HW.h>
#include <genericControllers/Controller.h>

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
    CONTROLLER::CONTROLLER_ERROR_CODE getPosAct(double pos);
    
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
      * \brief Set velocity of the joint: revolute (angular) and prismatic (linear).
      */
    CONTROLLER::CONTROLLER_ERROR_CODE setVel(double vel);


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
    
  private:
    CONTROLLER::CONTROLLER_CONTROL_MODE controlMode;      /**< Joint controller control mode >*/
};


