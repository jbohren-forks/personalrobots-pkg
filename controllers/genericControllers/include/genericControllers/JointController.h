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
    PR2::PR2_ERROR_CODE setPos(double pos);
    /*!
      * \brief Set the torque of the joint motor.
      * 
      *
      */       
    PR2::PR2_ERROR_CODE setTorq(double torq);

    /*!
      * \brief Set velocity of the joint: revolute (angular) and prismatic (linear).
      */
    PR2::PR2_ERROR_CODE setVel(double vel);


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
    PR2::PR2_ERROR_CODE setParam(string label,double value);
    PR2::PR2_ERROR_CODE setParam(string label,string value);
    
  private:
    PR2::PR2_CONTROL_MODE controlMode;      /**< Joint controller control mode >*/
};


