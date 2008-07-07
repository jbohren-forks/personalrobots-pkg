#pragma once
/***************************************************/
/*! \brief A PR2 Gripper controller
    
    This class implements controller loops for
    PR2 Gripper Control

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

class GripperController : CONTROLLER::Controller
{
  public:
  
    /*!
      * \brief Constructor of the GripperController class.
      *
      * \param 
      */
    GripperController();
    
    /*!
      * \brief Destructor of the GripperController class.
      */       
    ~GripperController( );

    /*!
      * \brief Set gap between the two finger tips.
      * 
      */       
    PR2::PR2_ERROR_CODE setGap(double distance);

    /*!
      * \brief Set force between the two finger tips.
      */       
    PR2::PR2_ERROR_CODE setForce(double force);
    
    /*!
      * \brief Close the gripper (force?).
      */       
    PR2::PR2_ERROR_CODE close();
    
    /*!
      * \brief Open the gripper.
      */       
    PR2::PR2_ERROR_CODE open();

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
      *<LI>   or setParam('maxGap',10);
      *<LI>   or setParam('maxForce',10);
      *<LI>   or setParam('closeForce',10);
      *</UL> 
      */
    PR2::PR2_ERROR_CODE setParam(string label,double value);
    PR2::PR2_ERROR_CODE setParam(string label,string value);
  private:
    PR2::PR2_CONTROL_MODE controlMode;      /**< Gripper controller control mode >*/
};


