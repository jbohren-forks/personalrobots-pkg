#pragma once
/***************************************************/
/*! \brief A PR2 Arm controller
    
    This class implements controller loops for
    PR2 Arm Control

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


class ArmController : Controller
{
  public:
  
    /*!
      * \brief Constructor.
      *
      * \param 
      */
    ArmController();
    
    /*!
      * \brief Destructor.
      */       
    ~ArmController( );

    /*!
      * \brief Set yaw and pitch of head in Local Arm Frame
      * 
      */       
    PR2::PR2_ERROR_CODE setAngularPos(double yaw , double pitch);

    /*!
      * \brief Drive robot on a course in the Robot Frame
      * 
      * Same as setCourse except the inputs are the x and y components of velocities.
      *
      */       
    PR2::PR2_ERROR_CODE setCourseXY(double vx, double vy);

    /*!
      * \brief Set target point in Global Frame
      */
    PR2::PR2_ERROR_CODE setTarget(double x,double y, double yaw, double vx, double vy, double yawDot);

    /*!
      * \brief Set target points (trajectory list) in Global Frame
      */       
    PR2::PR2_ERROR_CODE setTraj(int numPts, double x[],double y[], double yaw[], double vx[], double vy[], double yawDot[]);

    /*!
      * \brief Set force (linear summation of all wheels) in global frame
      */       
    PR2::PR2_ERROR_CODE setForce(double fx, double fy);

    /*!
      * \brief Set parameters for this controller
      *
      * user can set maximum velocity,
      * maximum acceleration, and
      * constraints for this controller
      *
      * e.g.: <br>
      * <UL type="none">
      * <LI> setParam('maxVel',10);
      * <LI> setParam('maxAcc',10);
      * </UL>
      *
      */
    PR2::PR2_ERROR_CODE setParam(string label,double value);

  private:
    PR2::PR2_CONTROL_MODE controlMode;      /**< Arm controller control mode >*/
};


