#pragma once
/***************************************************/
/*! \brief A PR2 Head controller
    
    This class implements controller loops for
    PR2 Head Control

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


class HeadController : Controller
{
  public:
  
    /*!
      * \brief Constructor,
      *
      * \param 
      */
    HeadController();
    
    /*!
      * \brief Destructor of Pid class.
      */       
    ~HeadController( );

    /*!
      * \brief Set yaw and pitch of head in Local Frames
      * 
      * angles are defined by robot description file.
      * angle of 0 is the home position.
      *
      */       
    PR2::PR2_ERROR_CODE setPosition(double yaw , double pitch, double yawDot, double pitchDot);

    /*!
      * \brief Set yaw rate and pitch rate for the head in Local Frames
      * 
      */       
    PR2::PR2_ERROR_CODE setPositionRate(double yawDot, double pitchDot);

    /*!
      * \brief Set gaze point in global frame
      *
      * omit vx, vy, vz to denote 0 velocity at target point.
      *
      */
    PR2::PR2_ERROR_CODE setGazePoint(double x,double y, double z, double vx, double vy, double vz);
    PR2::PR2_ERROR_CODE setGazePoint(double x,double y, double z);

    /*!
      * \brief Set seccading speed
      *
      * Not sure how to seccade yet.
      *
      */       
    PR2::PR2_ERROR_CODE setSeccadeSpeed(double vx, double vy, double vz);

    /*!
      * \brief Set parameters for this controller
      *
      * user can set maximum velocity
      * and maximum acceleration
      * constraints for this controller
      *
      * e.g. setParam('maxVel', 1.0);
      *   or setParam('maxAcc', 1.0);
      *   or setParam('maxLim', 1.57);
      *   or setParam('minLim',-1.57);
      *
      */
    PR2::PR2_ERROR_CODE setParam(string label,double value);
    PR2::PR2_ERROR_CODE setParam(string label,string value);

  private:
    PR2::PR2_CONTROL_MODE controlMode;      /**< Head controller control mode >*/
};


