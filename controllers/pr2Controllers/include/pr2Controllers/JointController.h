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
      * \brief Set yaw and pitch of head in Local Head Frame
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
      * \brief Heading pose for the robot
      *
      * Robot assume a stationary rotation mode, and achieve heading in Global Frame
      * yaw=0 implies +x-axis direction,
      * +yaw implies counter-clockwise
      *
      */       
    PR2::PR2_ERROR_CODE setHeading(double yaw);

    /*!
      * \brief Set force (linear summation of all wheels) in global frame
      */       
    PR2::PR2_ERROR_CODE setForce(double fx, double fy);

    /*!
      * \brief Set parameters for this controller
      *
      * user can set maximum velocity
      * and maximum acceleration
      * constraints for this controller
      *
      * e.g. setParam('maxVel',10);
      *   or setParam('maxAcc',10);
      *
      */
    PR2::PR2_ERROR_CODE setParam(string label,double value);

  private:
    PR2::PR2_CONTROL_MODE controlMode;      /**< Head controller control mode >*/
};


