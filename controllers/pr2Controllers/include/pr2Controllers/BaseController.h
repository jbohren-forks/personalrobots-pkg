#pragma once
/***************************************************/
/*! \brief A PR2 Base controller
    
    This class implements controller loops for
    PR2 Base Control

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


class BaseController
{
  public:
  
    /*!
      * \brief Constructor,
      *
      * \param 
      */
    BaseController();
    
    /*!
      * \brief Destructor of Pid class.
      */       
    ~BaseController( );

    /*!
      * \brief Drive robot on a course
      * 
      * Give the course in the robot frame, with theta=0 pointing forward.
      * e.g. setting theta=0 puts robot in a car-like mode.
      *
      *
      */       
    setCourse(double v , double theta);

    /*!
      * \brief Drive robot on a course
      * 
      * Same as setCourse except the inputs are the x and y components of velocities.
      *
      */       
    setCourseXY(double vx, double vy);

    /*!
      * \brief Set target point in Global Frame
      */       
    setTarget(double x,double y, double theta, double vx, double vy, double vw);

    /*!
      * \brief Set target points (trajectory list) in Global Frame
      */       
    setTraj(int num_pts, double x[],double y[], double theta[], double vx[], double vy[], double vw[]);

    /*!
      * \brief Heading pose for the robot
      *
      * Robot assume a stationary rotation mode, and achieve heading in Global Frame
      * theta=0 implies +x-axis direction,
      * +theta implies counter-clockwise
      *
      */       
    setHeading(double theta);

    /*!
      * \brief Set force (linear summation of all wheels) in global frame
      */       
    setForce(double fx, double fy);

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
    setParam(string label,double value);

  private:
    PR2::PR2_CONTROL_MODE controlMode;      /**< Base controller control mode >*/
};


