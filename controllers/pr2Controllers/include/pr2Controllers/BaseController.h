#pragma once
/***************************************************/
/*! \brief A PR2 Base controller
    
    This class implements controller loops for
    PR2 Base Control

    TODO:  naming configuration specification.

*/
/***************************************************/
//#include <newmat10/newmat.h>
//#include <libKinematics/ik.h>
//#include <sys/types.h>
//#include <stdint.h>
//#include <libKDL/kdl_kinematics.h> // for kinematics using KDL -- util/kinematics/libKDL

#include <string>
#include <iostream>

#include <pr2Core/pr2Core.h>
#include <pr2Core/pr2Misc.h>

//#include <libpr2HW/pr2HW.h>
#include <genericControllers/Controller.h>
#include <genericControllers/JointController.h>
#include <robot_model/joint.h>

#define BASE_NUM_JOINTS 12

using namespace std;

namespace CONTROLLER
{
  class BaseController : Controller
  {
    public:
    
      /*!
        * \brief Constructor of the BaseController class.
        *
        * \param 
        */
      BaseController();
      
      /*!
        * \brief Constructor of the BaseController class.
        *
        * \param 
        */
      BaseController(char *nbc);


      /*!
        * \brief Destructor of the BaseController class.
        */       
      ~BaseController( );

      /*!
        * \brief Update controller
        */       
      void Update( );

      /*!
        * \brief Initialize the controller
        */
      void Init();

      /*!
        * \brief Drive robot on a course
        * 
        * Give the course in the robot frame, with yaw=0 pointing forward.
        *
        * e.g. setting yaw=0 puts robot in a car-like mode.
        * 
        * \param v    The velocity of the robot.
        * \param yaw  The angle of the robot x-axis relative to the globe x-axis.
        *
        */       
      PR2::PR2_ERROR_CODE setCourse(double v , double yaw);

      /*!
        * \brief Drive robot on a course in the Robot Frame
        * 
        * Same as setCourse except the inputs are the x and y components of velocities.
        * \param xDot The velocity of the robot in the x-direction relative to the global frame.
        * \param yDot The velocity of the robot in the y-direction relative to the global frame.
        *
        * TODO:  is setVelocity a good name?
        *
        */       
      PR2::PR2_ERROR_CODE setVelocity(double xDot, double yDot, double yawDot);

      /*!
        * \brief Set target point in Global Frame
        *
        * TODO: this is a single point version of the setTrajectory function.
        *
        */
      PR2::PR2_ERROR_CODE setTarget(double x,double y, double yaw, double xDot, double yDot, double yawDot);

      /*!
        * \brief Set target points (trajectory list) in Global Frame
        *
        * TODO: define design requirements, see meeting notes (Mechanism Control minutes 2008-06-30.
        *
        */       
      PR2::PR2_ERROR_CODE setTraj(int numPts, double x[],double y[], double yaw[], double xDot[], double yDot[], double yawDot[]);

      /*!
        * \brief Heading pose for the robot
        *
        * Robot assume a stationary rotation mode, and achieve heading in Global Frame<br>
        * <UL type="none">
        * <LI> yaw=0 implies +x-axis direction,
        * <LI> +yaw implies counter-clockwise
        * </UL>
        *
        */       
      PR2::PR2_ERROR_CODE setHeading(double yaw);

      /*!
        * \brief Set force (linear summation of contributions from all wheels) in global frame
        */       
      PR2::PR2_ERROR_CODE setForce(double fx, double fy);

      /*!
        * \brief Set wrench (linear summation of contributions from all wheels) in global frame
        *
        * Use to apply a wrenching torque usig the base wheels.
        *
        */       
      PR2::PR2_ERROR_CODE setWrench(double yaw);

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
        *</UL>
        *
        */
      PR2::PR2_ERROR_CODE setParam(string label,double value);
      PR2::PR2_ERROR_CODE setParam(string label,string value);

    private:

      void InitJointControllers();

      JointController *baseJointControllers;

      Joint *baseJoints;

      char *ns;

      PR2::PR2_CONTROL_MODE controlMode; /*!< Base controller control mode */

      static const double PGain; /**< Proportional gain for speed control */

      static const double IGain; /**< Integral gain for speed control */

      static const double DGain; /**< Derivative gain for speed control */

      static const double IMax; /**< Max integral error term */

      static const double IMin; /**< Min integral error term */

      static const double maxPositiveTorque; /**< (in Nm) max current = 0.75 A. Torque constant = 70.4 mNm/A.Max Torque = 70.4*0.75 = 52.8 mNm */

      static const double maxNegativeTorque; /**< max negative torque */

      static const double maxEffort; /**< maximum effort */
      
      static const double PGain_Pos; /**< Proportional gain for position control */

      static const double IGain_Pos; /**< Integral gain for position control */

      static const double DGain_Pos; /**< Derivative gain for position control */

      double xDotCmd; /**< Forward speed cmd */

      double yDotCmd; /**< Sideways speed cmd (motion to the left is positive) */

      double yawDotCmd; /**< Rotational speed cmd (motion counter-clockwise is positive) */

      double xDotNew; /**< New forward speed cmd */

      double yDotNew; /**< New sideways speed cmd (motion to the left is positive) */

      double yawDotNew; /**< New rotational speed cmd (motion counter-clockwise is positive) */

      double GetTime();

  };
}

