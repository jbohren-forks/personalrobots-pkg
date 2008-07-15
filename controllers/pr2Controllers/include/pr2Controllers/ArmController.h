#pragma once
/***************************************************/
/*! \class ArmController
    \brief A PR2 Arm controller
    
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
#include <genericControllers/Controller.h>

namespace CONTROLLER
{
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
        * \brief Update controller
        */       
      void Update( );

      /*!
        * 
        * \brief Set Cartesian position of Hand (end-effector) in Global Frame (Euler Angles)
        * 
        */       
      PR2::PR2_ERROR_CODE setHandCartesianPosition   (double  x, double  y, double  z, double  roll, double  pitch, double  yaw);

      PR2::PR2_ERROR_CODE getHandCartesianPositionCmd(double *x, double *y, double *z, double *roll, double *pitch, double *yaw);
      PR2::PR2_ERROR_CODE getHandCartesianPositionAct(double *x, double *y, double *z, double *roll, double *pitch, double *yaw);

      /*!
        * 
        * \brief Set orientation of Hand (end-effector) in Global Frame (Euler Angles)
        * 
        */       
      PR2::PR2_ERROR_CODE setHandOrientation(double roll, double pitch, double yaw);

      PR2::PR2_ERROR_CODE getHandOrientationCmd(double *roll, double *pitch, double *yaw);
      PR2::PR2_ERROR_CODE getHandOrientationAct(double *roll, double *pitch, double *yaw);

      /*!
        * 
        * \brief Set parameters for the hand (end-effector) controller
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
      PR2::PR2_ERROR_CODE setHandParam(string label, double value);
      PR2::PR2_ERROR_CODE setHandParam(string label, string value);

      PR2::PR2_ERROR_CODE getHandParam(string label, double *value);
      PR2::PR2_ERROR_CODE getHandParam(string label, string *value);

      /*!
        *
        * \brief Set Arm joint positions individually.
        *
        */
      PR2::PR2_ERROR_CODE setArmJointPosition(int numJoints, double angles[],double speed[]);

      PR2::PR2_ERROR_CODE getArmJointPositionCmd(int *numJoints, double *angles[],double *speed[]);
      PR2::PR2_ERROR_CODE getArmJointPositionAct(int *numJoints, double *angles[],double *speed[]);

      /*!
        *
        * \brief Set Arm joint torques individually.
        *
        */
      PR2::PR2_ERROR_CODE setArmJointTorque(int numJoints, double torque[]);

      PR2::PR2_ERROR_CODE getArmJointTorqueCmd(int *numJoints, double *torque[]);
      PR2::PR2_ERROR_CODE getArmJointTorqueAct(int *numJoints, double *torque[]);

      /*!
        *
        * \brief Set Arm joint speeds individually.
        *
        */
      PR2::PR2_ERROR_CODE setArmJointSpeed(int numJoints, double speed[]);

      PR2::PR2_ERROR_CODE getArmJointSpeedCmd(int *numJoints, double *speed[]);
      PR2::PR2_ERROR_CODE getArmJointSpeedAct(int *numJoints, double *speed[]);

      /*!
        *
        * \brief Set arm joint maximum torques individually.
        *
        */
      PR2::PR2_ERROR_CODE setArmJointMaxTorque(int numJoints, double maxTorque[]);

      PR2::PR2_ERROR_CODE getArmJointMaxTorqueCmd(int *numJoints, double *maxTorque[]);
      PR2::PR2_ERROR_CODE getArmJointMaxTorqueAct(int *numJoints, double *maxTorque[]);

      /*!
        *
        * \brief Set forearm camera gazepoints.
        *
        * TODO:  global frame???
        *
        */
      PR2::PR2_ERROR_CODE setArmCamGazePoint(double x, double y, double z);

      PR2::PR2_ERROR_CODE getArmCamGazePointCmd(double *x, double *y, double *z);
      PR2::PR2_ERROR_CODE getArmCamGazePointAct(double *x, double *y, double *z);

    private:
      PR2::PR2_CONTROL_MODE controlMode;      /**< Arm controller control mode >*/
  };
}


