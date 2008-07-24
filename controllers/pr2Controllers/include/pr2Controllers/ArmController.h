#pragma once
/***************************************************/
/*! \class CONTROLLER::ArmController
    \brief A PR2 Arm controller
    
    This class implements controller loops for
    PR2 Arm Control

*/
/***************************************************/
//#include <newmat10/newmat.h>
//#include <libKinematics/ik.h>
//#include <sys/types.h>
//#include <stdint.h>
//#include <libKDL/kdl_kinematics.h> // for kinematics using KDL -- util/kinematics/libKDL

#include <iostream>

#include <pr2Core/pr2Core.h>
#include <genericControllers/Controller.h>
#include <genericControllers/JointController.h>
#include <mechanism_model/joint.h>



namespace controller
{
  class ArmController : Controller
  {
    public: 
    enum ARM_JOINT_ID{
      ARM_PAN         , 
      ARM_SHOULDER_PITCH, 
      ARM_SHOULDER_ROLL,
      ARM_ELBOW_PITCH , 
      ARM_ELBOW_ROLL  ,
      ARM_WRIST_PITCH , 
      ARM_WRIST_ROLL  ,
      ARM_MAX_JOINTS
    };

//---------------------------------------------------------------------------------//
//CONSTRUCTION/DESTRUCTION CALLS
//---------------------------------------------------------------------------------//
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
  
      //TEMPORARY
        /*! 
        * \brief Temporary way to initialize limits and gains for a single joint. JointNum is Identified with enum
        *
        */
      void initJoint(int jointNum, double PGain, double IGain, double DGain, double IMax, double IMin, controllerControlMode mode, double time, double maxPositiveTorque, double maxNegativeTorque, double maxEffort, mechanism::Joint *joint);

     //TEMPORARY
        /*! 
        * \brief Call after initializing individual joints to initialize arm as a whole
        *         
        */
      controllerErrorCode initArm(controllerControlMode mode);



//---------------------------------------------------------------------------------//
//MODE/ENABLE CALLS
//---------------------------------------------------------------------------------//
 
    /*!
        * \brief Switches command mode type (Automatic, Torque, position, velocity control)
        *  
        */
      controllerControlMode setMode(controllerControlMode mode);

    /*!
      * \brief Allow controller to send commands
      *      
      */
      controllerControlMode enableController(void);

        /*!
      * \brief Shut down controller.
      * 
      *       
      */
      controllerControlMode disableController(void);
      

        /*!
      * \brief Return controller mode
      * 
      */
    
      controllerControlMode getMode(void);

        /*!
        * \brief Return true if last command saturated the torque 
        *
        *  
        */
      void checkForSaturation(bool* status[]);


//---------------------------------------------------------------------------------//
//HAND CALLS
//---------------------------------------------------------------------------------//

        /*!
        * 
        * \brief Set Cartesian position of Hand (end-effector) in Global Frame (Euler Angles)
        * 
        */       
      controllerErrorCode setHandCartesianPosition   (double  x, double  y, double  z, double  roll, double  pitch, double  yaw);

        /*!
        * 
        * \brief Get commanded Cartesian position of Hand (end-effector) in Global Frame (Euler Angles)
        * 
        */      
       controllerErrorCode getHandCartesianPositionCmd(double *x, double *y, double *z, double *roll, double *pitch, double *yaw);

        /*!
        * 
        * \brief Get Cartesian position of Hand (end-effector) in Global Frame (Euler Angles)
        * 
        */      
       controllerErrorCode getHandCartesianPositionAct(double *x, double *y, double *z, double *roll, double *pitch, double *yaw);

        /*!
        * 
        * \brief Set orientation of Hand (end-effector) in Global Frame (Euler Angles)
        * 
        */       
       controllerErrorCode setHandOrientation(double roll, double pitch, double yaw);

        /*!
        * 
        * \brief Get commanded orientation of Hand (end-effector) in Global Frame (Euler Angles)
        * 
        */    
       controllerErrorCode getHandOrientationCmd(double *roll, double *pitch, double *yaw);
        
        /*!
        * 
        * \brief Get orientation of Hand (end-effector) in Global Frame (Euler Angles)
        * 
        */    
       controllerErrorCode getHandOrientationAct(double *roll, double *pitch, double *yaw);

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
       controllerErrorCode setHandParam(std::string label, double value);
       controllerErrorCode  setHandParam(std::string label, std::string value);

       controllerErrorCode getHandParam(std::string label, double *value);
       controllerErrorCode getHandParam(std::string label, std::string *value);

//---------------------------------------------------------------------------------//
//ARM JOINT POSITION CALLS
//---------------------------------------------------------------------------------//
      /*!
        *
        * \brief Set all arm joint positions 
        *
        */
       controllerErrorCode setArmJointPosition(int numJoints, double angles[],double speed[]);
  /*!
        *
        * \brief Get all commanded arm joint positions 
        *
        */

       controllerErrorCode getArmJointPositionCmd(int *numJoints, double *angles[],double *speed[]);
     /*!
        *
        * \brief Get all arm joint positions 
        *
        */
      controllerErrorCode getArmJointPositionAct(int *numJoints, double *angles[],double *speed[]);

//---------------------------------------------------------------------------------//
//ARM JOINT TORQUE CALLS
//---------------------------------------------------------------------------------//

      /*!
        *
        * \brief Set all arm joint torques 
        *
        */
       controllerErrorCode setArmJointTorque(int numJoints, double torque[]);
       /*!
        *
        * \brief Get all commanded arm joint torques 
        *
        */

       controllerErrorCode getArmJointTorqueCmd(int *numJoints, double *torque[]);
       /*!
        *
        * \brief Get all arm joint torques 
        *
        */

       controllerErrorCode getArmJointTorqueAct(int *numJoints, double *torque[]);

//---------------------------------------------------------------------------------//
//ARM JOINT VELOCITY CALLS
//---------------------------------------------------------------------------------//

      /*!
        *
        * \brief Set all arm joint speeds 
        *
        */
       controllerErrorCode setArmJointSpeed(int numJoints, double speed[]);
  
        /*!
        *
        * \brief Get all commanded arm joint speeds 
        *
        */

       controllerErrorCode getArmJointSpeedCmd(int *numJoints, double *speed[]);
      /*!
        *
        * \brief Get all arm joint speeds 
        *
        */
        controllerErrorCode getArmJointSpeedAct(int *numJoints, double *speed[]);

//---------------------------------------------------------------------------------//
// GAZE POINT CALLS
//---------------------------------------------------------------------------------//

      /*!
        *
        * \brief Set forearm camera gazepoints.
        *
        * TODO:  global frame???
        *
        */
       controllerErrorCode setArmCamGazePoint(double x, double y, double z);

      /*!
        *
        * \brief Get commanded forearm camera gazepoints.
        *
        * TODO:  global frame???
        *
        */

       controllerErrorCode getArmCamGazePointCmd(double *x, double *y, double *z);
     
      /*!
        *
        * \brief Get forearm camera gazepoints.
        *
        * TODO:  global frame???
        *
        */
       controllerErrorCode getArmCamGazePointAct(double *x, double *y, double *z);

//---------------------------------------------------------------------------------//
// UPDATE CALLS
//---------------------------------------------------------------------------------//


      /*!
        * \brief Update controller
        */       
      void Update( );


    private:
      bool enabled;   /**<Is the arm controller active?>*/
      controllerControlMode controlMode;      /**< Arm controller control mode >*/

      std::string name; /**<Namespace identifier for ROS>*/      

      JointController lowerControl[ARM_MAX_JOINTS]; /**< Lower level control done by JointControllers>*/

      double cmdPos[6]; /**<Last commanded cartesian position>*/
      double cmdVel[6]; /**<Last commanded cartesian velocity>*/

      
  };
}


      /*!
        *
        * \brief Set arm joint maximum torques individually.
        *
        */
     // PR2::PR2_ERROR_CODE setArmJointMaxTorque(int numJoints, double maxTorque[]);
      //PR2::PR2_ERROR_CODE getArmJointMaxTorqueCmd(int *numJoints, double *maxTorque[]);
      //PR2::PR2_ERROR_CODE getArmJointMaxTorqueAct(int *numJoints, double *maxTorque[]);

