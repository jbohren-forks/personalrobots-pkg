#pragma once
/***************************************************/
/*! \class controller::ArmController
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
#include <libKDL/kdl_kinematics.h>
#include <libpr2API/pr2API.h> 
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
  
      /*!
        * \brief initialize controller variables
        */       
     void init();

      //TEMPORARY
        /*! 
        * \brief Temporary way to initialize limits and gains for a single joint. JointNum is Identified with enum
        *
        */
      void initJoint(int jointNum, double pGain, double iGain, double dGain, double iMax, double iMin, controllerControlMode mode, double time,double maxEffort,double minEffort, mechanism::Joint *joint);

     //TEMPORARY
        /*! 
        * \brief Call after initializing individual joints to initialize arm as a whole
        *         
        */
      controllerErrorCode initArm(controllerControlMode mode, PR2::PR2Robot* robot);



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
      controllerErrorCode setHandCartesianPos(double  x, double  y, double  z, double  roll, double  pitch, double  yaw);

        /*!
        * 
        * \brief Get commanded Cartesian position of Hand (end-effector) in Global Frame (Euler Angles)
        * 
        */      
       controllerErrorCode getHandCartesianPosCmd(double *x, double *y, double *z, double *roll, double *pitch, double *yaw);

        /*!
        * 
        * \brief Get Cartesian position of Hand (end-effector) in Global Frame (Euler Angles)
        * 
        */      
       controllerErrorCode getHandCartesianPosAct(double *x, double *y, double *z, double *roll, double *pitch, double *yaw);

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
       controllerErrorCode setArmJointPos(double angles[], double speeds[]);
  /*!
        *
        * \brief Get all commanded arm joint positions 
        *
        */

       controllerErrorCode getArmJointPosCmd(double *angles[], double *speeds[]);
     /*!
        *
        * \brief Get all arm joint positions 
        *
        */
       controllerErrorCode getArmJointPosAct(double *angles[], double *speeds[]);

   /*!
        *
        * \brief Set one arm joint position 
        *
        */
       controllerErrorCode setOneArmJointPos(int numJoint, double angle);
  /*!
        *
        * \brief Get one commanded arm joint position 
        *
        */

       controllerErrorCode getOneArmJointPosCmd(int numJoint, double *angle);
     /*!
        *
        * \brief Get one arm joint position 
        *
        */
       controllerErrorCode getOneArmJointPosAct(int numJoint, double *angle);

//---------------------------------------------------------------------------------//
//ARM JOINT TORQUE CALLS
//---------------------------------------------------------------------------------//

      /*!
        *
        * \brief Set all arm joint torques 
        *
        */
       controllerErrorCode setArmJointTorque( double torque[]);
       /*!
        *
        * \brief Get all commanded arm joint torques 
        *
        */

       controllerErrorCode getArmJointTorqueCmd( double *torque[]);
       /*!
        *
        * \brief Get all arm joint torques 
        *
        */

       controllerErrorCode getArmJointTorqueAct( double *torque[]);
   /*!
        *
        * \brief Set one arm joint torque
        *
        */
       controllerErrorCode setOneArmJointTorque(int numJoint,double torque);
       /*!
        *
        * \brief Get one commanded arm joint torque
        *
        */

       controllerErrorCode getArmJointTorqueCmd(int numJoint, double *torque);
       /*!
        *
        * \brief Get one arm joint torque
        *
        */

       controllerErrorCode getArmJointTorqueAct(int numJoint, double *torque);

//---------------------------------------------------------------------------------//
//ARM JOINT VELOCITY CALLS
//---------------------------------------------------------------------------------//

      /*!
        *
        * \brief Set all arm joint speeds 
        *
        */
       controllerErrorCode setArmJointSpeed( double speed[]);
  
        /*!
        *
        * \brief Get all commanded arm joint speeds 
        *
        */

       controllerErrorCode getArmJointSpeedCmd( double *speed[]);
      /*!
        *
        * \brief Get all arm joint speeds 
        *
        */
        controllerErrorCode getArmJointSpeedAct( double *speed[]);
 /*!
        *
        * \brief Set one arm joint speed
        *
        */
       controllerErrorCode setOneArmJointSpeed(int numJoint, double speed);
  
        /*!
        *
        * \brief Get one commanded arm joint speed
        *
        */

       controllerErrorCode getOneArmJointSpeedCmd(int numJoint, double *speed);
      /*!
        *
        * \brief Get one arm joint speed
        *
        */
        controllerErrorCode getOneArmJointSpeedAct(int numJoint, double *speed);

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
      void update( );

//---------------------------------------------------------------------------------//
// MISC CALLS
//---------------------------------------------------------------------------------//
       /*!
        * \brief Return the number of joints
        */  
      int getNumJoints();

    private:
      bool enabled;   /**<Is the arm controller active?>*/
      controllerControlMode controlMode;      /**< Arm controller control mode >*/

      std::string name; /**<Namespace identifier for ROS>*/      

      JointController lowerControl[ARM_MAX_JOINTS]; /**< Lower level control done by JointControllers>*/

      double cmdPos[6]; /**<Last commanded cartesian position>*/
      double cmdVel[6]; /**<Last commanded cartesian velocity>*/

      PR2::PR2Robot* robot; /**<Track robot for kinematics>*/ 
  };
}


