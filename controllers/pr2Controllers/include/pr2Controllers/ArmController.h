/////////////////////////////////////////////////////////////////////////////////////
//Software License Agreement (BSD License)
//
//Copyright (c) 2008, Willow Garage, Inc.
//All rights reserved.
//
//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions
//are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of the Willow Garage nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//POSSIBILITY OF SUCH DAMAGE.
/////////////////////////////////////////////////////////////////////////////////////

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

// to be replaced by xml
#include <pr2Core/pr2Core.h>

// generic controllers
#include <genericControllers/Controller.h>
#include <genericControllers/JointController.h>

// robot model
#include <mechanism_model/joint.h>
#include <mechanism_model/robot.h>

// kinematics library
#include <libKDL/kdl_kinematics.h>
#include <kdl/rotational_interpolation_sa.hpp>

// for temporary gettimeofday call, will be passed in from hardware interface and deprecate
#include <sys/time.h>

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

      static const double SYSTOSIMTIME =150;/**<Crude way to convert system to sim time at 100 hz>*/

//---------------------------------------------------------------------------------//
//CONSTRUCTION/DESTRUCTION CALLS
//---------------------------------------------------------------------------------//
      /*! 
        * \brief Constructor.
        *
        * \param 
        */
      ArmController();
      ArmController(mechanism::Robot *robot, std::string name);
      ArmController(mechanism::Robot *robot);
      ArmController(mechanism::Robot *robot, std::string name,int armNumJoints, int jcToRobotJointMap[], int jcToHIActuatorMap[]);

      /*!
        * \brief Destructor.
        */       
      ~ArmController( );
  
      /*!
        * \brief load XML file
        */       
      controllerErrorCode loadXML(std::string filename);
      
      /*!
        * \brief initialize controller variables
        */       
     void init(int jcToRobotJointMap[]=NULL);

      //TEMPORARY
        /*! 
        * \brief Temporary way to initialize limits and gains for a single joint. JointNum is Identified with enum
        *
        */
      void initJoint(int jointNum, double pGain, double iGain, double dGain, double iMax, double iMin, controllerControlMode mode, double time,double maxEffort,double minEffort, mechanism::Joint *joint);

      //TEMPORARY
        /*! 
        * \brief get system time, want to get simulator time for simulation.
        *
        */
     double getTime();

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
        * \brief Set Cartesian position of Hand (end-effector) in Global Frame (Euler Angles). Uses linear interpolation to smooth motion
        * 
        */     
        controllerErrorCode setHandCartesianPosLinear(double x, double y, double z, double roll, double pitch, double yaw, double timestep, double anglestep);

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
        * \brief Command cartesian location via Frame
        * 
        */    
        controllerErrorCode commandCartesianPos(KDL::Frame f);

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
       controllerErrorCode setHandParam(std::string label, std::string value);

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
      void update();

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

      /*!
        * \brief initialize joint controllers
        */       
      void initJointControllers(int jcToRobotJointMap[]);
      JointController *armJointControllers; /**< Lower level control done by JointControllers>*/

      double cmdPos[6]; /**<Last commanded cartesian position>*/
      double cmdVel[6]; /**<Last commanded cartesian velocity>*/
    
      PR2_kinematics pr2_kin; /**<PR2 kinematics used for cartesian commands>*/


      double pGain; /**< Proportional gain for position control */

      double iGain; /**< Integral gain for position control */

      double dGain; /**< Derivative gain for position control */

      double iMax; /**< Max integral error term */

      double iMin; /**< Min integral error term */

      double maxEffort; /**< maximum effort */

      double minEffort; /**< maximum effort */
        
      double xDotCmd; /**< Forward speed cmd */

      double yDotCmd; /**< Sideways speed cmd (motion to the left is positive) */

      double yawDotCmd; /**< Rotational speed cmd (motion counter-clockwise is positive) */

      double xDotNew; /**< New forward speed cmd */

      double yDotNew; /**< New sideways speed cmd (motion to the left is positive) */

      double yawDotNew; /**< New rotational speed cmd (motion counter-clockwise is positive) */

      double maxXDot;

      double maxYDot;

      double maxYawDot;

      mechanism::Robot *robot;

      std::map<std::string,std::string> paramMap;

      std::string name; /**<Namespace identifier for ROS>*/      

      void loadParam(std::string label, double &value);

      void loadParam(std::string label, int &value);

  
      //Linear interpolation parameters
      KDL::RotationalInterpolation_SingleAxis rotInterpolator;/**<Rotational interpolator for move>*/

      bool linearInterpolation; /**<Flag used to indicate current move should be linearly interpolated>*/
      double angleStep; /**<Size of angular motion to make>*/
      double lastTime;/**<Record time to last step>*/

      double timeStep;/**<Elapsed time between movements>*/
      int nSteps;/**<Number of substeps during movement>*/
      double stepSize; /**<Size of substep>*/
      int stepIndex;/**<Track progress of substeps>*/
      
      KDL::Frame endFrame; /**<Track desired end frame>**/

      KDL::Vector moveDirection;/**<Unit direction vector of move>*/
      KDL::Vector startPosition;/**<Starting location for move>*/
  };
}

    
