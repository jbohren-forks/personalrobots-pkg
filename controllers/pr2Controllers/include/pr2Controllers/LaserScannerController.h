#pragma once
/***************************************************/
/*! \class LaserScannerController
    \brief A PR2 Pitching Laser Scanner Joint controller
    
    This class implements controller loops for
    PR2 Pitching Laser Scanner Joint

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
#include <math.h>

#define EPSILON 0.001 //Threshold value for floating point comparisons in waveform generation
namespace controller
{
  class LaserScannerController : Controller
  {
    public:
    //---------------------------------------------------------------------------------//
    //CONSTRUCTION/DESTRUCTION CALLS
    //---------------------------------------------------------------------------------//

      /*!
        * \brief Constructor.
        *
        * \param 
        */
      LaserScannerController();
      
      /*!
        * \brief Destructor.
        */       
      ~LaserScannerController( ); 


        //TEMPORARY
        /*! 
        * \brief Temporary way to initialize limits and gains. 
        *
        */
      void init(double pGain, double iGain, double dGain, double windupMax, double windupMin, controllerControlMode mode, double time, double maxEffort, double minEffort, mechanism::Joint *joint);



    //---------------------------------------------------------------------------------//
    //AUTOMATIC PROFILE FUNCTIONS
    //---------------------------------------------------------------------------------//

     
        /*!
        * \brief Set automatic profile to sawtooth
        *
        */

      void setSawtoothProfile(double period, double amplitude, double dt, double offset);


       /*!
        * \brief Generate a sawtooth wave
        * 
        * 
        *
        */      
      void generateSawtooth(double *&x, double *&t, double period, double amplitude, double dt, double offset, unsigned int numElements);
	
        /*!
        * \brief Generate a sawtooth wave
        * 
        * 
        *
        */         
      void setSinewaveProfile(double period, double amplitude, double dt, double offset);

   

        /*!
        * \brief Generate a sawtooth wave
        * 
        * 
        *
        */         

      void generateSinewave(double *&x, double *&t, double period, double amplitude,double dt, double offset, unsigned int numElements);
      
        /*!
        * \brief Generate a sawtooth wave
        * 
        * 
        *
        */         

      void setSquarewaveProfile(double period, double amplitude, double dt, double offset);


      /*!
        * \brief Generate a square wave
        * 
        * 
        *
        */       
      void generateSquarewave(double *&x, double *&t, double period, double amplitude, double dt, double offset, unsigned int numElements);
 
//---------------------------------------------------------------------------------//
//TIME CALLS
//
//---------------------------------------------------------------------------------//
   /*!
        * \brief TODO: Get the actual time
        *  
        *
        * \param double* time Pointer to value to change 
        */
       void getTime(double* time);

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
      bool checkForSaturation(void);

//---------------------------------------------------------------------------------//
//TORQUE CALLS
//---------------------------------------------------------------------------------//

      /*!
      * \brief Give a torque command to be issue on update (if in torque mode)
      *
      * \param torque Torque command to issue
      */
    

      controllerErrorCode setTorqueCmd(double torque);

	    /*!
      * \brief Fetch the latest user issued torque command 
      * 
      * \param double* torque Pointer to value to change 
      */ 
      controllerErrorCode getTorqueCmd(double *torque);
    
      /*!
      * \brief Get the actual torque of the joint motor.
      * 
      * \param double* torque Pointer to value to change
      */  
      controllerErrorCode getTorqueAct(double *torque);

//---------------------------------------------------------------------------------//
//POSITION CALLS
//---------------------------------------------------------------------------------//

      /*!
      * \brief Give set position of the joint for next update: revolute (angle) and prismatic (position)
      * 
      * \param double pos Position command to issue. Pos = 0 is home position of horizontal scan
      */       
      controllerErrorCode setPosCmd(double pos);
    
      /*!
      * \brief Get latest position command to the joint: revolute (angle) and prismatic (position).
      * \param double* pos Pointer to value to change
      */       
      controllerErrorCode getPosCmd(double *pos);
    
      /*!
      * \brief Read the torque of the motor
      * \param double* pos Pointer to value to change
      */       
      controllerErrorCode getPosAct(double *pos);    
 
//---------------------------------------------------------------------------------//
//VELOCITY CALLS
//---------------------------------------------------------------------------------//
   
      /*!
      * \brief Set velocity command to the joint to be issue next update
      * \param double vel Velocity to issue next command
      */
      controllerErrorCode setVelCmd(double vel);
    
      /*!
      * \brief Get latest velocity command to the joint
      * \param double* vel Pointer to value to change
      */
      controllerErrorCode getVelCmd(double *vel);
    
      /*!
      * \brief Get actual velocity of the joint
      * \param double* vel Pointer to value to change
      */
      controllerErrorCode getVelAct(double *vel);
      
//---------------------------------------------------------------------------------//
//UPDATE CALLS
//---------------------------------------------------------------------------------//
    /*!
        * \brief Issues commands to joint based on control mode
        *
        * 
        */

      //Issues commands to the joint. Should be called at regular intervals
       virtual void update();

//---------------------------------------------------------------------------------//
//PARAM SERVER CALLS
//---------------------------------------------------------------------------------//

      /*!
        * \brief Set scanning profile for the pitching Hokuyo
        * 
        * To be determined on how to implement
        * For now, use sawtooth or sine wave
        * Consider also, using setParam('profile','sawtooth')
        *
        */       
      controllerErrorCode setProfile(double *&t, double *&x, int numElements);

      /*!
        * \brief Set parameters for this controller
        *
        * user can set maximum velocity
        * and maximum acceleration
        * constraints for this controller
        *
        * e.g. <br>
        * <UL type="none">
        * <LI> setParam('maxVel', 1.0);
        * <LI> setParam('maxAcc', 1.0);
        * <LI> setParam('profile','sawtooth');
        * <LI> setParam('profile','sinewave');
        * <LI> setParam('maxLim', 1.0);
        * <LI> setParam('minLim',-1.0);
        * <LI> setParam('rateHz'  , 1);
        * </UL>
        *
        */      
      controllerErrorCode setParam(std::string label,     double value);
      controllerErrorCode setParam(std::string label,std::string value);

    
        private:
      int counter; 
      bool enabled; /**< Track enabled stats>*/
      controllerControlMode controlMode; /**<Allow different control modes for hokuyo>*/

      std::string name; /**<Namespace identifier for ROS>*/      

      double* profileX; /**<Contains locations for profile>*/
      double* profileT; /**<Contains target times for profile>*/
      int profileIndex; /**<Track location in profile>*/
      int profileLength; /**<Number of points in one cycle>*/

      double lastCycleStart; //**<Start of the last cycle>*/      

      JointController  lowerControl; /**< Lower level control done by JointController>*/

      double cmdPos; /**<Last commanded position>*/
      double cmdVel; /**<Last commanded velocity>*/
      double cmdTorque; /**<Last commanded torque>*/
  };
}


