#pragma once
/***************************************************/
/*! \brief A PR2 Pitching Hokuyo Joint controller
    
    This class implements controller loops for
    PR2 Pitching Hokuyo Joint

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


class LaserScannerController : Controller
{
  public:
  
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

    /*!
      * \brief Set scanning profile for the pitching Hokuyo
      * 
      * To be determined on how to implement
      * For now, use sawtooth or sine wave
      * Consider also, using setParam('profile','sawtooth')
      *
      */       
    PR2::PR2_ERROR_CODE setProfile(double *t, double *x);

    /*!
      * \brief Set pitch angle of the pitching Hokuyo joint
      * 
      * pitch=0 means home position of horizontal scan.
      *
      */       
    PR2::PR2_ERROR_CODE SetPosition(double pitch);

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
    PR2::PR2_ERROR_CODE setParam(string label,double value);
    PR2::PR2_ERROR_CODE setParam(string label,string value);

  private:
    PR2::PR2_CONTROL_MODE controlMode;      /**< Pitching Hokuyo laser scanner controller control mode >*/
};


