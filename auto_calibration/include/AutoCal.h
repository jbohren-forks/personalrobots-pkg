#pragma once

#include <string.h>
#include <iostream>
#include <map>
#include <utility>
#include "etherdrive/etherdrive.h"

/***************************************************/
/*! \brief A basic auto calibration class.
    
    

*/
/***************************************************/

struct _Info
{
  int motornum;           /**< Motor to control. */
  double range;           /**< Total degrees of rotation. */
  double maxEncoder;      /**< The max encoder value. */    
  double minEncoder;      /**< The min encoder value. */
  double negOffset;       /**< Degrees negative of zero. */
  double posOffset;       /**< Degrees positive of zero. */
  int sign;               /**< Positive or negative in the sense of right handedness (1 or -1). */
};

typedef struct _Info Info;

class AutoCal
{
  public:
  
    /*!
      * \brief Constructor
      */
    AutoCal();
    
    /*!
      * \brief Destructor 
      */       
    ~AutoCal();

    /*!
      * \brief Run the auto calibration
      */
    double RunAutoCal();  
    
    /*!
      * \brief Initialize AutoCal with the proper values from the config file
      *
      * \param object What you want to calibrate 
      */
    void   InitAutoCal();  
    

  private:
    int object;                         /**< Object to calibrate. */
    multimap<string, Info> paramMap;    /**< Motor information. */
    EtherDrive e;                       /**< The motor driver you are hooked up to. */
};

