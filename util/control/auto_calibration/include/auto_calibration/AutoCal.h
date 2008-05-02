#pragma once

#include <string.h>
#include <iostream>
#include <map>
#include <utility>
#include "etherdrive/etherdrive.h"
#include "XmlRpc.h"

/***************************************************/
/*! \brief A basic auto calibration class.
    
    

*/
/***************************************************/




class AutoCal
{
  public:
  
    /*!
      * \brief Constructor
      */
    AutoCal(EtherDrive &_e);
    
    /*!
      * \brief Destructor 
      */       
    ~AutoCal();

    /*!
      * \brief Run the auto calibration
      */
    double RunAutoCal(string object);  
    
 
    

  private:                              /**< Object to calibrate. */
    XmlRpc::XmlRpcValue paramMap;    /**< Motor information. */
    EtherDrive &e;                       /**< The motor driver you are hooked up to. */
};

