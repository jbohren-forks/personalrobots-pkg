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




struct _Info  
{  
  int motorNum;           /**< Motor to control. */  
  double rotationRange;           /**< Total degrees of rotation. */  
  double maxEncoder;      /**< The max encoder value. */  
  double minEncoder;      /**< The min encoder value. */  
  double negOffset;       /**< Degrees negative of zero. */  
  double posOffset;       /**< Degrees positive of zero. */  
  int sign;               /**< Positive or negative in the sense of right handedness (1 or -1). */  
  int flag;  
  int count;

#define TOXMLRPCVAL(val) x[#val] = val;

  XmlRpc::XmlRpcValue toXmlRpcValue() {
    XmlRpc::XmlRpcValue x;
    TOXMLRPCVAL(motorNum)
    TOXMLRPCVAL(rotationRange)
    TOXMLRPCVAL(maxEncoder)
    TOXMLRPCVAL(minEncoder)
    TOXMLRPCVAL(negOffset)
    TOXMLRPCVAL(posOffset)
    TOXMLRPCVAL(sign)
    TOXMLRPCVAL(flag)
    TOXMLRPCVAL(count)
    return x;
  }

#define FROMXMLRPCVAL(val) if (x.hasMember(#val)) { val = x[#val]; } else { return false; }

  bool fromXmlRpcValue(XmlRpc::XmlRpcValue &x) {
    FROMXMLRPCVAL(motorNum)
    FROMXMLRPCVAL(rotationRange)
    FROMXMLRPCVAL(maxEncoder)
    FROMXMLRPCVAL(minEncoder)
    FROMXMLRPCVAL(negOffset)
    FROMXMLRPCVAL(posOffset)
    FROMXMLRPCVAL(sign)
    FROMXMLRPCVAL(flag)
    FROMXMLRPCVAL(count)
    return true;
  }
};

typedef struct _Info Info;


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
    void RunAutoCal(string object);  
    
 
    

  //  private:                              /**< Object to calibrate. */
    XmlRpc::XmlRpcValue paramMap;    /**< Motor information. */
    EtherDrive &e;                       /**< The motor driver you are hooked up to. */
};

