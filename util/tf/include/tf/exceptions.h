#ifndef TF_EXCEPTIONS_H
#define TF_EXCEPTIONS_H

#include "ros_exception/exception.h"

namespace tf{

/** \brief A base class for all tf exceptions 
 * This inherits from ros::exception 
 * which inherits from std::runtime_exception
 */
class TFException: public ros::Exception
{ 
public:
  TFException(const std::string errorDescription) : ros::Exception(errorDescription) { ; };
};


  /** \brief An exception class to notify of no connection
   * 
   * This is an exception class to be thrown in the case 
   * that the Reference Frame tree is not connected between
   * the frames requested. */
class ConnectivityException:public TFException
{ 
public:
  ConnectivityException(const std::string errorDescription) : tf::TFException(errorDescription) { ; };
};


/** \brief An exception class to notify of bad frame number 
 * 
 * This is an exception class to be thrown in the case that 
 * a frame not in the graph has been attempted to be accessed.
 * The most common reason for this is that the frame is not
 * being published, or a parent frame was not set correctly 
 * causing the tree to be broken.  
 */
class LookupException: public TFException
{ 
public:
  LookupException(const std::string errorDescription) : tf::TFException(errorDescription) { ; };
};

  /** \brief An exception class to notify that the requested value would have required extrapolation beyond current limits.
   * 
   */
class ExtrapolationException: public TFException 
{ 
public:
  ExtrapolationException(const std::string errorDescription) : tf::TFException(errorDescription) { ; };
};


}


#endif //TF_EXCEPTIONS_H
