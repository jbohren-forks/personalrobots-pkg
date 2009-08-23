#ifndef LIBINST_DMM_HPP
#define LIBINST_DMM_HPP

#include "Instrument.h"
#include <string>
#include <exception>

/*
 * Digital Multi-Meter (DMM) base class.
 * Provides a general interface for digital mutltimeters  
 */
class DMM : public Instrument
{
public:
  enum DMM_Mode { INVALID, VOLTAGE, CURRENT, RESISTANCE_2W, RESISTANCE_4W };  
  virtual void Initialize() throw (std::exception) = 0;
  virtual void Reset() throw (std::exception) = 0;
  virtual void RemoteMode(bool enable) throw (std::exception)= 0;
  virtual void Identify(std::string &result) throw (std::exception)= 0;
  virtual void setMode( DMM_Mode new_mode ) throw (std::exception) = 0;
  virtual void Select() throw (std::exception) = 0;
  virtual void Trigger() throw (std::exception) = 0;
  virtual double Measure() throw (std::exception) = 0;
  virtual void setRange(double range) throw (std::exception) = 0;
};


#endif //LIBINST_DMM_HPP
