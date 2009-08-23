#ifndef LIBINST_AGILENT_34410A_HPP
#define LIBINST_AGILENT_34410A_HPP

#include "Interface.h"
#include "DMM.h"

/**
 * Interface to Agilent 34410A DMM
 * Should be able to GPIB or TCP/IP connection to DMM
 * @author cmeyers, dking 
 */
class Agilent34410A : public DMM
{
 public:
  explicit Agilent34410A(Interface &driver, int address=-1) throw();
  ~Agilent34410A();
  virtual void Initialize() throw (std::exception);
  virtual void Reset() throw (std::exception);
  virtual void RemoteMode(bool enable) throw (std::exception);
  virtual void Identify(std::string &result) throw (std::exception);
  virtual void setMode( DMM_Mode new_mode ) throw (std::exception);
  virtual void Select() throw (std::exception);
  virtual void Trigger() throw (std::exception);
  virtual double Measure() throw (std::exception);
  virtual void setRange(double range) throw (std::exception);
 private: 
  Interface &driver;
  int address;
  DMM_Mode current_mode;
  double last_range;
  std::string read_buf;
};


#endif //LIBINST_AGILENT_34410A_HPP
