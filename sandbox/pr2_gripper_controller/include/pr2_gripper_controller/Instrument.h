#ifndef LIBINST_INSTRUMENT_HPP
#define LIBINST_INSTRUMENT_HPP

#include <string>
#include <exception>

/*
 * Generial GPIB/LXI(Ethernet) instrument base class.
 * @author cmeyers
 */
class Instrument {
public:
  virtual ~Instrument(void);
  virtual void Initialize() throw (std::exception) =0;
  virtual void Reset() throw (std::exception) =0;
  virtual void RemoteMode(bool enable) throw (std::exception) =0;
  virtual void Identify(std::string &result) throw (std::exception) =0;
  virtual void Select() throw (std::exception) =0; 
};


#endif // LIBINST_INSTRUMENT_HPP
