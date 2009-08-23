#ifndef LIBINST_INTERFACE_HPP
#define LIBINST_INTERFACE_HPP

#include <string>
#include <exception>

/**
 * Abstract Interface to SCIP device.
 * Possible Implementations include GPIB and TCP/IP (LXI)
 * @author cmeyers, dking
 */
class Interface { 
public:
  Interface( const std::string &device ) : device_string(device) { }  
  virtual ~Interface() { }
  virtual void Open() throw (std::exception)  =0;
  virtual void Close() throw (std::exception) =0;
  virtual void setAddress(int address) throw (std::exception) =0;
  virtual void Write(const std::string &msg) throw (std::exception) =0;
  virtual void Read(std::string &buf) throw (std::exception) =0;
  virtual void WriteRead(const std::string &msg, std::string &buf) throw (std::exception) =0;
protected: 
  std::string device_string;
};

#endif //LIBINST_INTERFACE_HPP

