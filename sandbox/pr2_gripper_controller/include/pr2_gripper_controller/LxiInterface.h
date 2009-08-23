#ifndef LIBINST_LXI_INTERFACE_HPP
#define LIBINST_LXI_INTERFACE_HPP

#include "Interface.h"

/**
 * Implementation of interface over LXI (TCP/IP)
 * @author dking
 */
class LxiInterface : public Interface {
public:
  LxiInterface( const std::string &device_address, unsigned port=5025);
  LxiInterface(const LxiInterface &lxi);
  ~LxiInterface( );
  virtual void Open() throw (std::exception);
  virtual void Close() throw (std::exception);
  virtual void setAddress(int address) throw (std::exception);
  virtual void Write(const std::string &msg) throw (std::exception);
  virtual void Read(std::string &buf) throw (std::exception);
  virtual void WriteRead(const std::string &msg, std::string &buf) throw (std::exception);
protected:
  unsigned port;  
  int sock;
  std::string write_buf;
};

#endif //LIBINST_LXI_INTERFACE_HPP
