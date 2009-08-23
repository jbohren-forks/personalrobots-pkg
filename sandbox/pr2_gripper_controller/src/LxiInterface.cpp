#include <pr2_gripper_controller/LxiInterface.h>

#include <iostream>
#include <exception>
#include <sstream>
#include <stdexcept>

#include <sys/types.h>
#include <sys/select.h>
#include <sys/time.h>
#include <string.h>
#include <errno.h>

// TCP/IP sockets stuff
#include <sys/socket.h>
#include <bits/socket.h>
#include <netinet/tcp.h>
#include <netinet/in.h>
#include <arpa/inet.h>


using std::cout;
using std::cerr;
using std::endl;
using std::runtime_error;
using std::string;
using std::stringstream;

static const bool DEBUG = false;

LxiInterface::LxiInterface( const std::string &device_address, unsigned port) : 
  Interface(device_address), 
  port(port),
  sock(-1)
{  
  if (DEBUG) cerr << "LxiInterface constructor" << endl;
}

LxiInterface::LxiInterface(const LxiInterface &lxi) :
  Interface("Bad")
{
  if (DEBUG) cerr << "LxiInterface copy-constructor" << endl;
}


void LxiInterface::setAddress(int address) throw (std::exception)
{
  return; // LXI is not a bus, there is only one device on this connection
}

void LxiInterface::Write(const std::string &msg) throw (std::exception)
{
  if (DEBUG)
      cerr << "LxiInterface::Write msg = '" << msg << "' length = " << msg.length() << endl;  

  if(sock == -1) throw runtime_error("LxiInterface::Write : socket is not open");

  // for now just try send once.  
  // TODO : put a loop around send statement.  
  write_buf.reserve(msg.length() + 1);
  write_buf.assign(msg);
  write_buf.append("\n");
  int result = ::send(sock, write_buf.data(), write_buf.length(), MSG_DONTWAIT);
  if (result == -1) {
      int error = errno;
      stringstream ss;
      ss << "LxiInterface::Write : error sending '" << strerror(error) << '\'';
      throw runtime_error(ss.str());
  }
  else if (result != (int)(write_buf.length())) {
    stringstream ss;
    ss << "LxiInterface::Write : only " << result << " or " << write_buf.length() << " bytes got sent";
    throw runtime_error(ss.str());
  }
}

void LxiInterface::Read(std::string &buf) throw (std::exception)
{
  if(sock == -1) throw runtime_error("LxiInterface::Read : socket is not open");  

  buf.clear();

  {// Wait up to a second for recv
    fd_set readset;
    FD_ZERO(&readset);
    FD_SET(sock,&readset);
    struct timeval tv;
    tv.tv_sec=1; tv.tv_usec=0;
    int result=select(sock+1,&readset,NULL,NULL,&tv);
    if (result == -1) {
      int error = errno;
      stringstream ss;
      ss << "LxiInterface::Read : error select '" << strerror(error) << '\'';
      throw runtime_error(ss.str());
    }
    if (result == 0) {
        if (DEBUG) cerr << "LxiInterface::Read - select returned 0\n" << endl;
        return;
    }
  }
  
  { // read message data - assume it can be read all at once
    char cbuf[1500];
    int result = ::recv(sock, cbuf, sizeof(cbuf), MSG_DONTWAIT);
    if (result < 0) {
      int error = errno;
      stringstream ss;
      ss << "LxiInterface::Read : error recv'ing '" << strerror(error) << '\'';
      throw runtime_error(ss.str());
    }
    buf.reserve(result);
    buf.assign(cbuf, result);
    if (buf[buf.length()-1] != '\n') {
      stringstream ss;
      ss << "LxiInterface::Read : didn't recieve complete message '" << buf << "'"
         << " result = " << result;
      throw runtime_error(ss.str());
    }
  }    
}

void LxiInterface::WriteRead(const std::string &msg, std::string &buf) throw (std::exception) 
{
  if(sock == -1) throw runtime_error("LxiInterface::WriteRead : socket is not open");
  this->Write(msg);
  this->Read(buf);
}

void LxiInterface::Open( ) throw (std::exception) 
{  
  if (sock != -1) {
    throw runtime_error("LxiInterface::Open - Connection is already open\n");
  }
  
  sock = socket(PF_INET,SOCK_STREAM,0);
  if( sock == -1 ) {
      int error = errno;
      stringstream ss;
      ss << "LxiInterface::Open : error recv'ing '" << strerror(error) << '\'';
      throw runtime_error(ss.str());
  }  

  //cerr << "LxiInterface::Open sock = " << sock << endl;
  
  struct sockaddr_in addr;
  memset(&addr,0,sizeof(struct sockaddr_in)); 
  addr.sin_family=PF_INET; 
  addr.sin_port=htons(port); 
  if (inet_aton(this->device_string.c_str(), &addr.sin_addr)==0) {
      stringstream ss;
      ss << "LxiInterface::Open : error '" << device_string << "'is an invalid IPv4 address";
      throw runtime_error(ss.str());
  }
  
  { // Connect socket,,
    int result = connect(sock,(struct sockaddr *)&addr, sizeof(struct sockaddr_in));
    if (result == -1) {
      throw runtime_error("LxiInterface::Open : cannot connect socket");
    }
  }

  { // Turn off Nagle 
    int no_delay = 1; 
    int result=setsockopt(sock,IPPROTO_TCP, TCP_NODELAY, &no_delay, sizeof(no_delay));
    if (result == -1) {
      int error = errno;
      stringstream ss;
      ss << "LxiInterface::Open : error turning setting NoDelay TCP option '" << strerror(error) << '\'';
      throw runtime_error(ss.str());
    } 
  }

  //cerr << "LxiInterface::Open sock = " << sock << endl;
  
}
 
void LxiInterface::Close( ) throw (std::exception)
{
  //cerr << "LxiInterface::Close" << endl;
  if (sock != -1) {
    close(sock);
    sock = -1;
  }
  
}

LxiInterface::~LxiInterface( )
{
  //cerr << "LxiInterface destructor" << endl;
  this->Close();
}
