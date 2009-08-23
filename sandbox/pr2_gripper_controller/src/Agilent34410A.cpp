#include <pr2_gripper_controller/Agilent34410A.h>

#include <iostream>
#include <exception>
#include <sstream>
#include <stdexcept>

using std::cout;
using std::cerr;
using std::endl;
using std::runtime_error;
using std::invalid_argument;
using std::logic_error;
using std::string;
using std::stringstream;
using std::istringstream;
using std::ostringstream;

static const bool DEBUG = false;

Agilent34410A::Agilent34410A(Interface &driver, int address) throw() :
  driver(driver),
  address(address),
  current_mode(INVALID),
  last_range(-1.0)
{  
}

Agilent34410A::~Agilent34410A() 
{
}

void Agilent34410A::Initialize() throw(std::exception)
{
  if (DEBUG) cerr << "Agilent34410A::Initialize" << endl;
  Select();
  driver.Write("*CLS");  
  driver.Read(read_buf);
}

void Agilent34410A::Reset() throw(std::exception)
{
  if (DEBUG) cerr << "Agilent34410A::Reset" << endl;
  Select();
  driver.Write("*RST");
  
  if (usleep(1000*1000)) {
    perror("Agilent34410A::Reset : usleep : ");
  }
}

void Agilent34410A::RemoteMode(bool enable) throw(std::exception)
{
  Select();
}

void Agilent34410A::Identify(std::string &result) throw(std::exception)
{
  if (DEBUG) cerr << "Agilent34410A::Identify" << endl;
  Select();
  driver.Write("*IDN?");  
  driver.Read(result);  
}

void Agilent34410A::setMode(DMM_Mode new_mode) throw(std::exception)
{
  
  if ((current_mode != INVALID) && (new_mode == current_mode)) 
    return;

  Select();
  
  switch(new_mode) {
  case VOLTAGE:
    driver.Write("MEAS:VOLT:DC? AUTO");
    break;
  case CURRENT:
    driver.Write("MEAS:CURR:DC? AUTO");
    break;
  case RESISTANCE_2W:
    driver.Write("MEAS:RES? AUTO");
    break;
  case RESISTANCE_4W:
    driver.Write("MEAS:FRES? AUTO");
    break;
  default: 
    {
      ostringstream oss;
      oss << "Agilent34410A::setMode : invalid mode " << new_mode;
      throw invalid_argument(oss.str());
    }
    break;
  }
  last_range = -1;
  driver.Read(read_buf);
  current_mode = new_mode;
}

void Agilent34410A::Select() throw(std::exception)
{
  driver.setAddress(address); // set our own address so we are active
}

void Agilent34410A::Trigger() throw(std::exception)
{
  if (current_mode == INVALID) throw logic_error("Agilent34410A::Trigger : DMM mode needs to be set first");
  Select();
  driver.Write("INIT");
}

double Agilent34410A::Measure() throw(std::exception)
{
  if (DEBUG) cerr << "Agilent34410A::Measure" << endl;  
  if (current_mode == INVALID) throw logic_error("Agilent34410A::Measure : DMM mode needs to be set first");
  Select();
  driver.Write("READ?");
  driver.Read(read_buf);
  istringstream iss(read_buf);
  double result;
  iss >> result;
  if (!iss.good()) {    
    if (DEBUG) cerr << "Agilent34410A::Measure : could not convert '" << read_buf << "' to float" << endl;
    ostringstream oss;
    oss << "Agilent34410A::Measure : could not convert '" << read_buf << "' to float";
    if (DEBUG) cerr << oss.str() << endl;
    throw runtime_error(oss.str());
  }
  return result;
}

void Agilent34410A::setRange(double range) throw(std::exception)
{
  if (current_mode == INVALID) throw logic_error("Agilent34410A::setRange : DMM mode needs to be set first");
  Select();
  if (range != last_range) {      
    ostringstream oss;
    if(current_mode == CURRENT) {
      oss << "CURR:RANG " << range;
      driver.Write(oss.str());
    }
    else if(current_mode == VOLTAGE) {
      oss << "VOLT:RANG " << range;
      driver.Write(oss.str());
    }
    else {
      throw logic_error("Agilent34410A::setRange : cannot set range in current DMM mode");
    }	    
    
    last_range = range;
  }
}
