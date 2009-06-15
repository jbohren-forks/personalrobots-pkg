#include <rtt/TaskContext.hpp>
#include <rtt/Activities.hpp>
#include <rtt/Ports.hpp>
#include <ocl/ComponentLoader.hpp>

#include <std_msgs/Float64.h>


using namespace RTT;
using namespace std_msgs;

class Component1 : public RTT::TaskContext{
public:
  Component1(const std::string& name):
    TaskContext(name,PreOperational),
    rport1("ReadPort1"),
    rport2("ReadPort2"),
    rport3("ReadPort3"),
    wport1("WritePort1",0.0),
    wport2("WritePort2",std::vector<double>(6,0.0)),
    wport3("WritePort3")
  {
    this->ports()->addPort(&rport1);
    this->ports()->addPort(&rport2);
    this->ports()->addPort(&rport3);
    this->ports()->addPort(&wport1);
    this->ports()->addPort(&wport2);
    this->ports()->addPort(&wport3);
    
  };
  void updateHook(){
    float_.data++;
    
    wport1.Set(0.1);
    wport3.Set(float_);
  };

  
  ~Component1(){};
private:
  std::vector<double> array_;
  Float64 float_;
  ReadDataPort<double> rport1;
  ReadDataPort<std::vector<double> > rport2;
  ReadDataPort<Float64> rport3;
  WriteDataPort<double > wport1;
  WriteDataPort<std::vector<double> > wport2;
  WriteDataPort<Float64 > wport3;
  
};

ORO_CREATE_COMPONENT_TYPE()
ORO_LIST_COMPONENT_TYPE( Component1 )
