#include <rtt/TaskContext.hpp>
#include <rtt/Activities.hpp>
#include <rtt/Ports.hpp>
#include <ocl/ComponentLoader.hpp>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <ros/callback_queue.h>
#include "Orocos2Ros.hpp"

using namespace RTT;

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
  bool configureHook(){
    ros_node_ = RosNode::createRosNode(this);
    //np_act_ = new NonPeriodicActivity(ORO_SCHED_OTHER,OS::LowestPriority,ros_node_->engine());
    //p_act_ = new PeriodicActivity(ORO_SCHED_OTHER,OS::LowestPriority,0.1,ros_node_->engine());
    return true;
  };
  
  bool startHook(){
    ros_node_->start();
    return true;
  }
  
  void updateHook(){
    float_.data++;
    
    wport1.Set(0.1);
    wport3.Set(float_);
  };

  void stopHook(){
    ros_node_->stop();
  }
  void cleanupHook(){
    delete ros_node_;
    //delete np_act_;
    //delete p_act_;
  }
  
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
  
//  NonPeriodicActivity* np_act_;
//  PeriodicActivity* p_act_;

  TaskContext* ros_node_;
};

ORO_CREATE_COMPONENT_TYPE()
ORO_LIST_COMPONENT_TYPE( Component1 )
