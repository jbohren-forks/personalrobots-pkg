#ifndef ROS_TOOLKIT_HPP
#define ROS_TOOLKIT_HPP
#include <rtt/ToolkitPlugin.hpp>

class RosToolkitPlugin : public RTT::ToolkitPlugin{
public:
  virtual std::string getName();
  virtual bool loadTypes();
  virtual bool loadConstructors(){return true;};
  virtual bool loadOperators(){return true;};
};
  
extern RosToolkitPlugin RosToolkit;

#endif


