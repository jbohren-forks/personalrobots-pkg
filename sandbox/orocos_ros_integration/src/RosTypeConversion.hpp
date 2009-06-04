#ifndef ROS_TYPE_CONVERSION_HPP
#define ROS_TYPE_CONVERSION_HPP

#include <rtt/TemplateTypeInfo.hpp>

#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include "RosTypeTemplateConversion.hpp"

using namespace std_msgs;

namespace RTT
{
  //Float64 support
  template <>
  struct StdRosTypeConversion<double>:
    public RosTypeConversion<Float64,double>{
    static bool copyRosToOrocos(const Float64ConstPtr& ros,double& orocos){
      orocos=ros->data;
      return true;
    };
    static bool copyOrocosToRos(const double& orocos,Float64& ros){
      ros.data=orocos;
      return true;
    };
  };
  template <>
  struct StdRosTypeConversion<Float64>:
    public RosTypeConversion<Float64,Float64>{
    static bool copyRosToOrocos(const Float64ConstPtr& ros,Float64& orocos){
      orocos=(*ros);
      return true;
    };
    static bool copyOrocosToRos(const Float64& orocos,Float64& ros){
      ros=orocos;
      return true;
    };
  };
  struct Float64TypeInfo : public RTT::TemplateTypeInfo<Float64>{
    Float64TypeInfo():RTT::TemplateTypeInfo<Float64>("Float64")
    {}
  };



  //Float64MultiArray support
  template<>
  struct StdRosTypeConversion<std::vector<double> >:
    public RosTypeConversion<Float64MultiArray,std::vector<double> >{
    
    static bool copyRosToOrocos(const Float64MultiArrayConstPtr& ros,std::vector<double>& orocos){
      orocos=ros->data;
      return true;
    };
    static bool copyOrocosToRos(const std::vector<double>& orocos,Float64MultiArray& ros){
      ros.data=orocos;
      return true;
    };
  };
 template<>
  struct StdRosTypeConversion<Float64MultiArray >:
    public RosTypeConversion<Float64MultiArray,Float64MultiArray >{
    
    static bool copyRosToOrocos(const Float64MultiArrayConstPtr& ros,Float64MultiArray& orocos){
      orocos=(*ros);
      return true;
    };
    static bool copyOrocosToRos(const Float64MultiArray& orocos,Float64MultiArray& ros){
      ros=orocos;
      return true;
    };
  };
  
  struct Float64MultiArrayTypeInfo : public RTT::TemplateTypeInfo<Float64MultiArray>{
    Float64MultiArrayTypeInfo():RTT::TemplateTypeInfo<Float64MultiArray>("Float64MultiArray")
    {}
  };
  

}

#endif
